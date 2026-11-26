"""Bounded speed previews from directed roads, measured motion and model geometry.

Preview connections never change RoadTracker's current match or junction graph.
Packed maps may supply a unique aligned endpoint continuation, not a fork inferred
from proximity. Actual shared nodes are required for model-assisted fork selection.
"""
import math
from dataclasses import dataclass, replace

import numpy as np

from openpilot.selfdrive.mapd.osm_speed_limit import GpsFix, allowed_directions, offset_metres
from openpilot.selfdrive.mapd.speed_rules import road_speed


@dataclass(frozen=True)
class SpeedPreview:
  speed: float
  distance: float
  way_id: int
  advisory: bool = False
  merge: bool = False


@dataclass(frozen=True)
class ModelPath:
  points: np.ndarray  # east/north metres relative to the matched position observation
  std: np.ndarray
  origin: np.ndarray  # measured travel since that observation, not future model travel
  distances: np.ndarray  # cumulative path length, including measured travel
  heading_error: float = 3.  # bounded alignment sensitivity, not a probability


def model_path(sm, fix, motion, now):
  """Time-align the model with a recent GPS anchor using measured vehicle motion."""
  if fix is None or motion is None or fix.bearing is None or motion.speed < 2 or motion.turn_rate is None:
    return None
  gps_time = fix.gps_timestamp if fix.estimated else fix.timestamp
  if not 0 <= now - gps_time <= 1.2:
    return None  # no parked checkpoint or extended dead reckoning can predict a new road
  service = 'modelV2'
  if (service not in sm.services or not sm.valid[service] or
      not 0 <= now - sm.recv_time[service] <= .3 or not 0 <= now - sm.logMonoTime[service] * 1e-9 <= .3):
    return None
  model = sm[service]
  stamp = model.timestampEof * 1e-9
  age = stamp - fix.timestamp
  if not 0 <= now - stamp <= .3 or not -.2 <= age <= 1.2 or model.frameDropPerc > 20:
    return None
  p = model.position
  if not len(p.x) == len(p.y) == len(p.t) == len(p.yStd) == 33:
    return None
  x, y, t, std = (np.asarray(v, dtype=float) for v in (p.x, p.y, p.t, p.yStd))
  if (not all(np.isfinite(v).all() for v in (x, y, t, std)) or np.any(np.diff(t) <= 0) or
      np.any(np.diff(x) <= 0) or np.any(std < 0) or abs(x[0]) > 5 or abs(y[0]) > 5):
    return None
  keep = (t >= 2.) & (t <= 6.) & (x <= 180.) & (std <= 8.)
  if np.count_nonzero(keep) < 4:
    return None
  age = max(0., age)
  heading = math.radians(fix.bearing)
  turn = motion.turn_rate * age
  step = motion.speed * age
  origin = step * np.array([math.sin(heading + turn / 2), math.cos(heading + turn / 2)])
  heading += turn
  rotation = np.array([[math.sin(heading), math.cos(heading)], [math.cos(heading), -math.sin(heading)]])
  xy = np.column_stack((x - x[0], y - y[0]))
  distances = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))] + step
  points = xy[keep] @ rotation.T + origin
  return ModelPath(points, std[keep], origin, distances[keep])


def preview_speed(road, forward, local_time):
  """A legal restriction cannot be bypassed by an advisory fallback."""
  legal = road_speed(road.tags, forward, local_time=local_time)
  advisory = road_speed(road.tags, forward, advisory=True, local_time=local_time)
  legal_tags = any((key == 'maxspeed' or key.startswith('maxspeed:')) and
                   not key.startswith('maxspeed:advisory') for key in road.tags)
  if legal is None and legal_tags:
    return None, False
  if advisory is not None and legal is None:
    return advisory, True
  return legal, False


def merge_geometry_ready(tracker, match, road, forward, entry, prediction, points, speed):
  """Accelerate only on a straightened, unique approach to a highway merge."""
  if prediction is None or len(prediction.points) < 4 or np.median(prediction.std) > 3.0:
    return False
  nodes = tracker._nodes[id(road)]
  following = [i for i, (_, along) in enumerate(nodes) if (along - entry) * (1 if forward else -1) >= -.1]
  if not forward:
    following.reverse()
  if len(following) < 2:
    return False
  a, b = (road.geometry[i] for i in following[:2])
  east, north = offset_metres(*b, GpsFix(*a, None, 0., 0.))
  heading = math.degrees(math.atan2(east, north))
  if match.bearing is None or abs((heading - match.bearing + 180) % 360 - 180) > 15:
    return False
  distances, xy = zip(*points, strict=True)
  if distances[-1] < prediction.distances[-1]:
    return False
  xy = np.asarray(xy)
  expected = np.column_stack([np.interp(prediction.distances, distances, xy[:, i]) for i in range(2)])
  if np.max(np.linalg.norm(expected - prediction.points, axis=1)) > 8.0:
    return False
  steps = np.diff(prediction.points, axis=0)
  lengths = np.linalg.norm(steps, axis=1)
  if np.min(lengths) < 1.0:
    return False
  headings = np.unwrap(np.arctan2(steps[:, 0], steps[:, 1]))
  curvature = np.abs(np.diff(headings)) / ((lengths[:-1] + lengths[1:]) / 2)
  return bool(np.max(curvature) * speed ** 2 <= 1.5)


class SpeedPreviewer:
  def __init__(self):
    self._roads = None
    self._entries = {}

  def _prepare(self, tracker):
    if tracker._roads is self._roads:
      return
    self._roads = tracker._roads
    self._entries = {}
    for road in self._roads:
      if road.node_ids:
        continue
      for forward in allowed_directions(road):
        key = tuple(round(v, 7) for v in road.geometry[0 if forward else -1])
        self._entries.setdefault(key, []).append((road, forward, 0. if forward else tracker._nodes[id(road)][-1][1]))

  def _exits(self, tracker, road, forward, index):
    nodes = tracker._nodes[id(road)]
    node, along = nodes[index]
    remaining = nodes[-1][1] - along if forward else along
    exits = [(road, forward, along)] if remaining > .1 else []
    if road.node_ids:
      exits += [(other, direction, entry) for other, direction, entry in tracker._connections.get(node, ())
                if other != road and (tracker._nodes[id(other)][-1][1] - entry if direction else entry) > .1]
      return exits
    if remaining > .1 or not tracker._offline_continuity:
      return exits
    key = tuple(round(v, 7) for v in road.geometry[index])
    options = [p for p in self._entries.get(key, ()) if p[0] != road]
    # Count every possible exit before filtering identity/tangents. Incomplete
    # packed topology must never choose a fork by eliminating an awkward arm.
    if len(options) != 1:
      return []
    other, direction, entry = options[0]
    if tracker._continuations.get((id(road), forward)) == (other, direction):
      return options
    # Untagged one-way ramp -> its sole aligned one-way receiving road. This
    # inference supplies speed preview only, never an intersection/control target.
    if (not road.tags.get('highway', '').endswith('_link') or
        other.tags.get('highway', '').endswith('_link') or
        len(allowed_directions(road)) != 1 or len(allowed_directions(other)) != 1):
      return []
    old = road.geometry if forward else road.geometry[::-1]
    new = other.geometry if direction else other.geometry[::-1]
    before = np.array(offset_metres(*old[-2], GpsFix(*old[-1], None, 0., 0.)))
    after = np.array(offset_metres(*new[1], GpsFix(*new[0], None, 0., 0.)))
    norm = float(np.linalg.norm(before) * np.linalg.norm(after))
    # A ramp may join obliquely. Unlike same-road continuity (15 degrees),
    # allow a merge up to 45 degrees, but never a crossing/right-angle turn.
    return options if norm > 1 and float(-before @ after) / norm >= math.cos(math.radians(45)) else []

  def update(self, tracker, match, fix, prediction=None):
    if (match is None or match.forward is None or match.road.way_id <= 0 or fix.stationary or fix.speed < 2 or
        match.distance > 12 or match.heading_error is None or match.heading_error > 12 or
        fix.motion_bearing is not None and abs((match.bearing - fix.motion_bearing + 180) % 360 - 180) > 12):
      return None
    # A legal/advisory restriction on the ramp remains in force. Do not replace
    # an unresolved conditional limit with an apparently unrestricted successor.
    current, _ = preview_speed(match.road, match.forward, tracker._local_time)
    if match.advisory_speed is not None or (current is None and
        (not match.road.tags.get('highway', '').endswith('_link') or
         any(k == 'maxspeed' or k.startswith('maxspeed:') for k in match.road.tags))):
      return None
    self._prepare(tracker)
    # Untagged ramp pieces can span several hundred metres. Searching farther
    # does not apply a target farther away: cruise has its own bounded trigger.
    maximum = 1000. if current is None else 300.
    receiving = {}
    routes = []
    origin = fix

    def walk(road, forward, along, distance, points, target, visited, unique):
      if len(routes) >= 16 or len(visited) > 16:
        routes.append((points, None))
        return
      nodes = tracker._nodes[id(road)]
      for index in (range(len(nodes)) if forward else reversed(range(len(nodes)))):
        _, position = nodes[index]
        step = (position - along) * (1 if forward else -1)
        if step <= .1:
          continue
        point = np.array(offset_metres(*road.geometry[index], origin))
        points = points + [(distance + step, point)]
        distance += step
        along = position
        if distance >= maximum:
          routes.append((points, target))
          return
        exits = self._exits(tracker, road, forward, index)
        if len(exits) == 1 and exits[0][0] == road:
          continue
        if not exits:
          routes.append((points, target))
          return
        for other, direction, entry in exits:
          if other == road:
            walk(road, forward, along, distance, points, target, visited, unique and len(exits) == 1)
          elif id(other) not in visited:
            speed, advisory = preview_speed(other, direction, tracker._local_time)
            unknown = speed is None and (not other.tags.get('highway', '').endswith('_link') or
                                         any(k == 'maxspeed' or k.startswith('maxspeed:') for k in other.tags))
            if unknown:
              routes.append((points, None))
            else:
              upcoming = target
              if target is None and speed is not None and (current is None or abs(speed - current) > .01):
                merging = (unique and len(exits) == 1 and current is None and not advisory and
                           other.tags.get('highway') in ('motorway', 'trunk') and len(allowed_directions(other)) == 1 and
                           not any(k.startswith('maxspeed:advisory') for k in other.tags))
                upcoming = SpeedPreview(speed, distance, other.way_id, advisory, merging)
                receiving[other.way_id] = other, direction, entry
              walk(other, direction, entry, distance, points, upcoming, visited | {id(other)}, unique and len(exits) == 1)
        return
      routes.append((points, target))

    start = np.array(offset_metres(*match.position, origin))
    walk(match.road, match.forward, match.along, 0., [(0., start)], None, {id(match.road)}, True)
    if not routes or len(routes) > 16:
      return None
    targets = {(r[1].way_id, r[1].speed, round(r[1].distance, 1), r[1].advisory, r[1].merge) if r[1] else None for r in routes}
    if len(targets) == 1:
      target = routes[0][1]
    else:
      if prediction is None:
        return None
      # Compare all alternatives, including ones without a known speed. Equal
      # speed values are not evidence that two branches are interchangeable.
      distances = prediction.distances
      shapes = []
      for points, _ in routes:
        if points[-1][0] < float(distances[-1]):
          return None  # do not reward a truncated/incomplete competing road
        s, xy = zip(*points, strict=True)
        xy = np.array(xy)
        shapes.append(np.column_stack([np.interp(distances, s, xy[:, i]) for i in range(2)]))
      shapes = np.array(shapes)
      separation = np.max(np.linalg.norm(shapes - shapes[0], axis=2), axis=0)
      mask = separation >= 6.
      if np.count_nonzero(mask) < 3:
        return None
      winner = None
      for angle in (-prediction.heading_error, 0., prediction.heading_error):
        angle = math.radians(angle)
        rotation = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
        projected = (prediction.points - prediction.origin) @ rotation.T + prediction.origin
        errors = np.linalg.norm(shapes[:, mask] - projected[mask], axis=2)
        scores = np.sqrt(np.mean((errors / (3. + prediction.std[mask]))**2, axis=1))
        order = np.argsort(scores)
        best = int(order[0])
        if scores[best] > 2.5 or scores[order[1]] - scores[best] < .5 or winner is not None and winner != best:
          return None
        winner = best
      target = routes[winner][1]
    if target is None or not 0 < target.distance <= maximum or not 8 / 3.6 <= target.speed <= 145 / 3.6:
      return None
    # The advance is observed wheel motion to the model's capture time. Future
    # model travel may select a candidate but never advances the boundary clock.
    advance = float(np.linalg.norm(prediction.origin)) if prediction is not None else 0.
    if target.merge:
      road, direction, entry = receiving[target.way_id]
      route_points = next(points for points, candidate in routes if candidate == target)
      ready = fix.speed >= 10.0 and merge_geometry_ready(tracker, match, road, direction, entry, prediction, route_points, target.speed)
      target = replace(target, merge=ready)
    return replace(target, distance=max(0., target.distance - advance))
