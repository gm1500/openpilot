"""Bounded speed previews from directed roads, measured motion and model geometry.

Preview connections never change RoadTracker's current match or junction graph.
Packed maps may supply a unique aligned endpoint continuation, not a fork inferred
from proximity. Actual shared nodes are required for model-assisted fork selection.
"""
import math
from dataclasses import dataclass

import numpy as np

from openpilot.selfdrive.mapd.osm_speed_limit import GpsFix, allowed_directions, offset_metres
from openpilot.selfdrive.mapd.speed_rules import road_speed


@dataclass(frozen=True)
class SpeedPreview:
  speed: float
  distance: float
  way_id: int


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
    current = match.speed
    if match.advisory_speed is not None or (current is None and
        (not match.road.tags.get('highway', '').endswith('_link') or
         any(k == 'maxspeed' or k.startswith('maxspeed:') for k in match.road.tags))):
      return None
    self._prepare(tracker)
    maximum = min(300., max(60., fix.speed * 8.))
    routes = []
    origin = fix

    def walk(road, forward, along, distance, points, target, visited):
      if len(routes) >= 16 or len(visited) > 8:
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
            walk(road, forward, along, distance, points, target, visited)
          elif id(other) not in visited:
            speed = road_speed(other.tags, direction, local_time=tracker._local_time)
            unknown = speed is None and (not other.tags.get('highway', '').endswith('_link') or
                                         any(k == 'maxspeed' or k.startswith('maxspeed:') for k in other.tags))
            if unknown:
              routes.append((points, None))
            else:
              upcoming = target
              if target is None and speed is not None and (current is None or abs(speed - current) > .01):
                upcoming = SpeedPreview(speed, distance, other.way_id)
              walk(other, direction, entry, distance, points, upcoming, visited | {id(other)})
        return
      routes.append((points, target))

    start = np.array(offset_metres(*match.position, origin))
    walk(match.road, match.forward, match.along, 0., [(0., start)], None, {id(match.road)})
    if not routes or len(routes) > 16:
      return None
    targets = {(r[1].way_id, r[1].speed, round(r[1].distance, 1)) if r[1] else None for r in routes}
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
    return SpeedPreview(target.speed, max(0., target.distance - advance), target.way_id)
