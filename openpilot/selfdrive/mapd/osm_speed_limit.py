"""OSM road matching with fresh legal/advisory speeds shared by display and SET."""
import json
import math
import re
import threading
import time
from dataclasses import dataclass, field
from itertools import pairwise

import requests

OVERPASS_URL = "https://overpass-api.de/api/interpreter"
QUERY_RADIUS = 1500.0  # metres; prefetch again after travelling 700 m
QUERY_INTERVAL = 30.0
CACHE_TTL = 600.0
GPS_MAX_AGE = 3.0
ACQUIRE_RADIUS = 25.0
TRACK_RADIUS = 45.0
QCOM_UNKNOWN_ACCURACY = 15.0  # Matching allowance, not a measured GPS uncertainty
MAX_RESPONSE_BYTES = 2 * 1024 * 1024
ROAD_TYPES = ("motorway", "trunk", "primary", "secondary", "tertiary", "unclassified", "residential", "living_street", "service", "road",
              "motorway_link", "trunk_link", "primary_link", "secondary_link", "tertiary_link")
EARTH_RADIUS = 6371000.0
NodeKey = int | tuple[int, int]


@dataclass(frozen=True)
class GpsFix:
  latitude: float
  longitude: float
  bearing: float | None
  accuracy: float
  timestamp: float
  stationary: bool = False
  motion_bearing: float | None = None
  speed: float = 0.
  odometer: float | None = None
  motion_epoch: float | None = None
  estimated: bool = False
  gps_timestamp: float = 0.


@dataclass(frozen=True)
class Road:
  geometry: tuple[tuple[float, float], ...]
  tags: dict[str, str]
  way_id: int = 0
  node_ids: tuple[int, ...] = ()
  control_tags: dict[int, dict[str, str]] = field(default_factory=dict)
  bounds: tuple[float, float, float, float] = field(init=False, repr=False)

  def __post_init__(self):
    latitudes, longitudes = zip(*self.geometry, strict=True)
    object.__setattr__(self, "bounds", (min(latitudes), max(latitudes), min(longitudes), max(longitudes)))


@dataclass(frozen=True)
class RoadMatch:
  road: Road
  distance: float
  speed: float | None
  heading_error: float | None
  along: float
  forward: bool | None
  position: tuple[float, float]
  bearing: float | None = None

  @property
  def advisory_speed(self) -> float | None:
    return road_speed(self.road.tags, self.forward, advisory=True)


@dataclass(frozen=True)
class TrafficControl:
  node_id: int
  way_id: int
  kind: str
  distance: float


def gps_fix(sm, started_frame: int, now: float) -> GpsFix | None:
  fixes = []
  for source in ("gpsLocationExternal", "gpsLocation"):
    gps = sm[source]
    timestamp = min(sm.recv_time[source], sm.logMonoTime[source] * 1e-9)
    if not sm.valid[source] or sm.recv_frame[source] < started_frame or not gps.hasFix or not 0 <= now - timestamp <= GPS_MAX_AGE:
      continue
    if not (math.isfinite(gps.latitude) and math.isfinite(gps.longitude) and -85 < gps.latitude < 85 and -180 <= gps.longitude <= 180):
      continue
    accuracy = gps.horizontalAccuracy
    # qcomgpsd does not populate horizontalAccuracy. Zero means unavailable for
    # this source, not an invalid fix (or perfect accuracy). Keep the workaround
    # local to this map matcher and use a conservative ambiguity allowance.
    if accuracy == 0 and source == "gpsLocation" and str(gps.source) == "qcomdiag":
      accuracy = QCOM_UNKNOWN_ACCURACY
    if not math.isfinite(accuracy) or not 0 < accuracy <= 15:
      continue
    bearing = None
    if gps.speed >= 2 and math.isfinite(gps.bearingDeg) and 0 <= gps.bearingDeg < 360 and 0 <= gps.bearingAccuracyDeg <= 30:
      bearing = gps.bearingDeg
    speed = gps.speed if math.isfinite(gps.speed) and 0 <= gps.speed <= 75 else 0.
    fixes.append(GpsFix(gps.latitude, gps.longitude, bearing, accuracy, timestamp, speed=speed))
  return min(fixes, key=lambda fix: fix.accuracy) if fixes else None


def offset_metres(latitude: float, longitude: float, origin: GpsFix) -> tuple[float, float]:
  longitude_delta = (longitude - origin.longitude + 180) % 360 - 180
  return (math.radians(longitude_delta) * EARTH_RADIUS * math.cos(math.radians(origin.latitude)),
          math.radians(latitude - origin.latitude) * EARTH_RADIUS)


def parse_speed(value: str) -> float | None:
  """OSM numbers default to km/h. Do not invent limits for implicit/variable tags."""
  match = re.fullmatch(r"([0-9]+(?:\.[0-9]+)?)\s*(km/h|kmh|kph|mph)?", value.strip().lower())
  if match is None:
    return None
  speed = float(match[1]) * (0.44704 if match[2] == "mph" else 1 / 3.6)
  return speed if 0 < speed <= 300 / 3.6 else None


def road_speed(tags: dict[str, str], forward: bool | None, advisory: bool = False) -> float | None:
  # Advisory restrictions are independent of the legal limit. Reject unsupported
  # qualifiers within each family, rather than hiding both when one is present.
  prefix = "maxspeed:advisory" if advisory else "maxspeed"
  supported = {prefix, f"{prefix}:forward", f"{prefix}:backward", f"{prefix}:type"}
  qualified = (key for key in tags if key.startswith(f"{prefix}:"))
  if not advisory:
    qualified = (key for key in qualified if key != "maxspeed:advisory" and not key.startswith("maxspeed:advisory:"))
  if any(key not in supported for key in qualified):
    return None
  base = tags.get(prefix, "")
  ahead = parse_speed(tags.get(f"{prefix}:forward", base))
  behind = parse_speed(tags.get(f"{prefix}:backward", base))
  if forward is None:
    return ahead if ahead == behind else None
  return ahead if forward else behind


def road_candidates(roads: tuple[Road, ...], fix: GpsFix, radius: float = ACQUIRE_RADIUS) -> list[RoadMatch]:
  candidates = []
  latitude_margin = math.degrees(radius / EARTH_RADIUS)
  longitude_margin = latitude_margin / math.cos(math.radians(fix.latitude))
  for road in roads:
    south, north, west, east = road.bounds
    if not south - latitude_margin <= fix.latitude <= north + latitude_margin:
      continue
    # Skip the longitude shortcut across the date line; local offsets wrap below.
    if east - west < 180 and abs(fix.longitude) + longitude_margin < 180 and not west - longitude_margin <= fix.longitude <= east + longitude_margin:
      continue
    nearest = None
    along = 0.
    points = [offset_metres(lat, lon, fix) for lat, lon in road.geometry]
    for (ax, ay), (bx, by) in pairwise(points):
      dx, dy = bx - ax, by - ay
      length_sq = dx * dx + dy * dy
      length = math.sqrt(length_sq)
      segment_start = along
      along += length
      if length_sq < 1:
        continue
      projection = max(0., min(1., -(ax * dx + ay * dy) / length_sq))
      distance = math.hypot(ax + projection * dx, ay + projection * dy)
      if distance > radius:
        continue
      forward = None
      heading_error = None
      if fix.bearing is not None:
        heading = math.degrees(math.atan2(dx, dy)) % 360
        delta = abs((heading - fix.bearing + 180) % 360 - 180)
        heading_error = min(delta, 180 - delta)
        forward = delta <= 90
        if heading_error > 40 or forward not in allowed_directions(road):
          continue
      if nearest is None or distance < nearest.distance:
        x, y = ax + projection * dx, ay + projection * dy
        position = (fix.latitude + math.degrees(y / EARTH_RADIUS),
                    (fix.longitude + math.degrees(x / (EARTH_RADIUS * math.cos(math.radians(fix.latitude)))) + 180) % 360 - 180)
        nearest = RoadMatch(road, distance, road_speed(road.tags, forward), heading_error,
                            segment_start + projection * length, forward, position,
                            (heading if forward else (heading + 180) % 360) if forward is not None else None)
    if nearest is not None:
      candidates.append(nearest)
  return sorted(candidates, key=lambda candidate: candidate.distance)


def road_nodes(road: Road) -> list[tuple[NodeKey, float]]:
  origin = GpsFix(*road.geometry[0], None, 0., 0.)
  points = [offset_metres(*point, origin) for point in road.geometry]
  distances = [0.]
  for (ax, ay), (bx, by) in pairwise(points):
    distances.append(distances[-1] + math.hypot(bx - ax, by - ay))
  # Coordinate coincidence is not a junction: bridges can cross at the same
  # location. Missing node references permit same-way tracking only.
  keys = road.node_ids if len(road.node_ids) == len(road.geometry) else tuple((id(road), i) for i in range(len(points)))
  return list(zip(keys, distances, strict=True))


def allowed_directions(road: Road) -> tuple[bool, ...]:
  oneway = road.tags.get('oneway', 'yes' if road.tags.get('junction') == 'roundabout' else 'no')
  return (True,) if oneway in ('yes', '1', 'true') else (False,) if oneway == '-1' else (True, False)


def control_kind(tags: dict[str, str]) -> str | None:
  if tags.get('highway') == 'stop':
    return 'stopSign'
  if tags.get('highway') == 'traffic_signals' or (tags.get('highway') == 'crossing' and tags.get('crossing') == 'traffic_signals'):
    return 'trafficLight'
  return None


class RoadTracker:
  """Compare bounded connected paths across fresh fixes before selecting a limit."""
  def __init__(self):
    self.reset()

  def reset(self):
    self.match: RoadMatch | None = None
    self.fix: GpsFix | None = None
    self._roads: tuple[Road, ...] = ()
    self._connections: dict[NodeKey, list[tuple[Road, bool, float]]] = {}
    self._nodes: dict[int, list[tuple[NodeKey, float]]] = {}
    self._processed: GpsFix | None = None
    self._result: RoadMatch | None = None
    self._stop_anchor: GpsFix | None = None
    self._paths: list[tuple[RoadMatch, float, float, float]] = []
    self._path_fix: GpsFix | None = None
    self._winner: Road | None = None
    self._winner_since = self._winner_travel = 0.
    self.control_reason = 'noRoadMatch'

  def _set_roads(self, roads: tuple[Road, ...]):
    self._roads = roads
    self._connections = {}
    self._nodes = {}
    for road in roads:
      self._nodes[id(road)] = road_nodes(road)
      # Junctions can be interior nodes of a way, not just its endpoints.
      for node, along in self._nodes[id(road)]:
        for forward in allowed_directions(road):
          self._connections.setdefault(node, []).append((road, forward, along))

    def refresh(previous: RoadMatch, fix: GpsFix) -> RoadMatch | None:
      old = previous.road
      same = next((road for road in roads if (road.way_id == old.way_id if old.way_id else road == old)
                   and road.geometry == old.geometry), None)
      projected = road_candidates((same,), fix, radius=TRACK_RADIUS) if same is not None else []
      return projected[0] if projected and projected[0].forward == previous.forward else None

    # Cached ways are replaced atomically. Retain only unchanged geometry and
    # direction, reprojecting both the accepted road and competing paths so new
    # speed tags apply immediately.
    if self.match is not None and self.fix is not None:
      self.match = refresh(self.match, self.fix)
      if self.match is None:
        self.fix = self._stop_anchor = None
    refreshed = []
    if self._path_fix is not None:
      for previous, score, since, travelled in self._paths:
        projected = refresh(previous, self._path_fix)
        if projected is not None:
          refreshed.append((projected, score, since, travelled))
    self._paths = refreshed

  @staticmethod
  def _lateral_offset(match: RoadMatch, fix: GpsFix) -> float:
    x, y = offset_metres(*match.position, fix)
    angle = math.radians(match.bearing)
    return -x * math.cos(angle) + y * math.sin(angle)

  @staticmethod
  def _heading_cost(match: RoadMatch, fix: GpsFix) -> float:
    cost = (match.heading_error / 8.) ** 2
    if fix.motion_bearing is not None:
      error = abs((match.bearing - fix.motion_bearing + 180) % 360 - 180)
      cost += .5 * (error / 8.) ** 2
    return cost

  def _observation(self, match: RoadMatch, fix: GpsFix) -> float:
    return .4 * (match.distance / max(5., fix.accuracy)) ** 2 + self._heading_cost(match, fix)

  def _walk(self, candidates: list[RoadMatch], fix: GpsFix) -> RoadMatch | None:
    """Keep competing continuations; a nearer parallel ramp is not a shortcut.

    Score changes in cross-track offset, directed progress and heading. Absolute
    centreline distance has less weight: GPS bias and lane position can persist
    across a fork. Scores forget old evidence over three seconds.
    """
    previous_fix = self._path_fix
    dt = fix.timestamp - previous_fix.timestamp if previous_fix is not None else 0.
    moved = math.hypot(*offset_metres(fix.latitude, fix.longitude, previous_fix)) if previous_fix is not None else 0.
    if not 0 < dt <= GPS_MAX_AGE or moved > 75 * dt + 10:
      self._paths = []
      self._winner = None
    if (previous_fix is not None and fix.motion_epoch is not None and fix.motion_epoch == previous_fix.motion_epoch and
        fix.odometer is not None and previous_fix.odometer is not None):
      distance = fix.odometer - previous_fix.odometer
      if 0 <= distance <= 75 * dt + 10:
        moved = distance
    ranked = []
    plausible_motion = not self._paths
    minimum = min((p[1] for p in self._paths), default=0.)
    for current in candidates[:8]:
      if current.bearing is None or current.heading_error is None:
        continue
      x, y = offset_metres(*current.position, fix)
      angle = math.radians(current.bearing)
      if abs(x * math.sin(angle) + y * math.cos(angle)) > 3:
        continue  # do not walk beyond an endpoint by clamping to its last node
      observation = self._observation(current, fix)
      best = None
      for previous, score, since, travelled in self._paths:
        maximum = max(15., moved * 1.6 + 8.)
        if fix.motion_bearing is not None and previous.road != current.road:
          maximum = max(maximum, moved + 2 * fix.accuracy)
        progress = self._progress(previous, current, maximum)
        if progress is None or not -3 <= progress <= maximum:
          continue
        lateral_change = self._lateral_offset(current, fix) - self._lateral_offset(previous, previous_fix)
        allowance = max(4., moved * .25)
        if fix.motion_bearing is not None and previous_fix.motion_bearing is not None:
          allowance = max(allowance, min(15., fix.accuracy))
        plausible_motion |= abs(lateral_change) <= allowance
        history = min(3., score - minimum)
        cost = (math.exp(-dt / 3.) * history + observation + (lateral_change / 3.) ** 2 +
                ((progress - moved) / max(6., moved * .3)) ** 2)
        option = (current, cost, since, travelled + moved)
        if best is None or cost < best[1]:
          best = option
      if best is None and not self._paths:
        best = (current, observation, fix.timestamp, 0.)
      if best is not None:
        ranked.append(best)
    ranked.sort(key=lambda path: path[1])
    if not plausible_motion:
      ranked = []
    self._paths = ranked[:6]
    self._path_fix = fix
    if not ranked:
      self._winner = None
      return None
    best, score, since, travelled = ranked[0]
    walked = {(id(p[0].road), p[0].forward) for p in ranked}
    if any((id(c.road), c.forward) not in walked and c.bearing is not None and
           self._heading_cost(c, fix) + 1 < self._heading_cost(best, fix) for c in candidates):
      return None  # motion contradicts the connected paths; never force a match
    conflicts = [c for c in candidates if (c.speed, c.advisory_speed) != (best.speed, best.advisory_speed)]
    for other in conflicts:
      # An incoming ramp is not an alternative forward path at a merge.
      if ((id(other.road), other.forward) in walked and
          math.hypot(*offset_metres(*other.position, GpsFix(*best.position, None, 0., 0.))) < 1.5):
        return None  # still at the shared fork, before either branch is established
    if conflicts and moved < 3 and (best != candidates[0] or best.distance > 8 or best.heading_error > 15 or
                                    any(c.distance < best.distance + 3 for c in conflicts)):
      return None
    rivals = [p for p in ranked[1:] if (p[0].speed, p[0].advisory_speed) != (best.speed, best.advisory_speed)]
    if rivals and (fix.timestamp - since < .5 or travelled < 5 or rivals[0][1] - score < .25):
      return None
    # Commit a branch after sustained separated evidence. Keeping defeated
    # branches indefinitely lets a later GPS offset jump back across a fork.
    if self._winner == best.road:
      self._winner_travel += moved
    else:
      self._winner, self._winner_since, self._winner_travel = best.road, fix.timestamp, 0.
    if (fix.timestamp - self._winner_since >= 1 and self._winner_travel >= 20 and
        (len(ranked) == 1 or ranked[1][1] - score >= 2.)):
      self._paths = [ranked[0]]
    return best

  def _progress(self, previous: RoadMatch, current: RoadMatch, maximum: float) -> float | None:
    if previous.forward is None or current.forward is None:
      return None
    if current.road == previous.road:
      return (current.along - previous.along) * (1 if previous.forward else -1) if current.forward == previous.forward else None
    # A mapped ramp endpoint can lie beyond the physical merge. A lane change
    # may reach the incoming mainline before crossing that node. Allow only a
    # nearby, same-direction approach to ONE shared continuation with the same
    # legal limit; never bypass a ramp advisory, fork or unrelated parallel road.
    if (previous.road.tags.get('highway', '').endswith('_link') and
        not any(k == 'maxspeed' or k.startswith('maxspeed:') for k in previous.road.tags) and
        not current.road.tags.get('highway', '').endswith('_link') and current.speed is not None and
        current.distance <= 15 and current.heading_error is not None and current.heading_error <= 15):
      node, end = self._nodes[id(previous.road)][-1 if previous.forward else 0]
      remaining = (end - previous.along) * (1 if previous.forward else -1)
      entries = [(along - current.along) * (1 if current.forward else -1)
                 for key, along in self._nodes[id(current.road)] if key == node]
      exits = [(road, forward) for road, forward, along in self._connections[node]
               if (self._nodes[id(road)][-1][1] - along if forward else along) > .1]
      if (0 <= remaining <= 60 and len(exits) == 1 and
          road_speed(exits[0][0].tags, exits[0][1]) == current.speed):
        for approach in entries:
          progress = remaining - approach
          if 0 <= approach <= 25 and -3 <= progress <= maximum:
            return progress
    # Connected short ways may be crossed between 1 Hz GPS fixes. Bound both
    # distance and hops; do not traverse an arbitrary route through a junction.
    pending = [(node, max(0., (along - previous.along) * (1 if previous.forward else -1)), 0)
               for node, along in self._nodes[id(previous.road)]
               if -0.1 <= (along - previous.along) * (1 if previous.forward else -1) <= maximum]
    visited = {}
    while pending:
      node, distance, hops = pending.pop()
      if distance > maximum or hops >= 4 or distance >= visited.get(node, math.inf):
        continue
      visited[node] = distance
      for road, forward, entry_along in self._connections.get(node, ()):
        if road == previous.road:
          continue
        if road == current.road and forward == current.forward:
          remaining = (current.along - entry_along) * (1 if forward else -1)
          progress = distance + remaining
          if remaining >= -0.1 and progress <= maximum:
            return progress
        for exit_node, exit_along in self._nodes[id(road)]:
          step = (exit_along - entry_along) * (1 if forward else -1)
          if 0 < step <= maximum - distance:
            pending.append((exit_node, distance + step, hops + 1))
    return None

  def merge_limit(self, match: RoadMatch, fix: GpsFix) -> tuple[float | None, float]:
    """Look ahead from an untagged ramp along one directed continuation only."""
    road, forward, along = match.road, match.forward, match.along
    maximum = min(250., max(40., fix.speed * 8.))
    distance = 0.
    visited = set()
    for _ in range(8):
      if (forward is None or not road.tags.get('highway', '').endswith('_link') or
          any(k == 'maxspeed' or k.startswith('maxspeed:') for k in road.tags) or id(road) in visited):
        return None, 0.
      visited.add(id(road))
      nodes = self._nodes[id(road)]
      length = nodes[-1][1]
      next_road = None
      for node, position in (nodes if forward else reversed(nodes)):
        step = (position - along) * (1 if forward else -1)
        if step < -.1:
          continue
        continuations = []
        if (length - position if forward else position) > .1:
          continuations.append((road, forward, position))
        for other, direction, entry in self._connections.get(node, ()):
          remaining = self._nodes[id(other)][-1][1] - entry if direction else entry
          if other != road and remaining > .1:
            continuations.append((other, direction, entry))
        if len(continuations) > 1:
          return None, 0.  # an interior junction is a fork too
        if continuations and continuations[0][0] != road:
          distance += max(0., step)
          next_road = continuations[0]
          break
      if next_road is None or distance > maximum:
        return None, 0.
      road, forward, along = next_road
      limit = road_speed(road.tags, forward)
      if limit is not None:
        return limit, distance
    return None, 0.

  def _control_applies(self, road: Road, node: NodeKey, forward: bool) -> bool:
    tags = road.control_tags.get(node, {})
    kind = control_kind(tags)
    if kind is None:
      return False
    direction = tags.get('stop:direction' if kind == 'stopSign' else 'traffic_signals:direction', tags.get('direction', ''))
    if direction:
      return direction == 'both' or direction == ('forward' if forward else 'backward')
    # This node marks the signal-controlled crossing on the drivable way,
    # not a particular signal head. Both approaches need map awareness unless
    # explicitly restricted. Footway-only nodes are excluded during the fetch.
    if tags.get('highway') == 'crossing':
      return True
    if len(allowed_directions(road)) == 1:
      return True
    # A central junction signal applies to the incoming roads. Undirected
    # two-way approach nodes are ambiguous; do not borrow the opposing stop.
    neighbours = set()
    for other, _, _ in self._connections.get(node, ()):
      nodes = other.node_ids
      for index, key in enumerate(nodes):
        if key == node:
          neighbours.update(nodes[max(0, index - 1):index])
          neighbours.update(nodes[index + 1:index + 2])
    return len(neighbours) >= 3 and (kind == 'trafficLight' or tags.get('stop') == 'all')

  def _junction_bearing(self, road: Road, forward: bool, along: float, outgoing: bool) -> float | None:
    """Use up to 15 m of geometry on the requested side of a connected node."""
    nodes = self._nodes[id(road)]
    index = next(i for i, (_, position) in enumerate(nodes) if abs(position - along) < .01)
    origin = GpsFix(*road.geometry[index], None, 0., 0.)
    step = 1 if forward == outgoing else -1
    for other in range(index + step, len(nodes) if step > 0 else -1, step):
      x, y = offset_metres(*road.geometry[other], origin)
      if math.hypot(x, y) >= 3:
        if abs(nodes[other][1] - along) >= 15 or other in (0, len(nodes) - 1):
          return math.degrees(math.atan2(x, y)) % 360 if outgoing else math.degrees(math.atan2(-x, -y)) % 360
    return None

  def _control_continuation(self, road, forward, position, exits):
    if len(exits) <= 1:
      return exits[0] if exits else None
    # A side street must not hide a signal on the clearly continuing road.
    # Shallow splits, turns, and roundabouts still require a known route.
    incoming = self._junction_bearing(road, forward, position, False)
    if incoming is None or any(r.tags.get('junction') == 'roundabout' for r, _, _ in exits):
      return None
    ranked = []
    for exit_road, direction, entry in exits:
      bearing = self._junction_bearing(exit_road, direction, entry, True)
      if bearing is None:
        return None
      ranked.append((abs((bearing - incoming + 180) % 360 - 180), (exit_road, direction, entry)))
    ranked.sort(key=lambda item: item[0])
    return ranked[0][1] if ranked[0][0] <= 25 and ranked[1][0] - ranked[0][0] >= 30 else None

  def _next_control(self, match: RoadMatch, maximum: float = 1000.) -> tuple[TrafficControl | None, str]:
    road, forward, along = match.road, match.forward, match.along
    distance = 0.
    visited = set()
    reason = 'noControl'
    for hop in range(16):
      if forward is None or id(road) in visited:
        return None, 'lookaheadLimit'
      visited.add(id(road))
      nodes = self._nodes[id(road)]
      next_road = None
      for node, position in (nodes if forward else reversed(nodes)):
        step = (position - along) * (1 if forward else -1)
        if step < (-20. if hop == 0 else -.1):
          continue
        total = distance + step
        if total > maximum:
          return None, 'lookaheadLimit'
        if self._control_applies(road, node, forward):
          kind = control_kind(road.control_tags[node])
          return TrafficControl(node, road.way_id, kind, total), 'target'
        if node in road.control_tags:
          reason = 'controlDirection'
        if step < -.1:
          continue
        if hop > 0 and abs(step) < .1:
          continue  # this entry node's outgoing direction was selected on the previous way
        exits = []
        if (nodes[-1][1] - position if forward else position) > .1:
          exits.append((road, forward, position))
        for other, direction, entry in self._connections.get(node, ()):
          remaining = self._nodes[id(other)][-1][1] - entry if direction else entry
          if other != road and id(other) not in visited and remaining > .1:
            exits.append((other, direction, entry))
        continuation = self._control_continuation(road, forward, position, exits)
        if len(exits) > 1 and continuation is None:
          return None, 'ambiguousFork'
        if continuation is not None and continuation[0] != road:
          next_road = continuation
          distance = total
          break
      if next_road is None:
        return None, reason
      road, forward, along = next_road
    return None, 'lookaheadLimit'

  def traffic_control(self, match: RoadMatch) -> TrafficControl | None:
    result, self.control_reason = self._next_control(match)
    if match.distance > 20 or match.heading_error is None or match.heading_error > 20:
      self.control_reason = 'roadAlignment'
      return None
    if result is None:
      return None
    # Speed-limit equality does not make two roads equivalent for stopping.
    # Require plausible competing paths to agree on the control as well.
    best_score = min((p[1] for p in self._paths), default=0.)
    for other, score, _, _ in self._paths:
      if score <= best_score + 2. and other != match:
        alternative, _ = self._next_control(other)
        if alternative is None or alternative.node_id != result.node_id or abs(alternative.distance - result.distance) > 20:
          self.control_reason = 'ambiguousRoad'
          return None
    return result

  def update(self, roads: tuple[Road, ...], fix: GpsFix) -> RoadMatch | None:
    refreshed = roads is not self._roads
    if refreshed:
      self._set_roads(roads)
    if fix == self._processed:
      if refreshed and self._result is not None:
        old = self._result.road
        self._result = next((p[0] for p in self._paths if p[0].road.way_id == old.way_id and
                             p[0].road.geometry == old.geometry), None)
      return self._result  # repeated polling is not new motion evidence
    self._processed = fix
    if self.fix is not None:
      dt = fix.timestamp - self.fix.timestamp
      travelled = math.hypot(*offset_metres(fix.latitude, fix.longitude, self.fix))
      if not 0 <= dt <= GPS_MAX_AGE or travelled > 75 * dt + 10:
        self.match = self.fix = None
        self._stop_anchor = None
    candidates = road_candidates(roads, fix, radius=TRACK_RADIUS)
    # An established connected path can survive a bounded lateral GPS offset.
    # Unconnected roads still need normal acquisition within 25 metres.
    maximum = max(15., fix.speed * GPS_MAX_AGE * 1.6 + 8.)
    candidates = [c for c in candidates if c.distance <= ACQUIRE_RADIUS or
                  (self.match is not None and c.heading_error is not None and c.heading_error <= 12 and
                   (fix.motion_bearing is not None or fix.stationary and c.road == self.match.road) and
                   self._progress(self.match, c, maximum) is not None)]
    if not fix.stationary:
      self._stop_anchor = None
    elif self.match is not None and fix.bearing is not None:
      # A fresh CAN standstill and bounded GPS drift preserve the established
      # road/direction through a stop. Never walk the anchor along noisy fixes.
      if self._stop_anchor is None:
        self._stop_anchor = self.fix
      stopped = next((c for c in candidates if c.road == self.match.road and c.forward == self.match.forward), None)
      drift = math.hypot(*offset_metres(fix.latitude, fix.longitude, self._stop_anchor))
      if stopped is not None and drift <= 12:
        self.match, self.fix, self._result = stopped, fix, stopped
        self._paths = [(stopped, 0., fix.timestamp, 0.)]
        self._path_fix = fix
        return stopped
      self.match = self.fix = self._stop_anchor = None
      self._result = self._path_fix = None
      self._paths = []
      return None
    result = self._walk(candidates, fix)
    if result is not None:
      self.match, self.fix = result, fix
    else:
      self._winner = None
    self._result = result
    return result


def fetch_roads(fix: GpsFix) -> tuple[Road, ...]:
  query = f'[out:json][timeout:10][maxsize:16777216];way(around:{QUERY_RADIUS:.0f},{fix.latitude:.6f},{fix.longitude:.6f})'
  query += f'["highway"~"^({"|".join(ROAD_TYPES)})$"]->.roads;.roads out body geom;'
  query += '(node(w.roads)["highway"~"^(stop|traffic_signals)$"];'
  query += 'node(w.roads)["highway"="crossing"]["crossing"="traffic_signals"];);out body;'
  deadline = time.monotonic() + 20
  # One request at a time, bounded response size, no identifiers or route history.
  with requests.post(OVERPASS_URL, data={"data": query}, timeout=(3.05, 12), stream=True,
                     headers={"User-Agent": "boat-anchor-speed-limit/1.0 (https://github.com/gm1500/openpilot)"}) as response:
    response.raise_for_status()
    body = bytearray()
    for chunk in response.iter_content(65536):
      body.extend(chunk)
      if time.monotonic() > deadline:
        raise ValueError("OSM response deadline exceeded")
      if len(body) > MAX_RESPONSE_BYTES:
        raise ValueError("OSM response too large")
  data = json.loads(body)
  if not isinstance(data, dict) or "remark" in data or not isinstance(data.get("elements"), list):
    raise ValueError("Incomplete OSM response")
  roads = []
  controls = {element['id']: element['tags'] for element in data['elements']
              if element.get('type') == 'node' and isinstance(element.get('id'), int) and
              isinstance(element.get('tags'), dict) and control_kind(element['tags']) is not None and
              all(isinstance(k, str) and isinstance(v, str) for k, v in element['tags'].items())}
  for element in data["elements"]:
    tags = element.get("tags", {})
    if element.get("type") != "way" or tags.get("highway") not in ROAD_TYPES:
      continue
    geometry = tuple((float(point["lat"]), float(point["lon"])) for point in element.get("geometry", []))
    if len(geometry) < 2 or not all(math.isfinite(lat) and math.isfinite(lon) and -90 <= lat <= 90 and -180 <= lon <= 180 for lat, lon in geometry):
      continue
    if not all(isinstance(key, str) and isinstance(value, str) for key, value in tags.items()):
      continue
    nodes = element.get('nodes', [])
    node_ids = tuple(nodes) if len(nodes) == len(geometry) and all(isinstance(node, int) and node > 0 for node in nodes) else ()
    roads.append(Road(geometry, tags, int(element.get('id', 0)), node_ids,
                      {node: controls[node] for node in node_ids if node in controls}))
  return tuple(roads)


class OSMSpeedLimit:
  """Match cached roads on the map thread; only downloads run in the background."""
  def __init__(self):
    self._lock = threading.Lock()
    self._fix: GpsFix | None = None
    self._cache: tuple[tuple[Road, ...], GpsFix | None, float] = ((), None, -math.inf)
    self._tracker = RoadTracker()
    self._thread: threading.Thread | None = None
    self._wake = threading.Event()
    self.control: TrafficControl | None = None
    self.control_match: RoadMatch | None = None
    self.control_reason = 'noPosition'

  def update(self, fix: GpsFix | None, now: float) -> tuple[GpsFix, float | None, float | None, float] | None:
    self.control = self.control_match = None
    self.control_reason = 'noPosition'
    if fix is not None and (fix.speed < 0 or not 0 <= now - fix.timestamp <= GPS_MAX_AGE):
      fix = None  # reverse heading is retained by MapPosition, not walked as a forward road path
    with self._lock:
      changed = fix != self._fix
      self._fix = fix
      roads, centre, fetched_at = self._cache
    if changed:
      self._wake.set()
    if fix is None:
      self._tracker.reset()
      return None
    # Start only after the process has forked, on-road with a valid fix.
    if self._thread is None or not self._thread.is_alive():
      self._thread = threading.Thread(target=self._run, name="osm-fetch", daemon=True)
      self._thread.start()
    distance = math.hypot(*offset_metres(fix.latitude, fix.longitude, centre)) if centre is not None else math.inf
    # A fetch may complete just after the caller sampled `now`, before this
    # snapshot was read. Its internal completion timestamp is still fresh.
    if now - fetched_at >= CACHE_TTL or distance >= QUERY_RADIUS - 50:
      self._tracker.reset()
      self.control_reason = 'cacheUnavailable'
      return None
    # Publish the current match in this mapd cycle. No asynchronous grace period
    # or previous-limit hold is needed when GPS heading changes through a turn.
    match = self._tracker.update(roads, fix)
    gps_time = fix.gps_timestamp if fix.estimated else fix.timestamp
    self.control_reason = 'noRoadMatch' if match is None else 'gpsStale'
    if match is not None and match.forward is not None and gps_time > 0 and 0 <= now - gps_time <= GPS_MAX_AGE:
      self.control_match = match
      self.control = self._tracker.traffic_control(match)
      self.control_reason = self._tracker.control_reason
    speed = match.speed if match is not None else None
    advisory = match.advisory_speed if match is not None else None
    ahead = 0.
    if match is not None and speed is None and advisory is None:
      speed, ahead = self._tracker.merge_limit(match, fix)
    return fix, speed, advisory, ahead

  def _run(self):
    from openpilot.common.realtime import drop_realtime
    from openpilot.common.swaglog import cloudlog
    drop_realtime()
    next_query = 0.
    retry_interval = QUERY_INTERVAL
    while True:
      now = time.monotonic()
      with self._lock:
        fix = self._fix
        _, centre, fetched_at = self._cache
      if fix is not None and 0 <= now - fix.timestamp <= GPS_MAX_AGE:
        distance = math.hypot(*offset_metres(fix.latitude, fix.longitude, centre)) if centre is not None else math.inf
        if now >= next_query and (distance >= 700 or now - fetched_at >= CACHE_TTL):
          try:
            roads = fetch_roads(fix)
          except (requests.RequestException, ValueError, KeyError, TypeError, AttributeError) as error:
            # Do not log coordinates or response bodies; retain only the bounded cache.
            cloudlog.warning(f"OSM speed limit unavailable: {type(error).__name__}")
            next_query = time.monotonic() + retry_interval
            retry_interval = min(300., retry_interval * 2)
          else:
            fetched_at = time.monotonic()
            with self._lock:
              self._cache = roads, fix, fetched_at
            retry_interval = QUERY_INTERVAL
            next_query = fetched_at + QUERY_INTERVAL
      self._wake.wait(0.2)
      self._wake.clear()
