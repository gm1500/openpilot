"""OSM road matching with separate legal/advisory speeds and bounded display holds."""
import json
import math
import re
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field, replace
from itertools import pairwise

import requests

OVERPASS_URL = "https://overpass-api.de/api/interpreter"
QUERY_RADIUS = 1500.0  # metres; prefetch again after travelling 700 m
QUERY_INTERVAL = 30.0
CACHE_TTL = 600.0
GPS_MAX_AGE = 3.0
DISPLAY_HOLD_SECONDS = 2.0  # Display only; never a fresh control target
DISPLAY_HOLD_METRES = 60.0
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


@dataclass(frozen=True)
class Road:
  geometry: tuple[tuple[float, float], ...]
  tags: dict[str, str]
  way_id: int = 0
  node_ids: tuple[int, ...] = ()
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
  length: float
  forward: bool | None
  position: tuple[float, float]

  @property
  def advisory_speed(self) -> float | None:
    return road_speed(self.road.tags, self.forward, advisory=True)


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
    fixes.append(GpsFix(gps.latitude, gps.longitude, bearing, accuracy, timestamp))
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


def match_speed(roads: tuple[Road, ...], fix: GpsFix) -> float | None:
  """Strict acquisition without history; untagged competing roads also count."""
  candidates = road_candidates(roads, fix)
  return candidates[0].speed if candidates and not conflicting_roads(candidates, fix) else None


def road_candidates(roads: tuple[Road, ...], fix: GpsFix) -> list[RoadMatch]:
  candidates = []
  latitude_margin = math.degrees(25 / EARTH_RADIUS)
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
      if distance > 25:
        continue
      forward = None
      heading_error = None
      if fix.bearing is not None:
        heading = math.degrees(math.atan2(dx, dy)) % 360
        delta = abs((heading - fix.bearing + 180) % 360 - 180)
        heading_error = min(delta, 180 - delta)
        forward = delta <= 90
        oneway = road.tags.get("oneway", "yes" if road.tags.get("junction") == "roundabout" else "no")
        if min(delta, 180 - delta) > 40 or (oneway in ("yes", "1", "true") and not forward) or (oneway == "-1" and forward):
          continue
      if nearest is None or distance < nearest.distance:
        x, y = ax + projection * dx, ay + projection * dy
        position = (fix.latitude + math.degrees(y / EARTH_RADIUS),
                    (fix.longitude + math.degrees(x / (EARTH_RADIUS * math.cos(math.radians(fix.latitude)))) + 180) % 360 - 180)
        nearest = RoadMatch(road, distance, road_speed(road.tags, forward), heading_error,
                            segment_start + projection * length, 0., forward, position)
    if nearest is not None:
      candidates.append(replace(nearest, length=along))
  return sorted(candidates, key=lambda candidate: candidate.distance)


def conflicting_roads(candidates: list[RoadMatch], fix: GpsFix) -> list[RoadMatch]:
  best = candidates[0]
  return [c for c in candidates[1:] if (c.speed, c.advisory_speed) != (best.speed, best.advisory_speed)
          and c.distance <= best.distance + max(6., fix.accuracy * 2)]


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


class RoadTracker:
  """Track directed progress through cached OSM ways; heading is supporting evidence.

  Ambiguous fixes do not extend the accepted history. The nearest road must still
  win geometrically; topology never locks us to a farther road or invents a limit.
  """
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
    self._candidate: tuple[RoadMatch, GpsFix] | None = None

  def _set_roads(self, roads: tuple[Road, ...]):
    self._roads = roads
    self._candidate = None
    self._connections = {}
    self._nodes = {}
    for road in roads:
      self._nodes[id(road)] = road_nodes(road)
      # Junctions can be interior nodes of a way, not just its endpoints.
      for node, along in self._nodes[id(road)]:
        for forward in allowed_directions(road):
          self._connections.setdefault(node, []).append((road, forward, along))
    # Retain identity across cache refreshes only if geometry/direction still
    # agree. Reproject the accepted fix so changed speed tags are used at once.
    if self.match is not None and self.fix is not None:
      old = self.match.road
      same = next((road for road in roads if (road.way_id == old.way_id if old.way_id else road == old)
                   and road.geometry == old.geometry), None)
      projected = road_candidates((same,), self.fix) if same is not None else []
      if projected and projected[0].forward == self.match.forward:
        self.match = projected[0]
      else:
        self.match = self.fix = None

  def _progress(self, previous: RoadMatch, current: RoadMatch, maximum: float) -> float | None:
    if previous.forward is None or current.forward is None:
      return None
    if current.road == previous.road:
      return (current.along - previous.along) * (1 if previous.forward else -1) if current.forward == previous.forward else None
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

  def _follows_geometry(self, previous: RoadMatch, previous_fix: GpsFix, current: RoadMatch, fix: GpsFix) -> bool:
    if fix.bearing is None:
      return False
    travelled = math.hypot(*offset_metres(fix.latitude, fix.longitude, previous_fix))
    maximum = max(15., travelled * 1.6 + 8.)
    progress = self._progress(previous, current, maximum)
    if (progress is None or not -3 <= progress <= maximum or current.distance > 12 or
        current.heading_error is None or current.heading_error > 35):
      return False
    if travelled < 3:
      # At low displacement GPS motion cannot resolve competing roads.
      return current.road == previous.road and current.heading_error <= 15 and current.distance <= 8
    origin = GpsFix(*previous.position, None, 0., 0.)
    px, py = offset_metres(*current.position, origin)
    gx, gy = offset_metres(fix.latitude, fix.longitude, previous_fix)
    projected_distance = math.hypot(px, py)
    # Compare displacement along the CURVE with GPS displacement, rather than
    # requiring nearly constant absolute compass heading through a bend.
    return (projected_distance >= 2 and progress >= 0 and
            (px * gx + py * gy) / (projected_distance * travelled) >= math.cos(math.radians(25)))

  def update(self, roads: tuple[Road, ...], fix: GpsFix) -> RoadMatch | None:
    refreshed = roads is not self._roads
    if refreshed:
      self._set_roads(roads)
    elif fix == self._processed:
      return self._result  # repeated polling is not new motion evidence
    self._processed = fix
    if self.fix is not None:
      dt = fix.timestamp - self.fix.timestamp
      travelled = math.hypot(*offset_metres(fix.latitude, fix.longitude, self.fix))
      if not 0 <= dt <= GPS_MAX_AGE or travelled > 75 * dt + 10:
        self.match = self.fix = None
    candidates = road_candidates(roads, fix)
    result = None
    if candidates:
      best = candidates[0]
      conflicts = conflicting_roads(candidates, fix)
      follows = (self.match is not None and self.fix is not None and self._follows_geometry(self.match, self.fix, best, fix))
      # A long, shallow fork can outlive confirmed history. Reacquire only after
      # two fresh moving fixes favour the same/connected road by a clear margin.
      # Tentative fixes never publish speeds or extend confirmed history.
      separated = (best.distance <= 8 and best.heading_error is not None and best.heading_error <= 25 and
                   all(c.distance >= best.distance + 6 for c in conflicts))
      if self.match is None and separated and self._candidate is not None:
        previous, previous_fix = self._candidate
        dt = fix.timestamp - previous_fix.timestamp
        travelled = math.hypot(*offset_metres(fix.latitude, fix.longitude, previous_fix))
        connected = (0 < dt <= GPS_MAX_AGE and travelled <= 75 * dt + 10 and
                     self._follows_geometry(previous, previous_fix, best, fix))
        follows = connected and dt >= 0.5 and travelled >= 5
        if not connected:
          self._candidate = None
      if not conflicts or (follows and all(c.distance >= best.distance + 3 for c in conflicts)):
        result = best
      if self.match is None and separated and result is None:
        # Retain the starting fix while motion accumulates on faster GPS feeds.
        if self._candidate is None:
          self._candidate = (best, fix)
      else:
        self._candidate = None
    else:
      self._candidate = None
    if result is not None:
      self.match, self.fix = result, fix
    self._result = result
    return result


def fetch_roads(fix: GpsFix) -> tuple[Road, ...]:
  query = f'[out:json][timeout:10][maxsize:16777216];way(around:{QUERY_RADIUS:.0f},{fix.latitude:.6f},{fix.longitude:.6f})'
  query += f'["highway"~"^({"|".join(ROAD_TYPES)})$"];out body geom;'
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
    roads.append(Road(geometry, tags, int(element.get('id', 0)), node_ids))
  return tuple(roads)


class OSMSpeedLimit:
  """Caller supplies fresh fixes; background threads handle matching and network I/O."""
  def __init__(self):
    self._lock = threading.Lock()
    self._fix: GpsFix | None = None
    self._result: tuple[GpsFix, float | None, float | None] | None = None
    self._thread: threading.Thread | None = None
    self._wake = threading.Event()
    self._display: tuple[GpsFix, float, bool] | None = None
    self.control_sample: tuple[GpsFix, float | None] | None = None
    self._fix_received_at = 0.

  @property
  def display_timestamp(self) -> float:
    return self._display[0].timestamp if self._display is not None else 0.

  @property
  def display_is_advisory(self) -> bool:
    return self._display[2] if self._display is not None else False

  def update(self, fix: GpsFix | None, now: float) -> float | None:
    with self._lock:
      changed = fix != self._fix
      self._fix = fix
      if fix is None:
        self._result = None
      result = self._result
    if changed:
      self._fix_received_at = now
      self._wake.set()
    self.control_sample = None
    # Start only after the process has forked, on-road with a valid fix.
    if fix is not None and (self._thread is None or not self._thread.is_alive()):
      self._thread = threading.Thread(target=self._run, name="osm-speed-limit", daemon=True)
      self._thread.start()
    if fix is None or not 0 <= now - fix.timestamp <= GPS_MAX_AGE:
      self._display = None
      return None
    if result is not None and result[0] == fix:
      self.control_sample = result[:2]
    elif result is not None and 0 <= now - self._fix_received_at <= 0.3 and 0 <= now - result[0].timestamp <= GPS_MAX_AGE:
      # A short asynchronous handoff may retain the PREVIOUS confirmed sample,
      # with its original GPS timestamp. A current ambiguous result never qualifies.
      last_fix = result[0]
      if (fix.bearing is not None and last_fix.bearing is not None and
          abs((fix.bearing - last_fix.bearing + 180) % 360 - 180) <= 20 and
          math.hypot(*offset_metres(fix.latitude, fix.longitude, last_fix)) <= DISPLAY_HOLD_METRES):
        self.control_sample = result[:2]
    if result is not None and result[0] == fix:
      _, legal, advisory = result
      # A lower advisory takes display priority, but never enters control_sample.
      is_advisory = advisory is not None and (legal is None or advisory < legal)
      speed = advisory if is_advisory else legal
      if speed is not None:
        self._display = (fix, speed, is_advisory)
        return speed
    # New 1 Hz GPS fixes can be 30 m apart at highway speed. Retain the previous
    # confirmed sign while matching catches up or briefly becomes ambiguous.
    # Keep its original GPS timestamp: polling must never extend the hold.
    if self._display is not None:
      last_fix, speed, _ = self._display
      distance = math.hypot(*offset_metres(fix.latitude, fix.longitude, last_fix))
      same_heading = (fix.bearing is not None and last_fix.bearing is not None and
                      abs((fix.bearing - last_fix.bearing + 180) % 360 - 180) <= 20)
      stationary = fix.bearing is None and last_fix.bearing is None and distance <= 5
      if 0 <= now - last_fix.timestamp <= DISPLAY_HOLD_SECONDS and distance <= DISPLAY_HOLD_METRES and (same_heading or stationary):
        return speed
    self._display = None
    return None

  def _run(self):
    from openpilot.common.realtime import drop_realtime
    from openpilot.common.swaglog import cloudlog
    drop_realtime()
    roads: tuple[Road, ...] = ()
    centre = None
    fetched_at = -math.inf
    next_query = 0.
    retry_interval = QUERY_INTERVAL
    pending = None
    query_fix = None
    tracker = RoadTracker()
    with ThreadPoolExecutor(max_workers=1, thread_name_prefix="osm-fetch") as executor:
      while True:
        now = time.monotonic()
        with self._lock:
          fix = self._fix
        if pending is not None and pending.done():
          try:
            roads = pending.result()
            centre, fetched_at = query_fix, now
            retry_interval = QUERY_INTERVAL
          except (requests.RequestException, ValueError, KeyError, TypeError, AttributeError) as error:
            # Do not log coordinates or response bodies.
            cloudlog.warning(f"OSM speed limit unavailable: {type(error).__name__}")
            next_query = now + retry_interval
            retry_interval = min(300., retry_interval * 2)
          pending = None
        if fix is not None and 0 <= now - fix.timestamp <= GPS_MAX_AGE:
          distance = math.hypot(*offset_metres(fix.latitude, fix.longitude, centre)) if centre is not None else math.inf
          cache_fresh = now - fetched_at < CACHE_TTL
          if cache_fresh and distance < QUERY_RADIUS - 50:
            match = tracker.update(roads, fix)
          else:
            tracker.reset()
            match = None
          speed = match.speed if match is not None else None
          advisory = match.advisory_speed if match is not None else None
          with self._lock:
            self._result = (fix, speed, advisory)
          if pending is None and now >= next_query and (distance >= 700 or not cache_fresh):
            query_fix = fix
            pending = executor.submit(fetch_roads, fix)
            next_query = now + QUERY_INTERVAL
        else:
          tracker.reset()
        self._wake.wait(0.2)
        self._wake.clear()
