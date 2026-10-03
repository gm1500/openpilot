"""Conservative OSM matching and bounded background fetching."""
import json
import math
import re
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from itertools import pairwise

import requests

OVERPASS_URL = "https://overpass-api.de/api/interpreter"
QUERY_RADIUS = 1500.0  # metres; prefetch again after travelling 700 m
QUERY_INTERVAL = 30.0
CACHE_TTL = 600.0
GPS_MAX_AGE = 3.0
QCOM_UNKNOWN_ACCURACY = 15.0  # Matching allowance, not a measured GPS uncertainty
MAX_RESPONSE_BYTES = 2 * 1024 * 1024
ROAD_TYPES = ("motorway", "trunk", "primary", "secondary", "tertiary", "unclassified", "residential", "living_street", "service", "road",
              "motorway_link", "trunk_link", "primary_link", "secondary_link", "tertiary_link")
EARTH_RADIUS = 6371000.0


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
  bounds: tuple[float, float, float, float] = field(init=False, repr=False)

  def __post_init__(self):
    latitudes, longitudes = zip(*self.geometry, strict=True)
    object.__setattr__(self, "bounds", (min(latitudes), max(latitudes), min(longitudes), max(longitudes)))


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


def road_speed(tags: dict[str, str], forward: bool | None) -> float | None:
  # These may override the numeric base limit. Without evaluating their conditions,
  # vehicle classes or lane applicability, showing the base value would be misleading.
  supported = {"maxspeed", "maxspeed:forward", "maxspeed:backward", "maxspeed:type"}
  if any(key.startswith("maxspeed:") and key not in supported for key in tags):
    return None
  base = tags.get("maxspeed", "")
  ahead = parse_speed(tags.get("maxspeed:forward", base))
  behind = parse_speed(tags.get("maxspeed:backward", base))
  if forward is None:
    return ahead if ahead == behind else None
  return ahead if forward else behind


def match_speed(roads: tuple[Road, ...], fix: GpsFix) -> float | None:
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
    points = [offset_metres(lat, lon, fix) for lat, lon in road.geometry]
    for (ax, ay), (bx, by) in pairwise(points):
      dx, dy = bx - ax, by - ay
      length_sq = dx * dx + dy * dy
      if length_sq < 1:
        continue
      projection = max(0., min(1., -(ax * dx + ay * dy) / length_sq))
      distance = math.hypot(ax + projection * dx, ay + projection * dy)
      if distance > 25:
        continue
      forward = None
      if fix.bearing is not None:
        heading = math.degrees(math.atan2(dx, dy)) % 360
        delta = abs((heading - fix.bearing + 180) % 360 - 180)
        forward = delta <= 90
        oneway = road.tags.get("oneway", "yes" if road.tags.get("junction") == "roundabout" else "no")
        if min(delta, 180 - delta) > 40 or (oneway in ("yes", "1", "true") and not forward) or (oneway == "-1" and forward):
          continue
      if nearest is None or distance < nearest[0]:
        nearest = (distance, road_speed(road.tags, forward))
    if nearest is not None:
      candidates.append(nearest)
  if not candidates:
    return None
  candidates.sort(key=lambda candidate: candidate[0])
  distance, speed = candidates[0]
  # Include untagged roads: never borrow a nearby road's limit when ours is unknown.
  ambiguity_margin = max(6., fix.accuracy * 2)
  if any(other_speed != speed and other_distance <= distance + ambiguity_margin for other_distance, other_speed in candidates[1:]):
    return None
  return speed


def fetch_roads(fix: GpsFix) -> tuple[Road, ...]:
  query = f'[out:json][timeout:10][maxsize:16777216];way(around:{QUERY_RADIUS:.0f},{fix.latitude:.6f},{fix.longitude:.6f})'
  query += f'["highway"~"^({"|".join(ROAD_TYPES)})$"];out tags geom;'
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
    roads.append(Road(geometry, tags))
  return tuple(roads)


class OSMSpeedLimit:
  """Caller supplies fresh fixes; background threads handle matching and network I/O."""
  def __init__(self):
    self._lock = threading.Lock()
    self._fix: GpsFix | None = None
    self._result: tuple[GpsFix, float | None] | None = None
    self._thread: threading.Thread | None = None

  def update(self, fix: GpsFix | None, now: float) -> tuple[GpsFix, float | None] | None:
    with self._lock:
      self._fix = fix
      if fix is None:
        self._result = None
      result = self._result
    # Start only after the process has forked, with a valid fix.
    if fix is not None and (self._thread is None or not self._thread.is_alive()):
      self._thread = threading.Thread(target=self._run, name="osm-speed-limit", daemon=True)
      self._thread.start()
    if fix is None or result is None or not 0 <= now - result[0].timestamp <= GPS_MAX_AGE:
      return None
    if math.hypot(*offset_metres(fix.latitude, fix.longitude, result[0])) > 20:
      return None
    if fix.bearing is not None and result[0].bearing is not None and abs((fix.bearing - result[0].bearing + 180) % 360 - 180) > 40:
      return None
    return result

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
          speed = match_speed(roads, fix) if cache_fresh and distance < QUERY_RADIUS - 50 else None
          with self._lock:
            self._result = (fix, speed)
          if pending is None and now >= next_query and (distance >= 700 or not cache_fresh):
            query_fix = fix
            pending = executor.submit(fetch_roads, fix)
            next_query = now + QUERY_INTERVAL
        time.sleep(0.2)
