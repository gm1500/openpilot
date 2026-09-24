# Data source: © OpenStreetMap contributors, ODbL 1.0.
# https://www.openstreetmap.org/copyright
#
# Proof of concept only: this daemon reads explicit OSM maxspeed tags and
# publishes a display-only speed limit. It does not feed longitudinal control.

import json
import math
import time
import urllib.error
import urllib.parse
import urllib.request

from openpilot.cereal import messaging
from openpilot.common.swaglog import cloudlog


OVERPASS_URL = "https://overpass-api.de/api/interpreter"
QUERY_RADIUS_M = 700
REQUERY_DISTANCE_M = 300
REQUERY_INTERVAL_S = 60.0
RETRY_INTERVAL_S = 20.0
MAX_MATCH_DISTANCE_M = 35.0
MAX_HEADING_ERROR_DEG = 55.0
MIN_MATCH_SPEED_MS = 2.0
MAX_GPS_ACCURACY_M = 35.0
EARTH_RADIUS_M = 6371000.0

DRIVABLE_HIGHWAYS = {
  "motorway", "motorway_link",
  "trunk", "trunk_link",
  "primary", "primary_link",
  "secondary", "secondary_link",
  "tertiary", "tertiary_link",
  "unclassified", "residential", "service", "living_street", "road",
}


def angular_difference_deg(a: float, b: float) -> float:
  return abs((a - b + 180.0) % 360.0 - 180.0)


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
  lat1_r = math.radians(lat1)
  lat2_r = math.radians(lat2)
  dlat = lat2_r - lat1_r
  dlon = math.radians(lon2 - lon1)
  h = math.sin(dlat / 2.0) ** 2 + math.cos(lat1_r) * math.cos(lat2_r) * math.sin(dlon / 2.0) ** 2
  return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(h)))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
  lat1_r = math.radians(lat1)
  lat2_r = math.radians(lat2)
  dlon = math.radians(lon2 - lon1)
  y = math.sin(dlon) * math.cos(lat2_r)
  x = math.cos(lat1_r) * math.sin(lat2_r) - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlon)
  return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def point_to_segment_distance_m(lat: float, lon: float, a: dict, b: dict) -> float:
  lat_ref = math.radians(lat)
  meters_per_deg_lat = math.pi * EARTH_RADIUS_M / 180.0
  meters_per_deg_lon = meters_per_deg_lat * math.cos(lat_ref)

  ax = (float(a["lon"]) - lon) * meters_per_deg_lon
  ay = (float(a["lat"]) - lat) * meters_per_deg_lat
  bx = (float(b["lon"]) - lon) * meters_per_deg_lon
  by = (float(b["lat"]) - lat) * meters_per_deg_lat

  dx = bx - ax
  dy = by - ay
  denom = dx * dx + dy * dy
  if denom <= 1e-6:
    return math.hypot(ax, ay)

  t = max(0.0, min(1.0, -(ax * dx + ay * dy) / denom))
  return math.hypot(ax + t * dx, ay + t * dy)


def parse_maxspeed_kph(value: str | None) -> int | None:
  if not value:
    return None

  value = value.strip().lower().split(";")[0].strip()
  if value in ("none", "signals", "variable", "walk"):
    return None

  multiplier = 1.0
  for suffix, scale in (("km/h", 1.0), ("kmh", 1.0), ("kph", 1.0), ("mph", 1.609344)):
    if value.endswith(suffix):
      value = value[:-len(suffix)].strip()
      multiplier = scale
      break

  try:
    speed = float(value) * multiplier
  except ValueError:
    return None

  if not 5.0 <= speed <= 160.0:
    return None
  return int(round(speed))


class OsmSpeedLimitMatcher:
  def __init__(self):
    self.ways: list[dict] = []
    self.query_lat: float | None = None
    self.query_lon: float | None = None
    self.last_query_time = 0.0
    self.last_query_attempt = 0.0
    self.last_way_id: int | None = None

  def _build_query(self, lat: float, lon: float) -> str:
    around = f"around:{QUERY_RADIUS_M},{lat:.7f},{lon:.7f}"
    return (
      "[out:json][timeout:6];"
      "("
      f'way({around})["highway"]["maxspeed"];'
      f'way({around})["highway"]["maxspeed:forward"];'
      f'way({around})["highway"]["maxspeed:backward"];'
      ");"
      "out tags geom;"
    )

  def _fetch(self, lat: float, lon: float) -> list[dict]:
    payload = urllib.parse.urlencode({"data": self._build_query(lat, lon)}).encode("utf-8")
    request = urllib.request.Request(
      OVERPASS_URL,
      data=payload,
      headers={"User-Agent": "gm1500-openpilot-osm-speedlimit-poc/1.0"},
      method="POST",
    )
    with urllib.request.urlopen(request, timeout=8.0) as response:
      data = json.loads(response.read().decode("utf-8"))

    ways = []
    for element in data.get("elements", []):
      if element.get("type") != "way":
        continue
      tags = element.get("tags", {})
      if tags.get("highway") not in DRIVABLE_HIGHWAYS:
        continue
      geometry = element.get("geometry", [])
      if len(geometry) < 2:
        continue
      if not any(parse_maxspeed_kph(tags.get(k)) is not None for k in ("maxspeed", "maxspeed:forward", "maxspeed:backward")):
        continue
      ways.append({
        "id": int(element.get("id", 0)),
        "tags": tags,
        "geometry": geometry,
      })
    return ways

  def maybe_refresh(self, lat: float, lon: float, now: float) -> None:
    moved = (
      self.query_lat is None or self.query_lon is None or
      haversine_m(lat, lon, self.query_lat, self.query_lon) >= REQUERY_DISTANCE_M
    )
    expired = now - self.last_query_time >= REQUERY_INTERVAL_S
    if not (moved or expired):
      return
    if now - self.last_query_attempt < RETRY_INTERVAL_S:
      return

    self.last_query_attempt = now
    try:
      ways = self._fetch(lat, lon)
    except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError, json.JSONDecodeError, OSError) as exc:
      cloudlog.warning(f"osm speed limit query failed: {type(exc).__name__}")
      return

    self.ways = ways
    self.query_lat = lat
    self.query_lon = lon
    self.last_query_time = now
    cloudlog.info(f"osm speed limit cache refreshed: {len(ways)} candidate ways")

  def match(self, lat: float, lon: float, vehicle_bearing: float, speed_ms: float) -> dict | None:
    if speed_ms < MIN_MATCH_SPEED_MS:
      return None

    best = None
    best_score = float("inf")

    for way in self.ways:
      tags = way["tags"]
      geometry = way["geometry"]
      for a, b in zip(geometry, geometry[1:], strict=False):
        distance = point_to_segment_distance_m(lat, lon, a, b)
        if distance > MAX_MATCH_DISTANCE_M:
          continue

        segment_bearing = bearing_deg(float(a["lat"]), float(a["lon"]), float(b["lat"]), float(b["lon"]))
        directional_error = angular_difference_deg(vehicle_bearing, segment_bearing)
        heading_error = min(directional_error, 180.0 - directional_error)
        if heading_error > MAX_HEADING_ERROR_DEG:
          continue

        traveling_forward = directional_error <= 90.0
        directional_key = "maxspeed:forward" if traveling_forward else "maxspeed:backward"
        speed_limit_kph = parse_maxspeed_kph(tags.get(directional_key))
        if speed_limit_kph is None:
          speed_limit_kph = parse_maxspeed_kph(tags.get("maxspeed"))
        if speed_limit_kph is None:
          continue

        score = distance + 0.20 * heading_error
        if way["id"] == self.last_way_id:
          score -= 6.0

        if score < best_score:
          best_score = score
          best = {
            "speed_limit_kph": speed_limit_kph,
            "way_id": way["id"],
            "distance_m": round(distance, 1),
            "road_name": tags.get("name", ""),
          }

    if best is not None:
      self.last_way_id = best["way_id"]
    else:
      self.last_way_id = None
    return best


def send_speed_limit(pm: messaging.PubMaster, match: dict | None) -> None:
  payload = match if match is not None else {"speed_limit_kph": 0}
  msg = messaging.new_message("customReservedRawData0")
  msg.valid = True
  msg.customReservedRawData0 = json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
  pm.send("customReservedRawData0", msg)


def main() -> None:
  sm = messaging.SubMaster(["gpsLocationExternal"])
  pm = messaging.PubMaster(["customReservedRawData0"])
  matcher = OsmSpeedLimitMatcher()

  current_match: dict | None = None
  last_valid_fix = 0.0
  last_publish = 0.0

  while True:
    sm.update(500)
    now = time.monotonic()

    if sm.updated["gpsLocationExternal"]:
      gps = sm["gpsLocationExternal"]
      valid_fix = (
        gps.hasFix and
        -90.0 <= gps.latitude <= 90.0 and
        -180.0 <= gps.longitude <= 180.0 and
        (gps.horizontalAccuracy <= 0.0 or gps.horizontalAccuracy <= MAX_GPS_ACCURACY_M)
      )

      if valid_fix:
        last_valid_fix = now
        matcher.maybe_refresh(gps.latitude, gps.longitude, now)

        if gps.speed >= MIN_MATCH_SPEED_MS:
          current_match = matcher.match(gps.latitude, gps.longitude, gps.bearingDeg, gps.speed)
      elif now - last_valid_fix > 5.0:
        current_match = None

    if now - last_valid_fix > 5.0:
      current_match = None

    if now - last_publish >= 1.0:
      send_speed_limit(pm, current_match)
      last_publish = now


if __name__ == "__main__":
  main()
