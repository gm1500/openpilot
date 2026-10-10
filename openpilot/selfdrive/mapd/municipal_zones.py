"""Verified municipal restrictions supplement the matched OSM road, never POIs.

Providers supply geometry, posted speeds, schedules and their civil timezone.
Their coverage bounds only select downloads; exact road geometry selects limits.
"""
import json
import math
import re
import time
from dataclasses import dataclass
from datetime import UTC, datetime
from itertools import pairwise
from zoneinfo import ZoneInfo, ZoneInfoNotFoundError

import requests

from openpilot.selfdrive.mapd.speed_rules import parse_speed, schedule_active

ZONE_TTL = 600.
ZONE_RADIUS = 1500.
MAX_ROWS = 1000
MAX_BYTES = 2 * 1024 * 1024
EDMONTON_URL = 'https://data.edmonton.ca/resource/6vjs-shqe.json'


@dataclass(frozen=True)
class SpeedZone:
  geometry: tuple[tuple[float, float], ...]
  road_name: str
  speed: float
  schedule: str
  timezone: str
  kind: str
  source: str


@dataclass(frozen=True)
class ZoneLimit:
  speed: float | None
  kind: str = 'none'
  source: str = ''


def parse_edmonton(rows) -> tuple[SpeedZone, ...]:
  if not isinstance(rows, list) or len(rows) >= MAX_ROWS:
    raise ValueError('Incomplete municipal response')
  zones = []
  for row in rows:
    # The API includes bylaw defaults and advisory rows: neither establishes a
    # school zone. Do not interpret the misleading "duration" (road length).
    if row.get('type') != 'PLAYGROUND' or row.get('jurisdiction') != 'CITY OF EDMONTON':
      continue
    speed = parse_speed(row.get('speed', ''))
    hours = re.fullmatch(r'PLAYGROUND ZONE (\d{2}:\d{2}) - (\d{2}:\d{2})', row.get('effective_time', ''))
    name = row.get('street_name_full', '').strip().upper()
    if speed is None or not name or row.get('travel_direction', '').lower() != 'both directions':
      raise ValueError('Unsupported municipal restriction')
    # An unknown schedule stays unknown on that road instead of using OSM's
    # higher default. No hours are assumed from the name of the city.
    schedule = f'{hours[1]}-{hours[2]}' if hours else 'unknown'
    geometry = row.get('geometry_line', {})
    if geometry.get('type') != 'MultiLineString' or not geometry.get('coordinates'):
      raise ValueError('Missing municipal geometry')
    for line in geometry['coordinates']:
      points = tuple((float(lat), float(lon)) for lon, lat in line)
      if len(points) < 2 or not all(math.isfinite(lat) and math.isfinite(lon) and -85 < lat < 85 and -180 <= lon <= 180
                                    for lat, lon in points):
        raise ValueError('Invalid municipal geometry')
      zones.append(SpeedZone(points, name, speed, schedule, 'America/Edmonton', 'school', f'edmonton:{row["objectid"]}'))
  return tuple(zones)


def fetch_zones(fix) -> tuple[SpeedZone, ...]:
  # First verified municipal feed. Outside its coverage, OSM's global
  # conditional evaluator remains available; never export these hours elsewhere.
  if not (53.2 <= fix.latitude <= 53.8 and -113.8 <= fix.longitude <= -113.2):
    return ()
  lat_margin = math.degrees(ZONE_RADIUS / 6371000.)
  lon_margin = lat_margin / math.cos(math.radians(fix.latitude))
  box = f'{fix.latitude + lat_margin:.6f},{fix.longitude - lon_margin:.6f},{fix.latitude - lat_margin:.6f},{fix.longitude + lon_margin:.6f}'
  params = {'$where': f"type='PLAYGROUND' AND within_box(geometry_line,{box})", '$limit': MAX_ROWS, '$order': 'objectid'}
  deadline = time.monotonic() + 15.
  with requests.get(EDMONTON_URL, params=params, timeout=(3.05, 8), stream=True,
                    headers={'User-Agent': 'boat-anchor-speed-limit/1.0 (https://github.com/gm1500/openpilot)'}) as response:
    response.raise_for_status()
    body = bytearray()
    for chunk in response.iter_content(65536):
      body.extend(chunk)
      if len(body) > MAX_BYTES or time.monotonic() > deadline:
        raise ValueError('Municipal response bound exceeded')
  return parse_edmonton(json.loads(body))


def _on_zone(zone: SpeedZone, match) -> bool:
  if match.bearing is None or match.road.tags.get('name', '').strip().upper() != zone.road_name:
    return False
  latitude, longitude = match.position
  scale = math.pi / 180 * 6371000.
  points = [((lon - longitude) * scale * math.cos(math.radians(latitude)), (lat - latitude) * scale) for lat, lon in zone.geometry]
  for (ax, ay), (bx, by) in pairwise(points):
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq < 1.:
      continue
    projection = -(ax * dx + ay * dy) / length_sq
    # No endpoint-radius extension: being near a zone is insufficient.
    if not 0. <= projection <= 1. or math.hypot(ax + projection * dx, ay + projection * dy) > 12.:
      continue
    delta = abs((math.degrees(math.atan2(dx, dy)) - match.bearing + 180) % 360 - 180)
    if min(delta, 180 - delta) <= 30.:
      return True
  return False


def zone_limit(zones: tuple[SpeedZone, ...], match, utc: float | None, fresh: bool = True) -> ZoneLimit | None:
  """None means no active overlay; ZoneLimit(None) means a known unresolved zone."""
  active = []
  for zone in zones:
    if not _on_zone(zone, match):
      continue
    local_time = None
    if utc is not None:
      try:
        local_time = datetime.fromtimestamp(utc, UTC).astimezone(ZoneInfo(zone.timezone))
      except (ZoneInfoNotFoundError, ValueError, OverflowError):
        pass
    state = schedule_active(zone.schedule, local_time) if fresh else None
    if state is None:
      return ZoneLimit(None, source=zone.source)
    if state:
      active.append(zone)
  if not active:
    return None
  first = active[0]
  if any((zone.speed, zone.kind) != (first.speed, first.kind) for zone in active):
    return ZoneLimit(None, source=first.source)
  return ZoneLimit(first.speed, first.kind, first.source)
