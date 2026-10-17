"""Read persistent regional road tiles without running another control daemon."""
import json
import math
import os
from pathlib import Path
import threading

import capnp

from openpilot.common.basedir import BASEDIR
from openpilot.common.hardware import PC
from openpilot.common.hardware.hw import Paths

SOURCE = Path(BASEDIR) / 'openpilot/third_party/offline_maps'
MAX_TILE_BYTES = 32 * 1024 * 1024
SCHEMA = capnp.load(str(SOURCE / 'offline.capnp'))


def map_root() -> Path:
  return (Path(Paths.comma_home()) if PC else Path('/data')) / 'maps/slc'


def atomic_json(path: Path, data):
  path.parent.mkdir(parents=True, exist_ok=True)
  tmp = path.with_suffix('.tmp')
  with tmp.open('w') as file:
    json.dump(data, file, separators=(',', ':'))
    file.flush()
    os.fsync(file.fileno())
  os.replace(tmp, path)


def load_json(path: Path, default):
  try:
    return json.loads(path.read_text())
  except (OSError, ValueError):
    return default


def region_catalog():
  return json.loads((SOURCE / 'regions.json').read_text())


def region_name(region: str) -> str:
  if region == 'region.NA':
    return 'North America (Canada, U.S., Mexico)'
  kind, _, code = region.partition('.')
  return region_catalog().get(kind, {}).get(code, {}).get('full_name', region)


def region_groups(region: str) -> list[str]:
  catalog = region_catalog()
  if region == 'region.NA':
    return sorted(set().union(*(region_groups(x) for x in ('nation.CA', 'nation.US', 'nation.MX', 'us_state.AK', 'us_state.HI'))))
  kind, _, code = region.partition('.')
  if kind not in catalog or code not in catalog[kind]:
    raise ValueError('Unknown map region')
  item = catalog[kind][code]
  b = item['bounding_box']
  south, west = math.floor(b['min_lat'] / 2) * 2, math.floor(b['min_lon'] / 2) * 2
  north, east = math.ceil(b['max_lat'] / 2) * 2, math.ceil(b['max_lon'] / 2) * 2
  rows = item.get('download_rows') or [(lat, west, east) for lat in range(south, north, 2)]
  if any(len(row) != 3 or any(not isinstance(v, int) or v % 2 for v in row) or
         not south <= row[0] < north or not west <= row[1] < row[2] <= east for row in rows):
    raise ValueError('Invalid map region bounds')
  groups = {f'{lat}_{lon}' for lat, lo, hi in rows for lon in range(lo, hi, 2)}
  if region == 'nation.US':
    groups.update(region_groups('us_state.AK'))
    groups.update(region_groups('us_state.HI'))
  return sorted(groups)


def tile_name(lat: float, lon: float) -> tuple[str, str]:
  # Same quarter-degree grid as the publisher. Read both neighbouring tiles
  # when the query straddles a boundary; never depend on the tiny tile overlap.
  lat, lon = math.floor(lat * 4) / 4, math.floor(lon * 4) / 4
  group = f'{math.floor(lat / 2) * 2}_{math.floor(lon / 2) * 2}'
  return group, f'{lat:.6f}_{lon:.6f}_{lat+.25:.6f}_{lon+.25:.6f}'


class OfflineMaps:
  def __init__(self, root=None):
    self.root = Path(root) if root is not None else map_root()
    self._tiles = {}
    self._stamp = None
    self._manifest = {}

  def roads(self, fix, radius=1500.):
    from openpilot.selfdrive.mapd.osm_speed_limit import Road
    try:
      stamp = (self.root / 'installed.json').stat().st_mtime_ns
      if stamp != self._stamp:
        self._manifest = load_json(self.root / 'installed.json', {})
        self._stamp = stamp
        self._tiles.clear()
      # Use a bounding square, including all boundary tiles. At polar latitudes
      # an excessive tile span is left to the existing online fallback.
      dy = radius / 110000.
      dx = dy / max(.01, math.cos(math.radians(fix.latitude)))
      south, north = fix.latitude - dy, fix.latitude + dy
      west, east = fix.longitude - dx, fix.longitude + dx
      tiles = [tile_name(y / 4, x / 4) for y in range(math.floor(south*4), math.floor(north*4)+1)
               for x in range(math.floor(west*4), math.floor(east*4)+1)]
      if len(tiles) > 4 or south < -90 or north > 90 or west < -180 or east > 180:
        return None
      roads = {}
      for group, filename in tiles:
        info = self._manifest.get('groups', {}).get(group)
        if not info:
          return None
        path = self.root / 'groups' / group / info['generation'] / filename
        if path not in self._tiles:
          if path.stat().st_size > MAX_TILE_BYTES:
            return None
          data = path.read_bytes()
          if not data:
            return None
          # Decode with finite traversal and nesting limits. A missing/corrupt
          # tile never becomes a valid empty region or triggers a cellular fetch.
          tile = SCHEMA.Offline.from_bytes_packed(data, traversal_limit_in_words=16*1024*1024, nesting_limit=64)
          if not all(math.isfinite(v) for v in (tile.minLat, tile.minLon, tile.maxLat, tile.maxLon)):
            return None
          if len(self._tiles) >= 4:
            self._tiles.pop(next(iter(self._tiles)))
          self._tiles[path] = tile
        tile = self._tiles[path]
        for way in tile.ways:
          if way.maxLat < south or way.minLat > north or way.maxLon < west or way.minLon > east:
            continue
          if len(way.nodes) < 2 or len(way.nodes) > 20000:
            continue
          points = tuple((p.latitude, p.longitude) for p in way.nodes)
          if not all(math.isfinite(a) and math.isfinite(b) and -90 <= a <= 90 and -180 <= b <= 180 for a, b in points):
            continue
          tags = {'name': str(way.name), 'oneway': 'yes' if way.oneWay else 'no'}
          for attr, key in (('maxSpeed', 'maxspeed'), ('maxSpeedForward', 'maxspeed:forward'),
                            ('maxSpeedBackward', 'maxspeed:backward'), ('advisorySpeed', 'maxspeed:advisory')):
            speed = getattr(way, attr)
            if math.isfinite(speed) and 0 < speed <= 300 / 3.6:
              tags[key] = str(round(speed * 3.6, 3))
          for attr, key in (('maxSpeedConditional', 'maxspeed:conditional'),
                            ('maxSpeedForwardConditional', 'maxspeed:forward:conditional'),
                            ('maxSpeedBackwardConditional', 'maxspeed:backward:conditional')):
            if value := str(getattr(way, attr)):
              tags[key] = value
          # No synthetic node IDs: coincident coordinates do not prove a junction.
          roads[(way.id, points)] = Road(points, tags, int(way.id))
      return tuple(roads.values())
    except (OSError, ValueError, TypeError, KeyError, capnp.KjException):
      return None


class OfflineWorker:
  """Disk reads cannot delay the live publisher or wait behind network timeouts."""
  def __init__(self, reader=None):
    self.reader = reader or OfflineMaps()
    self._lock = threading.Lock()
    self._fix = None
    self._cache = ((), None, -math.inf)
    self._thread = None
    self._wake = threading.Event()

  def update(self, fix, now):
    from openpilot.selfdrive.mapd.osm_speed_limit import offset_metres
    if self._thread is None and not (self.reader.root / 'installed.json').is_file():
      return None
    with self._lock:
      self._fix = fix
      roads, centre, stamp = self._cache
    if self._thread is None:
      self._thread = threading.Thread(target=self._run, daemon=True, name='offline-roads')
      self._thread.start()
    self._wake.set()
    if centre is not None and now - stamp < 600 and math.hypot(*offset_metres(fix.latitude, fix.longitude, centre)) < 1450:
      return roads
    return None

  def _run(self):
    import time
    from openpilot.selfdrive.mapd.osm_speed_limit import offset_metres
    from openpilot.common.realtime import drop_realtime
    drop_realtime()
    last_attempt = -math.inf
    while True:
      with self._lock:
        fix = self._fix
        _, centre, stamp = self._cache
      now = time.monotonic()
      distance = math.hypot(*offset_metres(fix.latitude, fix.longitude, centre)) if fix is not None and centre is not None else math.inf
      if fix is not None and now-last_attempt >= 5 and (distance >= 700 or now-stamp >= 300):
        last_attempt = now
        roads = self.reader.roads(fix)
        with self._lock:
          self._cache = (roads, fix, now) if roads is not None else ((), None, -math.inf)
      self._wake.wait(1)
      self._wake.clear()
