"""Use the system clock and remember a geographic timezone without continuous GPS."""
from datetime import datetime, UTC
from functools import lru_cache
import threading
from zoneinfo import ZoneInfo, ZoneInfoNotFoundError

from openpilot.common.swaglog import cloudlog


@lru_cache(maxsize=16)
def timezone(name):
  try:
    return ZoneInfo(name) if name else None
  except (ZoneInfoNotFoundError, ValueError):
    return None


def time_status(params):
  local = datetime.now(timezone(params.get('SentryTimeZone')) or UTC)
  return local.strftime('%Y-%m-%d %H:%M:%S %Z')


class SentryClock:
  def __init__(self, params):
    self.params = params
    self.zone_name = params.get('SentryTimeZone') or ''
    if timezone(self.zone_name) is None:
      self.zone_name = ''
    self._finder = None
    self._worker = None
    self._last_position = None
    self._next_lookup = 0.

  def update(self, location, now):
    if location is None or now < self._next_lookup or (self._worker is not None and self._worker.is_alive()):
      return
    point = location['latitude'], location['longitude']
    if point == self._last_position:
      return
    self._next_lookup = now+60.
    # Loading offline timezone polygons must not block the ignition/capture loop.
    self._worker = threading.Thread(target=self._resolve, args=(point,), daemon=True)
    self._worker.start()

  def _resolve(self, point):
    try:
      if self._finder is None:
        from timezonefinder import TimezoneFinder
        self._finder = TimezoneFinder(in_memory=False)
      name = self._finder.timezone_at(lat=point[0], lng=point[1])
      if timezone(name) is not None:
        if name != self.zone_name:
          self.params.put('SentryTimeZone', name, block=True)
          self.zone_name = name
        self._last_position = point
    except Exception:
      cloudlog.exception('sentry timezone lookup')

  def snapshot(self, wall):
    name = self.zone_name
    zone = timezone(name)
    if zone is None:
      return '', 0
    local = datetime.fromtimestamp(wall, zone)
    return name, int(local.utcoffset().total_seconds())
