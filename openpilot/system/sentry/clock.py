"""Use the system clock and remember a geographic timezone without continuous GPS."""
from datetime import datetime, UTC, timedelta, timezone as fixed_timezone
from functools import lru_cache
import math
import threading
import time
from zoneinfo import ZoneInfo, ZoneInfoNotFoundError

from openpilot.common.swaglog import cloudlog
from openpilot.common.hardware import HARDWARE
from openpilot.system.sentry.network_time import read_network_timezone

CELL_TIME_MAX_AGE = 300.


@lru_cache(maxsize=16)
def timezone(name):
  try:
    return ZoneInfo(name) if name else None
  except (ZoneInfoNotFoundError, ValueError):
    return None


def cellular_offset(modem, now):
  if not isinstance(modem, dict):
    return None
  offset = modem.get('network_timezone')
  observed = modem.get('network_timezone_mono')
  if (type(offset) is int and -12*3600 <= offset <= 14*3600 and offset % 900 == 0 and
      type(observed) in (int, float) and math.isfinite(observed) and 0 <= now-observed < CELL_TIME_MAX_AGE):
    return offset
  return None


def time_status(params, state=None):
  name = getattr(state, 'timeZone', '')
  if name == 'cellular' and -12*3600 <= state.utcOffsetSeconds <= 14*3600:
    zone = fixed_timezone(timedelta(seconds=state.utcOffsetSeconds))
    return datetime.now(zone).strftime('%Y-%m-%d %H:%M:%S %Z')+' (cellular)'
  local = datetime.now(timezone(name) or timezone(params.get('SentryTimeZone')) or UTC)
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
    self._next_cell_read = 0.
    self._cell_offset = None
    self._cell_sample = {}
    self._cell_worker = None
    self._next_cell_query = 0.

  def update(self, location, now):
    if now >= self._next_cell_read:
      self._next_cell_read = now+5.
      try:
        modem = HARDWARE.get_modem_state()
        self._cell_offset = cellular_offset(modem, now)
        if self._cell_offset is None:
          self._cell_offset = cellular_offset(self._cell_sample, now)
          # New AGNOS owns the modem process and does not publish its timezone.
          # Query only when geographic time is unavailable and the OS reports
          # a healthy EG25. Serial I/O cannot block capture/ignition handling.
          observed = modem.get('seconds_since_boot') if isinstance(modem, dict) else None
          if (not self.zone_name and isinstance(modem, dict) and str(modem.get('modem_version', '')).startswith('EG25') and
              modem.get('state') in ('SEARCHING', 'CONNECTED') and type(observed) in (int, float) and
              0 <= now-observed < 10. and now >= self._next_cell_query and
              (self._cell_worker is None or not self._cell_worker.is_alive())):
            self._next_cell_query = now+60.
            self._cell_worker = threading.Thread(target=self._read_network_timezone, daemon=True)
            self._cell_worker.start()
      except OSError:
        self._cell_offset = cellular_offset(self._cell_sample, now)
    if location is None and not self.zone_name:
      # A parking checkpoint can still identify the timezone when it doesn't
      # qualify as the route's current position. Never turn this into a GPS fix.
      location = self.params.get('MapParkedPosition')
    if location is None or now < self._next_lookup or (self._worker is not None and self._worker.is_alive()):
      return
    try:
      point = float(location['latitude']), float(location['longitude'])
      if not all(math.isfinite(v) for v in point) or not (-90 < point[0] < 90 and -180 <= point[1] <= 180):
        return
    except (KeyError, TypeError, ValueError, OverflowError):
      return
    if point == self._last_position:
      return
    self._next_lookup = now+60.
    # Loading offline timezone polygons must not block the ignition/capture loop.
    self._worker = threading.Thread(target=self._resolve, args=(point,), daemon=True)
    self._worker.start()

  def _read_network_timezone(self):
    offset = read_network_timezone()
    if offset is not None:
      self._cell_sample = {'network_timezone': offset, 'network_timezone_mono': time.monotonic()}

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
      return ('cellular', self._cell_offset) if self._cell_offset is not None else ('', 0)
    local = datetime.fromtimestamp(wall, zone)
    return name, int(local.utcoffset().total_seconds())
