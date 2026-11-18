"""GPS-referenced UTC and offline geographic timezone lookup for map schedules."""
import math
from datetime import UTC, datetime
from zoneinfo import ZoneInfo, ZoneInfoNotFoundError

UTC_MIN = datetime(2025, 1, 1, tzinfo=UTC).timestamp()
UTC_MAX = datetime(2035, 1, 1, tzinfo=UTC).timestamp()
CLOCK_MAX_AGE = 600.


class ZoneClock:
  def __init__(self):
    self._anchor = None
    self._finder = None
    self._prepared = False
    self._last_fix_timestamp = None
    self._zones = {}

  def prepare(self):
    if not self._prepared:
      self._prepared = True
      try:
        from timezonefinder import TimezoneFinder
        self._finder = TimezoneFinder(in_memory=False)
      except (ImportError, OSError, ValueError):
        pass  # optional map functionality; ordinary speeds remain available

  def observe(self, fix, now: float):
    # Never infer date/time from coordinates or from a parked position checkpoint.
    if fix is None or fix.estimated or not 0 <= now - fix.timestamp <= 3:
      return
    utc = fix.utc_timestamp
    if fix.timestamp == self._last_fix_timestamp:
      return
    self._last_fix_timestamp = fix.timestamp
    if utc is None or not math.isfinite(utc) or not UTC_MIN <= utc < UTC_MAX:
      return
    if self._anchor is not None:
      previous_utc, previous_mono = self._anchor
      if fix.timestamp == previous_mono:
        return
      if abs((utc - previous_utc) - (fix.timestamp - previous_mono)) > 2:
        self._anchor = None  # one contradictory fix cannot change an active schedule
        return
    self._anchor = utc, fix.timestamp

  def utc(self, now: float) -> float | None:
    if self._anchor is None:
      return None
    utc, monotonic = self._anchor
    age = now - monotonic
    return utc + age if 0 <= age <= CLOCK_MAX_AGE else None

  def _zone(self, latitude: float, longitude: float) -> str | None:
    key = latitude, longitude
    if key in self._zones:
      return self._zones[key]
    if self._finder is None:
      self.prepare()
    if self._finder is None:
      return None
    result = self._finder.timezone_at(lat=latitude, lng=longitude)
    if len(self._zones) >= 32:
      del self._zones[next(iter(self._zones))]
    self._zones[key] = result
    return result

  def local_time(self, fix, now: float) -> datetime | None:
    utc = self.utc(now)
    if utc is None or fix is None:
      return None
    try:
      # Exact coordinates, no city-radius cache that can leak across a boundary.
      # Sample the reported position allowance too; uncertain border -> unknown.
      delta = math.degrees(max(0., fix.accuracy) / 6371000.)
      lon_delta = delta / math.cos(math.radians(fix.latitude))
      points = [(fix.latitude, fix.longitude), (fix.latitude - delta, fix.longitude), (fix.latitude + delta, fix.longitude),
                (fix.latitude, fix.longitude - lon_delta), (fix.latitude, fix.longitude + lon_delta)]
      names = {self._zone(lat, (lon + 180) % 360 - 180) for lat, lon in points}
      if None in names:
        return None
      times = [datetime.fromtimestamp(utc, ZoneInfo(name)) for name in names]
      if not times or any(t.utcoffset() != times[0].utcoffset() for t in times):
        return None
      return times[0]
    except (ImportError, OSError, ValueError, OverflowError, ZoneInfoNotFoundError):
      return None
