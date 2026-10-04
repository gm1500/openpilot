"""Optional map target selection; no actuator, planner or engagement policy."""
import math

from openpilot.common.constants import CV

MAP_MESSAGE_MAX_AGE = 0.8
MAP_GPS_MAX_AGE = 3.0
MAP_STABLE_TIME = 2.0


def map_cruise_supported(CP) -> bool:
  return CP is not None and CP.openpilotLongitudinalControl and not (CP.pcmCruise or CP.notCar or CP.passive)


def read_map_speed(sm, now: float, require_heading: bool = True, replay: bool = False) -> tuple[float | None, float]:
  """Reject stale publishers even if a receiver has only just seen their packet."""
  msg = sm['mapSpeedLimit']
  stamp = msg.gpsMonoTime * 1e-9
  valid = (sm.valid['mapSpeedLimit'] and 0 < stamp <= now and
           0 <= now - sm.logMonoTime['mapSpeedLimit'] * 1e-9 <= MAP_MESSAGE_MAX_AGE and
           # Replay compares recorded CAN/GPS/publisher times, not host receive time.
           (replay or 0 <= now - sm.recv_time['mapSpeedLimit'] <= MAP_MESSAGE_MAX_AGE) and
           now - stamp <= MAP_GPS_MAX_AGE and (msg.headingValid or not require_heading) and
           math.isfinite(msg.speedLimit) and 0 < msg.speedLimit <= 300 / 3.6)
  return (msg.speedLimit if valid else None), stamp


def read_map_display(sm, now: float) -> float | None:
  """Legal/advisory display has independent validity; Event.valid is for control only."""
  msg = sm['mapSpeedLimit']
  valid = (msg.displayValid and 0 < msg.displayGpsMonoTime * 1e-9 <= now and
           now - msg.displayGpsMonoTime * 1e-9 <= 2.0 and
           0 <= now - sm.logMonoTime['mapSpeedLimit'] * 1e-9 <= MAP_MESSAGE_MAX_AGE and
           0 <= now - sm.recv_time['mapSpeedLimit'] <= MAP_MESSAGE_MAX_AGE and
           math.isfinite(msg.displaySpeedLimit) and 0 < msg.displaySpeedLimit <= 300 / 3.6)
  return msg.displaySpeedLimit if valid else None


class MapCruise:
  def __init__(self, CP):
    self.supported = map_cruise_supported(CP)
    self.enabled = False
    self.tracking = True
    self.target_kph: float | None = None
    self._candidate: float | None = None
    self._since = 0.
    self._first_fix = 0.
    self._last_fix = 0.
    self._last_qualified: float | None = None

  def update(self, enabled: bool, speed_ms: float | None, gps_time: float, now: float):
    enabled = enabled and self.supported
    if enabled and not self.enabled:
      self.tracking = True
    self.enabled = enabled
    speed = speed_ms * CV.MS_TO_KPH if speed_ms is not None else math.nan
    # Do not turn an unsupported walking-zone/highway limit into a different limit
    # by clipping it. Ordinary cruise retains its existing 8..145 km/h bounds.
    speed = round(speed, 3) if math.isfinite(speed) else math.nan
    candidate = speed if 8 <= speed <= 145 else None
    if not (0 < gps_time <= now and now - gps_time <= MAP_GPS_MAX_AGE):
      candidate = None
    if candidate is None or candidate != self._candidate or gps_time < self._last_fix:
      self._candidate, self._since, self._first_fix = candidate, now, gps_time
      self.target_kph = None
    self._last_fix = gps_time
    # A single frozen GPS fix cannot qualify a new target merely by waiting.
    if candidate is not None and now - self._since >= MAP_STABLE_TIME and gps_time > self._first_fix:
      self.target_kph = candidate
      if self.enabled and candidate != self._last_qualified:
        self.tracking = True
      # Keep this across missing/ambiguous data: recovering the SAME limit is
      # not a new speed zone and must not undo RES or a manual adjustment.
      self._last_qualified = candidate

  @property
  def pending_kph(self) -> float | None:
    """Preview qualification without changing tracking or the cruise target."""
    if self.enabled and self.target_kph is None and (self.tracking or self._candidate != self._last_qualified):
      return self._candidate
    return None

  def select_current(self):
    """An explicit SET accepts a fresh match without the automatic debounce."""
    if self.enabled:
      self.tracking = True
      if self._candidate is not None:
        self.target_kph = self._last_qualified = self._candidate

  def state(self, engaged: bool) -> str:
    if not self.supported:
      return 'unsupported'
    if not self.enabled:
      return 'off'
    if not self.tracking:
      return 'paused'
    if self.target_kph is None:
      return 'waiting'
    return 'active' if engaged else 'armed'


class MapCruisePulse:
  """Read-only driver feedback; never feeds the speed selector or actuators."""
  def __init__(self):
    self.target_kph: float | None = None
    self._seen_target: float | None = None
    self._interrupted = False
    self._applied = False
    self._settled_since: float | None = None

  def cancel(self):
    self.target_kph = None
    self._applied = False
    self._settled_since = None

  def update(self, candidate: float | None, set_kph: float, speed_kph: float, now: float,
             active: bool, intervention: bool, applied: bool):
    changed = candidate is not None and candidate != self._seen_target
    if changed:
      self._seen_target = candidate
      self._interrupted = False
    if not active or intervention or not math.isfinite(speed_kph):
      self._interrupted = True
      self.cancel()
      return
    if changed:
      self.cancel()
    if (changed or applied) and not self._interrupted and candidate is not None and (applied or abs(candidate - set_kph) > 0.01):
      self.target_kph = candidate
    if self.target_kph is None:
      return
    if abs(set_kph - self.target_kph) <= 0.01:
      self._applied = True
    elif self._applied or candidate != self.target_kph:
      self.cancel()
      return
    # A brief crossing of the target is not settled speed. Never restart a
    # completed/cancelled pulse merely because vehicle speed later drifts.
    if self._applied and abs(speed_kph - self.target_kph) <= 1.0:
      if self._settled_since is None or now < self._settled_since:
        self._settled_since = now
      if now - self._settled_since >= 1.0:
        self.cancel()
    else:
      self._settled_since = None
