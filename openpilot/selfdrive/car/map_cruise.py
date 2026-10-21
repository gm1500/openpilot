"""Map SET selection and SLC's automatic E2E request/recovery handshake."""
import math
from dataclasses import dataclass

from openpilot.common.constants import CV
from openpilot.selfdrive.car.slc_settings import SLCSettings

MAP_MESSAGE_MAX_AGE = 0.8
MAP_GPS_MAX_AGE = 3.0
MAP_STABLE_TIME = 2.0
MAP_MISSING_TIME = 1.0
MAP_ADVANCE_TIME = 2.0


@dataclass(frozen=True)
class MapPreview:
  speed: float
  distance: float
  way_id: int


def read_map_preview(sm, now: float, replay: bool = False) -> MapPreview | None:
  msg = sm['mapSpeedLimit']
  stamp = (msg.positionMonoTime if msg.positionEstimated else msg.gpsMonoTime) * 1e-9
  if (not sm.valid['mapSpeedLimit'] or not msg.nextSpeedLimitValid or not msg.headingValid or
      msg.nextSpeedLimitWayId == 0 or not 0 < stamp <= now or
      not 0 <= now - sm.logMonoTime['mapSpeedLimit'] * 1e-9 <= MAP_MESSAGE_MAX_AGE or
      not replay and not 0 <= now - sm.recv_time['mapSpeedLimit'] <= MAP_MESSAGE_MAX_AGE or
      now - stamp > (MAP_MESSAGE_MAX_AGE if msg.positionEstimated else MAP_GPS_MAX_AGE) or
      not 0 < msg.gpsMonoTime * 1e-9 <= now or now - msg.gpsMonoTime * 1e-9 > MAP_GPS_MAX_AGE or
      not math.isfinite(msg.nextSpeedLimit) or not 8 <= round(msg.nextSpeedLimit * CV.MS_TO_KPH, 3) <= 145 or
      not math.isfinite(msg.nextSpeedLimitDistance) or not 0 <= msg.nextSpeedLimitDistance <= 300):
    return None
  return MapPreview(msg.nextSpeedLimit, msg.nextSpeedLimitDistance, msg.nextSpeedLimitWayId)


def map_cruise_supported(CP) -> bool:
  return CP is not None and CP.openpilotLongitudinalControl and not (CP.pcmCruise or CP.notCar or CP.passive)


def fallback_e2e_ready(sm, now: float, replay: bool = False) -> bool:
  """SET may rise only after the planner acknowledges active no-speed E2E."""
  service = 'longitudinalPlan'
  if service not in sm.services:
    return False
  if (not sm.valid[service] or not 0 <= now - sm.logMonoTime[service] * 1e-9 <= .3 or
      not replay and not 0 <= now - sm.recv_time[service] <= .3):
    return False
  policy = sm[service].conditionalExperimental
  return policy.e2eEnabled and str(policy.state) == 'active' and policy.reason == 'noSpeedLimit'


def read_map_speed(sm, now: float, replay: bool = False) -> tuple[float | None, float]:
  """Reject stale publishers even if a receiver has only just seen their packet."""
  msg = sm['mapSpeedLimit']
  stamp = (msg.positionMonoTime if msg.positionEstimated else msg.gpsMonoTime) * 1e-9
  # Only the fresh, matched advisory can replace an absent legal limit. Legacy
  # display fields are never targets, and malformed legal values cannot fall back.
  speed = msg.advisorySpeed if msg.speedLimit == 0 else msg.speedLimit
  valid = (sm.valid['mapSpeedLimit'] and 0 < stamp <= now and
           0 <= now - sm.logMonoTime['mapSpeedLimit'] * 1e-9 <= MAP_MESSAGE_MAX_AGE and
           # Replay compares recorded CAN/GPS/publisher times, not host receive time.
           (replay or 0 <= now - sm.recv_time['mapSpeedLimit'] <= MAP_MESSAGE_MAX_AGE) and
           now - stamp <= (MAP_MESSAGE_MAX_AGE if msg.positionEstimated else MAP_GPS_MAX_AGE) and msg.headingValid and
           math.isfinite(speed) and 0 < speed <= 300 / 3.6)
  return (speed if valid else None), stamp


def read_map_display(sm, now: float) -> float | None:
  """Show the fresh map input to SET, before its current-driving-speed floor."""
  speed, _ = read_map_speed(sm, now)
  return speed if speed is not None and 8 <= round(speed * CV.MS_TO_KPH, 3) <= 145 else None


def lead_limits_speed(sm, started_frame: int, now: float) -> bool:
  """Hide adjustment feedback only when a fresh plan selects a real lead."""
  for service in ('longitudinalPlan', 'radarState'):
    if (not sm.valid[service] or sm.recv_frame[service] < started_frame or
        not 0 <= now - sm.logMonoTime[service] * 1e-9 <= 0.3 or
        not 0 <= now - sm.recv_time[service] <= 0.3):
      return False
  plan, radar = sm['longitudinalPlan'], sm['radarState']
  source = str(plan.longitudinalPlanSource)
  if source == 'lead1':
    return radar.leadTwo.present
  return radar.leadOne.present and (source == 'lead0' or (source == 'e2e' and plan.e2eAssistActive))


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
    self.automatic_e2e = self.e2e_fallback = self.fallback_set_pending = False
    self._missing_since: float | None = None
    self.settings = SLCSettings()
    self._previous_settings = self.settings
    self.metric = True
    self._previous_metric = self.metric
    self._manual_override = False
    self.current_kph = 255.
    self._preview_key = None
    self._preview_since = self._preview_first_fix = self._preview_last_fix = self._preview_seen = 0.
    self._preview_qualified = self.preview_active = False

  def _anticipate(self, speed_ms, gps_time, now, preview, way_id, v_ego):
    self.preview_active = False
    if not self.enabled or not self.settings.advance:
      self._preview_key = None
      self._preview_qualified = False
      return speed_ms, None
    # Carry qualification across the boundary only for the SAME identified way
    # and limit, with a fresh position. A different/changed road starts normally.
    entered = (self._preview_qualified and self._preview_key is not None and speed_ms is not None and
               self._preview_key == (way_id, round(speed_ms, 4)) and 0 <= now - self._preview_seen <= MAP_GPS_MAX_AGE and
               0 < gps_time <= now and now - gps_time <= MAP_GPS_MAX_AGE and gps_time >= self._preview_last_fix)
    qualified_fix = self._preview_first_fix if entered else None
    valid = (preview is not None and 0 < gps_time <= now and now - gps_time <= MAP_GPS_MAX_AGE and
             math.isfinite(preview.speed) and 8 <= round(preview.speed * CV.MS_TO_KPH, 3) <= 145 and preview.way_id > 0 and
             math.isfinite(preview.distance) and 0 <= preview.distance <= 300 and math.isfinite(v_ego) and 2 <= v_ego <= 75)
    if valid:
      key = preview.way_id, round(preview.speed, 4)
      if (key != self._preview_key or not 0 <= now - self._preview_seen <= MAP_MESSAGE_MAX_AGE or gps_time < self._preview_last_fix):
        self._preview_since, self._preview_first_fix = now, gps_time
        self._preview_qualified = False
      self._preview_key, self._preview_seen, self._preview_last_fix = key, now, gps_time
      self._preview_qualified |= now - self._preview_since >= MAP_STABLE_TIME and gps_time > self._preview_first_fix
      target = self.settings.target(preview.speed * CV.MS_TO_KPH, self.metric)
      # Higher limits are qualified ahead but applied only on matched entry.
      # An early lower target can coexist with no-speed E2E until actual recovery.
      lowering = 8 <= self.current_kph <= 145 and target <= self.current_kph and (speed_ms is None or preview.speed < speed_ms)
      if self._preview_qualified and preview.distance <= v_ego * MAP_ADVANCE_TIME and lowering:
        self.preview_active = True
        return preview.speed, self._preview_first_fix
    elif not 0 <= now - self._preview_seen <= MAP_GPS_MAX_AGE or gps_time < self._preview_last_fix:
      # A short map gap cannot apply an early target. Retain only its already
      # earned qualification, for a fresh exact-way/limit match at the boundary.
      self._preview_key = None
      self._preview_qualified = False
    return speed_ms, qualified_fix

  def update(self, enabled: bool, speed_ms: float | None, gps_time: float, now: float, automatic_e2e: bool = False,
             *, preview=None, way_id=0, v_ego=0.):
    previous_target = self.target_kph
    settings_changed = self.settings != self._previous_settings or self.metric != self._previous_metric
    enabled = enabled and self.supported
    if enabled and not self.enabled:
      self.tracking = True
      self._manual_override = False
    self.enabled = enabled
    actual_speed = speed_ms
    speed_ms, qualified_fix = self._anticipate(speed_ms, gps_time, now, preview, way_id, v_ego)
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
      if qualified_fix is not None:
        self._since = now - MAP_STABLE_TIME
        self._first_fix = qualified_fix
    self._last_fix = gps_time
    # A single frozen GPS fix cannot qualify a new target merely by waiting.
    if candidate is not None and now - self._since >= MAP_STABLE_TIME and gps_time > self._first_fix:
      self.target_kph = self.settings.target(candidate, self.metric)
      if settings_changed and not self.settings.accepts(self.target_kph, self.current_kph):
        self.tracking = False
      self._previous_settings, self._previous_metric = self.settings, self.metric
      if self.enabled and candidate != self._last_qualified:
        permitted = self.settings.accepts(self.target_kph, self.current_kph)
        if not self._manual_override or self.settings.resume_policy == 0:
          self.tracking = permitted
          if permitted:
            self._manual_override = False
      # A brief map gap recovering the SAME limit is not a new speed zone.
      # Preserve manual selection unless the no-speed E2E fallback recovers.
      self._last_qualified = candidate
    self.automatic_e2e = self.enabled and automatic_e2e
    if not self.automatic_e2e:
      self.e2e_fallback = self.fallback_set_pending = False
      self._missing_since = None
    elif candidate is None or self.preview_active and actual_speed is None:
      if self._missing_since is None or now < self._missing_since:
        self._missing_since = now
      if now - self._missing_since >= MAP_MISSING_TIME and not self.e2e_fallback:
        # A new no-speed interval owns its initial SET even if an earlier
        # manual adjustment paused SLC. Later buttons can still cancel it.
        self.request_fallback()
    else:
      self._missing_since = None
      if self.e2e_fallback and previous_target is None and self.target_kph is not None:
        # A real recovery resumes SLC even if it is the same pre-outage limit.
        # Buttons processed after this update can still override it explicitly.
        self.tracking = (self.settings.accepts(self.target_kph, self.current_kph) and
                         (not self._manual_override or self.settings.resume_policy == 0))

  def request_fallback(self):
    if self.automatic_e2e:
      self.e2e_fallback = True
      self.fallback_set_pending = True

  def finish_recovery(self, set_kph: float):
    # Keep E2E requested until the mapped SET has been applied (or the driver
    # explicitly chooses a manual SET). Publishing follows the carState update.
    if not self.preview_active and self.target_kph is not None and (abs(set_kph - self.target_kph) < .01 or not self.tracking):
      self.e2e_fallback = self.fallback_set_pending = False

  @property
  def pending_kph(self) -> float | None:
    """Preview qualification without changing tracking or the cruise target."""
    if self.enabled and self.target_kph is None and (self.tracking or self._candidate != self._last_qualified):
      return self.settings.target(self._candidate, self.metric) if self._candidate is not None else None
    return None

  def select_current(self):
    """An explicit SET accepts a fresh match without the automatic debounce."""
    if self.enabled:
      self.tracking = True
      self._manual_override = False
      self._previous_settings, self._previous_metric = self.settings, self.metric
      if self._candidate is not None:
        self._last_qualified = self._candidate
        self.target_kph = self.settings.target(self._candidate, self.metric)

  def pause(self):
    """A manual selection also wins over this limit's pending qualification."""
    self.tracking = False
    self._manual_override = True
    self.fallback_set_pending = False
    if self._candidate is not None:
      self._last_qualified = self._candidate

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

  def start(self, target_kph: float):
    """Explicit map SET starts fresh, even when the same limit was cancelled."""
    self.cancel()
    self.target_kph = self._seen_target = target_kph
    self._interrupted = False
    self._applied = True

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
