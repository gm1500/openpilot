"""Map SET selection and SLC's automatic E2E request/recovery handshake."""
import math
from dataclasses import dataclass

from openpilot.common.constants import CV
from openpilot.selfdrive.car.slc_settings import SLCSettings

MAP_MESSAGE_MAX_AGE = 0.8
MAP_GPS_MAX_AGE = 3.0
MAP_MISSING_TIME = 1.0
MAP_ADVANCE_TIME = 2.0
MAP_MERGE_ADVANCE_TIME = 8.0
MAP_APPROACH_DECEL = 1.0


@dataclass(frozen=True)
class MapPreview:
  speed: float
  distance: float
  way_id: int
  advisory: bool = False
  merge: bool = False


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
      not math.isfinite(msg.nextSpeedLimitDistance) or not 0 <= msg.nextSpeedLimitDistance <= 1000):
    return None
  return MapPreview(msg.nextSpeedLimit, msg.nextSpeedLimitDistance, msg.nextSpeedLimitWayId,
                    msg.nextSpeedLimitAdvisory, msg.nextSpeedLimitMerge)


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
  return sm[service].slcE2eFallback


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


def real_lead_present(sm, started_frame: int, now: float) -> bool:
  """Hide adjustment feedback whenever fresh radar data contains a real lead."""
  service = 'radarState'
  if (not sm.valid[service] or sm.recv_frame[service] < started_frame or
      not 0 <= now - sm.logMonoTime[service] * 1e-9 <= 0.3 or
      not 0 <= now - sm.recv_time[service] <= 0.3):
    return False
  radar = sm[service]
  return radar.leadOne.present or radar.leadTwo.present


class MapCruise:
  def __init__(self, CP):
    self.supported = map_cruise_supported(CP)
    self.enabled = False
    self.tracking = True
    self.target_kph: float | None = None
    self._candidate: float | None = None
    self._last_fix = 0.
    self._last_current_limit: float | None = None
    self._last_preview_limit: float | None = None
    self.automatic_e2e = self.e2e_fallback = self.fallback_set_pending = False
    self._missing_since: float | None = None
    self.settings = SLCSettings()
    self._previous_settings = self.settings
    self.metric = True
    self._previous_metric = self.metric
    self._manual_override = False
    self.driver_speed_override = False
    self.current_kph = 255.
    self.preview_active = False
    self.preview_speed_ms: float | None = None
    self.preview_advisory = False

  def _anticipate(self, speed_ms, gps_time, now, preview, v_ego):
    self.preview_active = False
    self.preview_speed_ms = None
    self.preview_advisory = False
    if not self.enabled or not self.settings.advance:
      return speed_ms
    valid = (preview is not None and 0 < gps_time <= now and now - gps_time <= MAP_GPS_MAX_AGE and
             gps_time >= self._last_fix and
             math.isfinite(preview.speed) and 8 <= round(preview.speed * CV.MS_TO_KPH, 3) <= 145 and preview.way_id > 0 and
             math.isfinite(preview.distance) and 0 <= preview.distance <= 1000 and math.isfinite(v_ego) and 2 <= v_ego <= 75)
    if valid:
      target = self.settings.target(preview.speed * CV.MS_TO_KPH, self.metric)
      # A matched preview is available immediately. Display remains independent
      # of the distance and road-entry checks that determine when SET changes.
      if speed_ms is None or round(preview.speed, 4) != round(speed_ms, 4):
        self.preview_speed_ms = preview.speed
        self.preview_advisory = preview.advisory
      # Earlier braking uses the distance needed to shed speed plus two seconds
      # of travel. This is a distance allowance, not a timer before applying SET.
      # Raising SET early requires mapd's unique, aligned, model-confirmed merge.
      lowering = 8 <= self.current_kph <= 145 and target <= self.current_kph and (speed_ms is None or preview.speed < speed_ms)
      braking_distance = max(0.0, v_ego ** 2 - preview.speed ** 2) / (2 * MAP_APPROACH_DECEL)
      lower_range = min(300.0, max(50.0, v_ego * MAP_ADVANCE_TIME + braking_distance))
      merging = preview.merge and not preview.advisory and speed_ms is None and 8 <= self.current_kph <= 145
      in_range = lowering and preview.distance <= lower_range or merging and preview.distance <= min(250.0, v_ego * MAP_MERGE_ADVANCE_TIME)
      if in_range:
        self.preview_active = True
        return preview.speed
    return speed_ms

  def update(self, enabled: bool, speed_ms: float | None, gps_time: float, now: float, automatic_e2e: bool = False,
             *, preview=None, v_ego=0.):
    previous_target = self.target_kph
    settings_changed = self.settings != self._previous_settings or self.metric != self._previous_metric
    enabled = enabled and self.supported
    if enabled and not self.enabled:
      self.tracking = True
      self._manual_override = False
      self.driver_speed_override = False
    self.enabled = enabled
    actual_speed = speed_ms
    speed_ms = self._anticipate(speed_ms, gps_time, now, preview, v_ego)
    speed = speed_ms * CV.MS_TO_KPH if speed_ms is not None else math.nan
    # Do not turn an unsupported walking-zone/highway limit into a different limit
    # by clipping it. Ordinary cruise retains its existing 8..145 km/h bounds.
    speed = round(speed, 3) if math.isfinite(speed) else math.nan
    candidate = speed if 8 <= speed <= 145 else None
    if not (0 < gps_time <= now and now - gps_time <= MAP_GPS_MAX_AGE and gps_time >= self._last_fix):
      candidate = None
    self._candidate = candidate
    self.target_kph = self.settings.target(candidate, self.metric) if candidate is not None else None
    # mapd already validates the road match; an additional warning/debounce here
    # leaves the previous SET (including the E2E ceiling) active after map recovery.
    # Reusing a still-fresh GPS fix is valid; an older or expired fix is not.
    if candidate is not None:
      self._last_fix = gps_time
      actual_kph = actual_speed * CV.MS_TO_KPH if actual_speed is not None else math.nan
      actual_kph = round(actual_kph, 3) if math.isfinite(actual_kph) else math.nan
      new_limit = 8 <= actual_kph <= 145 and actual_kph != self._last_current_limit
      if new_limit:
        self._last_current_limit = actual_kph
        self._last_preview_limit = None
      if self.preview_active:
        new_limit |= candidate != self._last_preview_limit
        self._last_preview_limit = candidate
      if settings_changed and not self.settings.accepts(self.target_kph, self.current_kph):
        self.tracking = False
      self._previous_settings, self._previous_metric = self.settings, self.metric
      # Withdrawing a preview after manual SET drops below it exposes the same
      # current road again. That source switch is not a new speed zone and must
      # not resume SLC or erase the preview limit across a brief data gap.
      if self.enabled and new_limit:
        permitted = self.settings.accepts(self.target_kph, self.current_kph)
        if not self._manual_override or self.settings.resume_policy == 0:
          self.tracking = permitted
          if permitted:
            self._manual_override = False
            self.driver_speed_override = False
    self.automatic_e2e = self.enabled and automatic_e2e
    if not self.automatic_e2e:
      self.e2e_fallback = self.fallback_set_pending = False
      self._missing_since = None
    elif candidate is None or self.preview_active and actual_speed is None:
      if self._missing_since is None or now < self._missing_since:
        self._missing_since = now
      if now - self._missing_since >= MAP_MISSING_TIME and not self.e2e_fallback:
        # A missing map may change the planning mode, never the driver's SET.
        # An explicit SET can reapply Initial Max Speed during the gap.
        self.request_fallback()
    else:
      self._missing_since = None
      if self.e2e_fallback and previous_target is None and self.target_kph is not None and not self.driver_speed_override:
        # A recovery resumes SLC unless the driver overrode this mapped limit.
        # An adjustment made with no known limit uses the configured resume policy.
        self.tracking = (self.settings.accepts(self.target_kph, self.current_kph) and
                         (not self._manual_override or self.settings.resume_policy == 0))

  def request_fallback(self, *, apply_ceiling: bool = False):
    if self.automatic_e2e:
      self.e2e_fallback = True
      self.fallback_set_pending = apply_ceiling

  def finish_recovery(self, set_kph: float):
    # Keep E2E requested until the mapped SET has been applied (or the driver
    # explicitly chooses a manual SET). Publishing follows the carState update.
    if not self.preview_active and self.target_kph is not None and (abs(set_kph - self.target_kph) < .01 or not self.tracking):
      self.e2e_fallback = self.fallback_set_pending = False

  def select_current(self):
    """An explicit SET accepts a fresh match even when automatic following is paused."""
    if self.enabled:
      self.tracking = True
      self._manual_override = False
      self.driver_speed_override = False
      self._previous_settings, self._previous_metric = self.settings, self.metric
      if self._candidate is not None:
        self.target_kph = self.settings.target(self._candidate, self.metric)

  def pause(self, *, driver_adjusted_speed: bool = False):
    """A manual selection wins over this limit's automatic application."""
    self.tracking = False
    self._manual_override = True
    if driver_adjusted_speed and self.target_kph is not None:
      self.driver_speed_override = True
    self.fallback_set_pending = False

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
