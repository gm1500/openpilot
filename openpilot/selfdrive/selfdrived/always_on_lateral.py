"""Separate lateral-only intent from normal cruise engagement.

The host requests permission; the Panda independently grants it. Losing readiness
after a request suspends steering until a new driver action, rather than silently
resuming after a fault. No state here enables longitudinal control.
"""
import math
from dataclasses import dataclass

from openpilot.cereal import log
from openpilot.selfdrive.selfdrived.events import ET, EVENTS
from opendbc.car.gm.values import GMSafetyFlags
from opendbc.car.structs import car

EventName = log.OnroadEvent.EventName
IGNORED_SAFETY_MODES = (car.CarParams.SafetyModel.silent, car.CarParams.SafetyModel.noOutput)

# These events only prevent/disable cruise. All other entry, user-disable and
# system-fault gates remain in effect for lateral-only operation.
LONGITUDINAL_EVENTS = frozenset((EventName.pedalPressed, EventName.preEnableStandstill,
                                EventName.pcmDisable, EventName.resumeBlocked, EventName.belowEngageSpeed))
BLOCKING_TYPES = (ET.NO_ENTRY, ET.USER_DISABLE, ET.SOFT_DISABLE, ET.IMMEDIATE_DISABLE)


def lateral_events_clear(events, paused: bool = False) -> bool:
  ignored = LONGITUDINAL_EVENTS | ({EventName.wrongGear, EventName.reverseGear, EventName.parkBrake} if paused else set())
  return not any(name not in ignored and any(kind in EVENTS.get(name, {}) for kind in BLOCKING_TYPES)
                 for name in events.names)


def lateral_safety_supported(CP, panda_states) -> bool:
  if CP.passive or CP.notCar or CP.brand != 'gm' or not CP.openpilotLongitudinalControl or CP.pcmCruise:
    return False
  if len(panda_states) != len(CP.safetyConfigs):
    return False
  active = []
  for config, state in zip(CP.safetyConfigs, panda_states, strict=True):
    if config.safetyModel in IGNORED_SAFETY_MODES:
      if state.safetyModel not in IGNORED_SAFETY_MODES:
        return False
      continue
    active.append(state)
    if (config.safetyModel != car.CarParams.SafetyModel.gm or
        not (config.safetyParam & GMSafetyFlags.LATERAL_ONLY) or
        state.safetyModel != config.safetyModel or state.safetyParam != config.safetyParam or
        state.alternativeExperience != CP.alternativeExperience or not state.lateralSupported):
      return False
  return bool(active)


@dataclass
class AlwaysOnLateralState:
  supported: bool = False
  requested: bool = False
  active: bool = False
  suspended: bool = False
  paused: bool = False
  just_stopped: bool = False
  _enabled: bool = False
  _armed: bool = False
  _rearm_until: float = 0.

  def update(self, *, enabled: bool, onroad: bool, supported: bool, ready: bool,
             firmware_allowed: bool, firmware_suspended: bool, cancel: bool, rearm: bool, now: float, paused: bool = False):
    previous_active = self.active
    self.supported = supported
    self.paused = enabled and onroad and paused
    self.requested = self.active = False
    fresh_toggle = enabled and not self._enabled
    self._enabled = enabled
    if not enabled or not onroad:
      self.suspended = False
      self._armed = False
      self._rearm_until = 0.
    else:
      # A physical SET/RESUME or a genuine saved OFF->ON can rearm. A transient
      # loss of planner/DM readiness is deliberately not a rearm event.
      if (fresh_toggle or rearm) and math.isfinite(now):
        self.suspended = False
        self._rearm_until = now + .5
      if cancel or (self._armed and (not ready or not supported)):
        self.suspended = True
      if firmware_suspended and now >= self._rearm_until:
        self.suspended = True
      self.requested = bool(supported and ready and not self.paused and not self.suspended and math.isfinite(now))
      self.active = self.requested and firmware_allowed
      self._armed |= self.requested
    self.just_stopped = previous_active and not self.active
