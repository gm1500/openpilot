import math
import numpy as np

from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.selfdrive.car.map_cruise import MapCruise, MapCruisePulse


# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
# V_CRUISE's are in kph
V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255
V_CRUISE_INITIAL = 40
V_CRUISE_INITIAL_EXPERIMENTAL_MODE = 105
IMPERIAL_INCREMENT = round(CV.MPH_TO_KPH, 1)  # round here to avoid rounding errors incrementing set speed

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
CRUISE_LONG_PRESS = 50
CRUISE_NEAREST_FUNC = {
  ButtonType.accelCruise: math.ceil,
  ButtonType.decelCruise: math.floor,
}
CRUISE_INTERVAL_SIGN = {
  ButtonType.accelCruise: +1,
  ButtonType.decelCruise: -1,
}


class VCruiseHelper:
  def __init__(self, CP):
    self.CP = CP
    self.v_cruise_kph = V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_UNSET
    self.v_cruise_kph_last = 0
    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False, "enabled": False} for btn in self.button_timers}
    self.map_cruise = MapCruise(CP)
    self.map_pulse = MapCruisePulse()
    self._map_set_held = False

  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  def update_v_cruise(self, CS, enabled, is_metric, *, map_enabled=False, map_speed=None, map_gps_time=0., now=0.,
                      automatic_e2e=False, e2e_ready=False):
    self.v_cruise_kph_last = self.v_cruise_kph
    self.map_cruise.update(map_enabled, map_speed, map_gps_time, now, automatic_e2e)
    map_applied = False

    if CS.cruiseState.available:
      if not self.CP.pcmCruise:
        # if stock cruise is completely disabled, then we can use our own set speed logic
        self._update_v_cruise_non_pcm(CS, enabled, is_metric)
        self.update_button_timers(CS, enabled)
        if enabled and not any(self.button_timers.values()) and not (CS.gasPressed or CS.brakePressed):
          map_applied = self._apply_map_speed()
          if (self.map_cruise.e2e_fallback and self.map_cruise.fallback_set_pending and
              self.map_cruise.target_kph is None and e2e_ready):
            self.v_cruise_kph = V_CRUISE_INITIAL_EXPERIMENTAL_MODE
            self.map_cruise.fallback_set_pending = False
        self.map_cruise.finish_recovery(self.v_cruise_kph)
        self.v_cruise_cluster_kph = self.v_cruise_kph
      else:
        self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
        self.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
        if CS.cruiseState.speed == 0:
          self.v_cruise_kph = V_CRUISE_UNSET
          self.v_cruise_cluster_kph = V_CRUISE_UNSET
        elif CS.cruiseState.speed == -1:
          self.v_cruise_kph = -1
          self.v_cruise_cluster_kph = -1
    else:
      self.v_cruise_kph = V_CRUISE_UNSET
      self.v_cruise_cluster_kph = V_CRUISE_UNSET

    candidate = self.map_cruise.pending_kph
    if candidate is None and self.map_cruise.tracking:
      candidate = self.map_cruise.target_kph
    # Finishing the SET that engaged cruise is not a subsequent manual override.
    # This exemption only affects feedback; ordinary +/- and hold behavior stays above.
    held_override = any(timer and not (button == ButtonType.decelCruise and self._map_set_held)
                        for button, timer in self.button_timers.items())
    map_selected = self.map_cruise.enabled and self.map_cruise.tracking and self.map_cruise.target_kph == self.v_cruise_kph
    button_override = any(
      b.type in (ButtonType.accelCruise, ButtonType.decelCruise, ButtonType.setCruise, ButtonType.resumeCruise, ButtonType.cancel)
      and not (not b.pressed and ((b.type == ButtonType.decelCruise and self._map_set_held) or
                                 (b.type == ButtonType.setCruise and map_selected))) for b in CS.buttonEvents)
    intervention = CS.gasPressed or CS.brakePressed or held_override or button_override
    if not enabled or any(b.type == ButtonType.decelCruise for b in CS.buttonEvents):
      self._map_set_held = False
    self.map_pulse.update(candidate, self.v_cruise_kph, CS.vEgo * CV.MS_TO_KPH, now,
                          enabled and CS.cruiseState.available and self.map_cruise.enabled,
                          intervention, map_applied)

  def _update_v_cruise_non_pcm(self, CS, enabled, is_metric):
    # handle button presses. TODO: this should be in state_control, but a decelCruise press
    # would have the effect of both enabling and changing speed is checked after the state transition
    if not enabled:
      return

    if self.map_cruise.enabled:
      for b in CS.buttonEvents:
        if b.type in (ButtonType.resumeCruise, ButtonType.cancel):
          self.map_cruise.pause()
        elif b.type == ButtonType.setCruise and not b.pressed:
          self._select_map_speed(CS)

    long_press = False
    button_type = None

    v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers and not b.pressed:
        if self.button_timers[b.type.raw] > CRUISE_LONG_PRESS:
          if self.map_cruise.enabled:
            self.map_cruise.pause()  # the completed manual hold wins over a concurrent map change
          return  # end long press
        button_type = b.type.raw
        break
    else:
      for k, timer in self.button_timers.items():
        if timer and timer % CRUISE_LONG_PRESS == 0:
          button_type = k
          long_press = True
          break

    if button_type is None:
      return

    # Don't adjust speed when pressing resume to exit standstill
    cruise_standstill = self.button_change_states[button_type]["standstill"] or CS.cruiseState.standstill
    if button_type == ButtonType.accelCruise and cruise_standstill:
      return

    # Don't adjust speed if we've enabled since the button was depressed (some ports enable on rising edge)
    if not self.button_change_states[button_type]["enabled"]:
      return

    if self.map_cruise.enabled:
      # Once engaged, SET/- and RES/+ are ordinary speed adjustments. Map
      # selection belongs to engagement or a separate, dedicated SET event.
      self.map_cruise.pause()

    v_cruise_delta = v_cruise_delta * (5 if long_press else 1)
    if long_press and self.v_cruise_kph % v_cruise_delta != 0:  # partial interval
      self.v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](self.v_cruise_kph / v_cruise_delta) * v_cruise_delta
    else:
      self.v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

    # If set is pressed while overriding, clip cruise speed to minimum of vEgo
    if CS.gasPressed and button_type in (ButtonType.decelCruise, ButtonType.setCruise):
      self.v_cruise_kph = max(self.v_cruise_kph, CS.vEgo * CV.MS_TO_KPH)

    self.v_cruise_kph = np.clip(round(self.v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)

  def update_button_timers(self, CS, enabled):
    # increment timer for buttons still pressed
    for k in self.button_timers:
      if self.button_timers[k] > 0:
        self.button_timers[k] += 1

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers:
        # Start/end timer and store current state on change of button pressed
        self.button_timers[b.type.raw] = 1 if b.pressed else 0
        self.button_change_states[b.type.raw] = {"standstill": CS.cruiseState.standstill, "enabled": enabled}

  def initialize_v_cruise(self, CS, experimental_mode: bool) -> None:
    self.map_pulse.cancel()
    self._map_set_held = bool(self.button_timers[ButtonType.decelCruise])
    # initializing is handled by the PCM
    if self.CP.pcmCruise:
      return

    initial = V_CRUISE_INITIAL_EXPERIMENTAL_MODE if experimental_mode else V_CRUISE_INITIAL

    resume = any(b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents) and self.v_cruise_initialized
    if resume:
      self.v_cruise_kph = self.v_cruise_kph_last
    else:
      self.v_cruise_kph = int(round(np.clip(CS.vEgo * CV.MS_TO_KPH, initial, V_CRUISE_MAX)))

    if self.map_cruise.enabled:
      if resume:
        self.map_cruise.pause()
      else:
        self._select_map_speed(CS)

    self.v_cruise_cluster_kph = self.v_cruise_kph

  def _apply_map_speed(self):
    if self.map_cruise.enabled and self.map_cruise.tracking and self.map_cruise.target_kph is not None and self.v_cruise_initialized:
      changed = abs(self.v_cruise_kph - self.map_cruise.target_kph) > 0.01
      self.v_cruise_kph = self.map_cruise.target_kph
      return changed
    return False

  def _select_map_speed(self, CS):
    """Explicit SET uses the map as its minimum, including during pedal override."""
    self.map_cruise.select_current()
    target = self.map_cruise.target_kph
    if target is not None:
      self.v_cruise_kph = max(target, round(float(np.clip(CS.vEgo * CV.MS_TO_KPH, V_CRUISE_MIN, V_CRUISE_MAX))))
      # Like a manual selection, a higher driving speed must not be immediately
      # undone by the same map limit after the pedal is released.
      self.map_cruise.tracking = self.v_cruise_kph == target
      if self.map_cruise.tracking and not (CS.gasPressed or CS.brakePressed):
        self.map_pulse.start(target)
      self.map_cruise.finish_recovery(self.v_cruise_kph)
    elif self.map_cruise.enabled:
      # Explicit SET bypasses the missing-data debounce, but SET 105 waits for
      # the planner's fresh E2E acknowledgement in update_v_cruise.
      self.map_cruise.request_fallback()
