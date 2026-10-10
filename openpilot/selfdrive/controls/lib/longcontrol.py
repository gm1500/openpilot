import numpy as np
from opendbc.car.structs import car
from opendbc.car.gm.values import CAR as GM_CAR
from openpilot.common.realtime import DT_CTRL
from openpilot.common.pid import PIDController

LAUNCH_INTEGRATOR_MAX_SPEED = 1.0  # m/s; only suppress standstill/creep launch windup

LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(active, long_control_state, should_stop, brake_pressed, cruise_standstill):
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.pid:
      if should_stop:
        long_control_state = LongCtrlState.stopping

  return long_control_state

class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController(0.0, (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.last_output_accel = 0.0
    self.launch_integrator_hold = 0.0

  def reset(self):
    self.pid.reset()
    self.launch_integrator_hold = 0.0

  def update(self, active, CS, a_target, should_stop, accel_limits):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    previous_state = self.long_control_state
    # Sierra direct longitudinal control owns brake release. Waiting for stock
    # ACC's standstill bit can keep our hold applied after the planner departs.
    cruise_standstill = CS.cruiseState.standstill and not (
      self.CP.carFingerprint == GM_CAR.CHEVROLET_SILVERADO and self.CP.openpilotLongitudinalControl)
    self.long_control_state = long_control_state_trans(active, self.long_control_state, should_stop,
                                                       CS.brakePressed, cruise_standstill)
    if (
      previous_state == LongCtrlState.stopping and self.long_control_state == LongCtrlState.pid
      and self.CP.carFingerprint == GM_CAR.CHEVROLET_SILVERADO
      and CS.vEgo < LAUNCH_INTEGRATOR_MAX_SPEED and a_target > 0.0
    ):
      # The feedforward command is already sent immediately. Do not learn the
      # expected acceleration error before the configured actuator delay has
      # elapsed; route 288 showed that this otherwise creates a large positive
      # integral at launch which can survive into the next braking event.
      self.launch_integrator_hold = max(float(self.CP.longitudinalActuatorDelay), 0.0)

    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        # TODO: can we just go straight to stopAccel?
        output_accel -= 1.0 * DT_CTRL  # m/s^2/s while trying to stop
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo
      freeze_integrator = self.launch_integrator_hold > 0.0
      output_accel = self.pid.update(error, speed=CS.vEgo, feedforward=a_target,
                                     freeze_integrator=freeze_integrator)
      self.launch_integrator_hold = max(0.0, self.launch_integrator_hold - DT_CTRL)

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel
