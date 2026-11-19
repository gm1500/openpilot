#!/usr/bin/env python3
import os
import time
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.structs import car
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.curve_speed import CurveSpeed
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.e2e_slowing import E2ESlowingAssist, get_model_lead_speed
from openpilot.selfdrive.controls.lib.e2e_stop import E2EStopTarget
from openpilot.selfdrive.controls.lib.lead_stop import get_lead_distance, get_lead_stop_distance
from openpilot.selfdrive.controls.lib.longitudinal_mode import slc_e2e_fallback
from openpilot.selfdrive.car.map_cruise import map_cruise_supported
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan, should_stop, smooth_value
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog

REPLAY = "REPLAY" in os.environ

J_CRUISE_VALS = [1.6, 1.2, 0.8, 0.6]
J_CRUISE_BP = [0., 10.0, 25., 40.]
A_CRUISE_MIN = -0.5
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]


def get_cruise_accel(v_cruise, v_ego, a_cruise_prev, a_target_prev, dt):
  target_accel = smooth_value(v_cruise - v_ego, a_cruise_prev, tau=1.0, dt=dt)
  target_accel = np.clip(target_accel, A_CRUISE_MIN, ACCEL_MAX)
  j_cruise = np.interp(v_ego, J_CRUISE_BP, J_CRUISE_VALS)
  target_accel = float(np.clip(target_accel, min(a_cruise_prev, a_target_prev) - j_cruise * dt, a_cruise_prev + j_cruise * dt))

  return target_accel


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    self.fcw = False
    self.dt = dt
    self.curve_speed = CurveSpeed(dt)
    self.e2e_assist = E2ESlowingAssist(dt)
    self.stop_target = E2EStopTarget()
    self.e2e_assist_active = False
    self.slc_e2e_fallback = False

    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.a_cruise = init_a
    self.output_a_target = init_a
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  def update(self, sm):
    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    if sm['controlsState'].forceDecel:
      v_cruise = 0.0

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET
    reset_state = reset_state or not v_cruise_initialized

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.output_a_target = np.clip(sm['carState'].aEgo, ACCEL_MIN, ACCEL_MAX)
      self.a_cruise = self.output_a_target
      self.curve_speed.reset()

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    now = sm.logMonoTime['modelV2'] * 1e-9 if REPLAY else time.monotonic()
    self.slc_e2e_fallback = map_cruise_supported(self.CP) and slc_e2e_fallback(sm, now, replay=REPLAY)
    curve_offset = self.curve_speed.update(sm['modelV2'], sm['vehicleParameters'].roll)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration + curve_offset
    output_should_stop_e2e = sm['modelV2'].action.shouldStop
    overlay_eligible = (self.CP.openpilotLongitudinalControl and not reset_state and sm['selfdriveState'].enabled and
                        sm['carControl'].longActive and not (sm['carState'].gasPressed or sm['carState'].brakePressed) and
                        np.isfinite(output_a_target_e2e) and sm.all_checks())
    stop_eligible = overlay_eligible and sm['carState'].gearShifter in (car.CarState.GearShifter.drive,
                                                                      car.CarState.GearShifter.low,
                                                                      car.CarState.GearShifter.sport)
    if stop_eligible:
      self.stop_target.update(eligible=stop_eligible, model=sm['modelV2'], now=now,
                              model_time=sm.logMonoTime['modelV2'] * 1e-9, v_ego=v_ego,
                              e2e_accel=output_a_target_e2e, e2e_stop=output_should_stop_e2e,
                              lead_stop_distance=get_lead_stop_distance(sm['radarState']),
                              lead_distance=get_lead_distance(sm['radarState']))
    else:
      self.stop_target.reset()

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    if self.CP.openpilotLongitudinalControl:
      self.mpc.set_cur_state(self.v_desired_filter.x, self.output_a_target)
    else:
      self.mpc.set_cur_state(v_ego, sm['carState'].aEgo)
    self.mpc.update(sm['radarState'], personality=sm['selfdriveState'].personality,
                    stop_distance=self.stop_target.distance if self.stop_target.active else None)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Save starting point for next iteration
    a_prev = self.output_a_target

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                              action_t=action_t)
    output_should_stop_mpc = should_stop(v_ego, output_a_target_mpc)

    self.a_cruise = get_cruise_accel(v_cruise, v_ego, self.a_cruise, self.output_a_target, self.dt)
    cruise_should_stop = should_stop(v_ego, self.a_cruise)

    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),
                  (self.a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop)]
    # Chill retains native real-lead MPC and cruise. The model action is enabled
    # by no-limit fallback or qualified stopping intent; stock-ACC advisory
    # planning retains its upstream candidate selection.
    full_e2e = self.slc_e2e_fallback or not self.CP.openpilotLongitudinalControl
    if full_e2e or self.stop_target.requested:
      candidates.append((output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e))
    output_a_target, self.mpc.source, _ = min(candidates, key=lambda c: c[0])
    self.output_should_stop = any(stop for _, _, stop in candidates) or self.stop_target.holding

    output_a_target, assist_stop, self.e2e_assist_active = self.e2e_assist.update(
      eligible=overlay_eligible and not full_e2e, lead=sm['radarState'].leadOne,
      model_lead_speed=get_model_lead_speed(sm['modelV2'], v_ego), v_ego=v_ego,
      regular_accel=output_a_target, e2e_accel=output_a_target_e2e, e2e_stop=output_should_stop_e2e)
    if self.e2e_assist_active:
      self.mpc.source = LongitudinalPlanSource.e2e
    self.output_should_stop |= assist_stop
    self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)

    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.output_a_target + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks()

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime - sm.logMonoTime['modelV2']) / 1e9
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.present
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True

    longitudinalPlan.e2eAssistActive = bool(self.e2e_assist_active)
    longitudinalPlan.slcE2eFallback = self.slc_e2e_fallback
    longitudinalPlan.e2eStopActive = self.stop_target.active
    longitudinalPlan.e2eStopDistance = self.stop_target.distance
    target = longitudinalPlan.stopTarget
    target.active = self.stop_target.active
    target.distance = self.stop_target.distance
    target.holding = self.stop_target.holding
    target.approaching = self.stop_target.approaching
    target.approachDistance = self.stop_target.approach_distance
    target.horizonCandidate = self.stop_target.horizon.observation.distance >= 0.
    target.horizonDistance = self.stop_target.horizon.observation.distance
    target.horizonRemaining = self.stop_target.horizon.observation.remaining_time

    pm.send('longitudinalPlan', plan_send)
