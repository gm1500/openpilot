import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from opendbc.car.gm.values import CAR
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner, T_IDXS_MPC


class FakeSubMaster(dict):
  valid = True

  def all_checks(self, services=None):
    return self.valid


class TestGMLeadStop(unittest.TestCase):
  def setUp(self):
    self.cp = SimpleNamespace(openpilotLongitudinalControl=True, carFingerprint=CAR.CHEVROLET_SILVERADO,
                              longitudinalActuatorDelay=0.5, steerRatio=16.3, wheelbase=3.745, stopAccel=-0.37,
                              longitudinalTuning=SimpleNamespace(kiBP=[0.0], kiV=[0.05]))
    def lead():
      return SimpleNamespace(present=True, modelProb=1.0, dRel=4.0, vLead=0.0)

    self.sm = FakeSubMaster(
      carState=SimpleNamespace(vEgo=0.5, aEgo=-0.15, vCruise=50.0, steeringAngleDeg=0.0,
                               gasPressed=False, brakePressed=False, standstill=False,
                               cruiseState=SimpleNamespace(standstill=False)),
      carControl=SimpleNamespace(orientationNED=[]),
      controlsState=SimpleNamespace(longControlState=LongCtrlState.pid, forceDecel=False),
      selfdriveState=SimpleNamespace(enabled=True, experimentalMode=False, personality=0),
      vehicleParameters=SimpleNamespace(angleOffsetDeg=0.0),
      radarState=SimpleNamespace(leadOne=lead(), leadTwo=lead()),
      modelV2=SimpleNamespace(meta=SimpleNamespace(disengagePredictions=SimpleNamespace(gasPressProbs=[1.0, 1.0])),
                             action=SimpleNamespace(desiredAcceleration=1.0, shouldStop=False)))
    self.sm['radarState'].leadTwo.present = False
    self.mpc = Mock(crash_cnt=0, source='lead0')
    self.set_trajectory(-0.3)

  def set_trajectory(self, accel):
    self.mpc.v_solution = np.maximum(self.sm['carState'].vEgo + accel * T_IDXS_MPC, 0.0)
    self.mpc.a_solution = np.full(len(T_IDXS_MPC), accel)
    self.mpc.j_solution = np.zeros(len(T_IDXS_MPC) - 1)

  def planner(self, init_v=0.2):
    with patch('openpilot.selfdrive.controls.lib.longitudinal_planner.LongitudinalMpc', return_value=self.mpc):
      return LongitudinalPlanner(self.cp, init_v=init_v)

  def test_measured_creep_and_early_hold(self):
    planner = self.planner()
    planner.update(self.sm)
    self.assertEqual(self.mpc.set_cur_state.call_args.args[0], self.sm['carState'].vEgo)
    self.assertTrue(planner.output_should_stop)

  def test_does_not_reduce_higher_internal_speed(self):
    planner = self.planner(init_v=0.7)
    planner.update(self.sm)
    self.assertGreater(self.mpc.set_cur_state.call_args.args[0], self.sm['carState'].vEgo)

  def test_moving_trajectory_does_not_enter_hold(self):
    self.set_trajectory(-0.12)  # still moving at the end of the 2.5 s control trajectory
    planner = self.planner()
    planner.update(self.sm)
    self.assertFalse(planner.output_should_stop)

  def test_no_hold_for_acceleration(self):
    self.set_trajectory(0.3)
    planner = self.planner()
    planner.update(self.sm)
    self.assertFalse(planner.output_should_stop)

  def test_lead_validation(self):
    invalid = [('present', False), ('modelProb', 0.89), ('modelProb', float('nan')),
               ('dRel', 0.0), ('dRel', 10.0), ('dRel', float('nan')),
               ('vLead', 0.51), ('vLead', -1.01), ('vLead', float('nan'))]
    for field, value in invalid:
      with self.subTest(field=field, value=value):
        lead = self.sm['radarState'].leadOne
        original = getattr(lead, field)
        setattr(lead, field, value)
        planner = self.planner()
        planner.update(self.sm)
        self.assertFalse(planner.output_should_stop)
        self.assertLess(self.mpc.set_cur_state.call_args.args[0], self.sm['carState'].vEgo)
        setattr(lead, field, original)

  def test_second_lead_can_trigger_stop(self):
    self.sm['radarState'].leadOne.present = False
    self.sm['radarState'].leadTwo.present = True
    planner = self.planner()
    planner.update(self.sm)
    self.assertTrue(planner.output_should_stop)

  def test_stock_and_other_vehicle_keep_baseline(self):
    for stock, fingerprint in [(True, CAR.CHEVROLET_SILVERADO), (False, CAR.CHEVROLET_EQUINOX)]:
      with self.subTest(stock=stock, fingerprint=fingerprint):
        self.cp.openpilotLongitudinalControl = not stock
        self.cp.carFingerprint = fingerprint
        planner = self.planner()
        planner.update(self.sm)
        self.assertFalse(planner.output_should_stop)
        self.assertLess(self.mpc.set_cur_state.call_args.args[0], self.sm['carState'].vEgo)

  def test_driver_override_disables_assist(self):
    for pedal in ('gasPressed', 'brakePressed'):
      with self.subTest(pedal=pedal):
        setattr(self.sm['carState'], pedal, True)
        planner = self.planner()
        planner.update(self.sm)
        self.assertFalse(planner.output_should_stop)
        self.assertLess(self.mpc.set_cur_state.call_args.args[0], self.sm['carState'].vEgo)
        setattr(self.sm['carState'], pedal, False)

  def test_invalid_inputs_disable_assist(self):
    self.sm.valid = False
    planner = self.planner()
    planner.update(self.sm)
    self.assertFalse(planner.output_should_stop)
    self.assertLess(self.mpc.set_cur_state.call_args.args[0], self.sm['carState'].vEgo)

  def test_disengaged_or_unset_cruise_disables_assist(self):
    for disengaged in (True, False):
      with self.subTest(disengaged=disengaged):
        self.sm['controlsState'].longControlState = LongCtrlState.off if disengaged else LongCtrlState.pid
        self.sm['carState'].vCruise = 50.0 if disengaged else 255.0
        planner = self.planner()
        planner.update(self.sm)
        self.assertFalse(planner.output_should_stop)

  def test_higher_speeds_keep_baseline(self):
    self.sm['carState'].vEgo = 3.0
    self.set_trajectory(-1.5)
    planner = self.planner()
    planner.update(self.sm)
    self.assertFalse(planner.output_should_stop)
    self.assertLess(self.mpc.set_cur_state.call_args.args[0], self.sm['carState'].vEgo)

  def test_hold_hysteresis_survives_lead_jitter(self):
    self.sm['controlsState'].longControlState = LongCtrlState.stopping
    self.sm['radarState'].leadOne.present = False
    for speed, expected in [(0.59, True), (0.61, True), (0.79, True), (0.81, False)]:
      with self.subTest(speed=speed):
        self.sm['carState'].vEgo = speed
        self.set_trajectory(-0.3)
        planner = self.planner()
        planner.update(self.sm)
        self.assertEqual(planner.output_should_stop, expected)

  def test_brake_retained_then_released_for_departure_and_disengagement(self):
    planner = self.planner()
    control = LongControl(self.cp)
    control.long_control_state = LongCtrlState.pid
    control.last_output_accel = -0.55
    planner.update(self.sm)
    accel = control.update(True, self.sm['carState'], planner.output_a_target, planner.output_should_stop, (-4.0, 2.0))
    self.assertEqual(control.long_control_state, LongCtrlState.stopping)
    self.assertEqual(accel, -0.55)
    self.sm['controlsState'].longControlState = control.long_control_state
    self.sm['radarState'].leadOne.vLead = 1.0
    self.set_trajectory(0.3)
    planner.update(self.sm)
    accel = control.update(True, self.sm['carState'], planner.output_a_target, planner.output_should_stop, (-4.0, 2.0))
    self.assertEqual(control.long_control_state, LongCtrlState.pid)
    self.assertGreater(accel, 0.0)
    self.assertEqual(control.update(False, self.sm['carState'], -0.5, True, (-4.0, 2.0)), 0.0)
    self.assertEqual(control.long_control_state, LongCtrlState.off)


if __name__ == '__main__':
  unittest.main()
