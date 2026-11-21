import time
import unittest
from unittest.mock import patch

import numpy as np
from opendbc.car.structs import car
from openpilot.cereal import log, messaging
from openpilot.selfdrive.car.cruise import VCruiseHelper
from openpilot.selfdrive.car.map_cruise import fallback_e2e_ready
from openpilot.selfdrive.car.slc_settings import SLCSettings
from openpilot.selfdrive.controls.lib.longitudinal_mode import slc_e2e_fallback, full_e2e_requested, e2e_map_speed_cap
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner, get_cruise_accel
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_T_FOLLOW
from openpilot.selfdrive.controls.tests.test_stop_horizon import endpoint_model
from openpilot.selfdrive.modeld.constants import ModelConstants


class Inputs(dict):
  def __init__(self, now):
    services = ['carState', 'carControl', 'controlsState', 'selfdriveState', 'vehicleParameters',
                'radarState', 'modelV2', 'mapCruiseState']
    super().__init__({s: getattr(messaging.new_message(s), s) for s in services})
    self.services = services
    self.valid = dict.fromkeys(services, True)
    self.recv_time = dict.fromkeys(services, now)
    self.logMonoTime = dict.fromkeys(services, int(now * 1e9))

  def all_checks(self):
    return all(v for s, v in self.valid.items() if s != 'mapCruiseState')


class PlanSink:
  def send(self, service, event):
    # Exercise the actual current cereal schema, not a test-only plan object.
    self.plan = messaging.log_from_bytes(event.to_bytes()).longitudinalPlan


class TestLongitudinalModes(unittest.TestCase):
  def setUp(self):
    self.now = time.monotonic()
    self.cp = car.CarParams(openpilotLongitudinalControl=True, longitudinalActuatorDelay=.5)
    self.sm = Inputs(self.now)
    self.sm['carState'].vEgo = 20.
    self.sm['carState'].vCruise = 90.
    self.sm['carState'].gearShifter = 'drive'
    self.sm['carState'].cruiseState.available = True
    self.sm['carControl'].longActive = True
    self.sm['controlsState'].longControlState = 'pid'
    self.sm['selfdriveState'].enabled = True
    self.sm['selfdriveState'].personality = 'standard'
    self.sm['mapCruiseState'].state = 'active'
    self.sm['mapCruiseState'].automaticE2e = True
    self.set_model_speed(20.)
    self.sm['modelV2'].action.desiredAcceleration = -.5
    self.planner = LongitudinalPlanner(self.cp, init_v=20.)
    self.sink = PlanSink()

  def set_model_speed(self, speed):
    m = self.sm['modelV2']
    times = ModelConstants.T_IDXS
    m.position.t = times
    m.position.x = [speed * t for t in times]
    m.position.y = m.velocity.y = m.orientationRate.z = [0.] * len(times)
    m.velocity.x = [speed] * len(times)

  def tick(self, *, refresh_map=True):
    self.now += .05
    for s in self.sm.services:
      if refresh_map or s != 'mapCruiseState':
        self.sm.recv_time[s] = self.now
        # Publish just before receipt so float conversion cannot put a packet in the future.
        self.sm.logMonoTime[s] = int(self.now * 1e9) - 100
    with patch('openpilot.selfdrive.controls.lib.longitudinal_planner.time.monotonic', return_value=self.now), \
         patch('openpilot.selfdrive.controls.lib.longitudinal_planner.cloudlog.info'):
      self.planner.update(self.sm)
      self.planner.publish(self.sm, self.sink)
    return self.sink.plan

  def test_chill_and_fresh_no_limit_fallback_use_native_candidates(self):
    plan = self.tick()
    self.assertFalse(plan.slcE2eFallback)
    self.assertGreater(plan.aTarget, -.5)
    self.assertFalse(plan.e2eStopActive)
    self.sm['mapCruiseState'].e2eFallback = True
    plan = self.tick()
    self.assertTrue(plan.slcE2eFallback)
    self.assertAlmostEqual(plan.aTarget, -.5)
    self.assertEqual(str(plan.longitudinalPlanSource), 'e2e')
    self.sm['mapCruiseState'].e2eFallback = False
    self.assertFalse(self.tick().slcE2eFallback)

  def test_manual_e2e_uses_model_with_posted_limit_and_off_returns_to_chill(self):
    self.sm['mapCruiseState'].targetSpeed = 90. / 3.6
    self.sm['selfdriveState'].experimentalMode = True
    plan = self.tick()
    self.assertFalse(plan.slcE2eFallback)
    self.assertAlmostEqual(plan.aTarget, -.5)
    self.assertEqual(str(plan.longitudinalPlanSource), 'e2e')
    self.sm['selfdriveState'].experimentalMode = False
    for _ in range(20):
      plan = self.tick()
    self.assertGreater(plan.aTarget, -.5)
    self.sm['mapCruiseState'].e2eFallback = True
    self.assertAlmostEqual(self.tick().aTarget, -.5)  # automatic assist survives OFF

  def test_manual_e2e_map_maximum_never_raises_lower_set(self):
    self.sm['selfdriveState'].experimentalMode = True
    self.sm['mapCruiseState'].targetSpeed = 65. / 3.6  # 60 posted plus configured offset
    for selected, expected in ((105., 65.), (50., 50.)):
      with self.subTest(selected=selected):
        self.sm['carState'].vCruise = selected
        with patch('openpilot.selfdrive.controls.lib.longitudinal_planner.get_cruise_accel', wraps=get_cruise_accel) as calculate:
          self.tick()
        self.assertAlmostEqual(calculate.call_args.args[0], expected / 3.6, places=5)

  def test_manual_e2e_map_gap_uses_existing_set_not_configured_ceiling(self):
    self.sm['selfdriveState'].experimentalMode = True
    self.sm['carState'].vCruise = 100.
    self.sm['mapCruiseState'].state = 'waiting'
    self.sm['mapCruiseState'].targetSpeed = 0.
    with patch('openpilot.selfdrive.controls.lib.longitudinal_planner.get_cruise_accel', wraps=get_cruise_accel) as calculate:
      self.tick()
    self.assertAlmostEqual(calculate.call_args.args[0], 100. / 3.6)

  def test_stale_or_invalid_manual_mode_cannot_enable_e2e(self):
    self.sm['selfdriveState'].experimentalMode = True
    self.assertTrue(full_e2e_requested(self.sm, self.now))
    self.assertFalse(full_e2e_requested(self.sm, self.now + .31))
    self.sm.valid['selfdriveState'] = False
    self.assertFalse(full_e2e_requested(self.sm, self.now))

  def test_e2e_map_cap_rejects_stale_and_invalid_values(self):
    for speed in (8., 60., 145.):
      self.sm['mapCruiseState'].targetSpeed = speed / 3.6
      self.assertAlmostEqual(e2e_map_speed_cap(self.sm, self.now), speed / 3.6, places=5)
    self.assertIsNone(e2e_map_speed_cap(self.sm, self.now + .81))
    for speed in (0., -1., float('nan'), float('inf'), 300.):
      self.sm['mapCruiseState'].targetSpeed = speed / 3.6
      self.assertIsNone(e2e_map_speed_cap(self.sm, self.now))

  def test_aggressive_model_candidate_scales_in_full_e2e(self):
    for personality, speed, model_accel, expected in (
      ('aggressive', 5., -.5, -.5), ('aggressive', 7.5, -.75, -.725),
      ('aggressive', 10., -.5, -.4), ('aggressive', 20., -1., -1.),
      ('standard', 20., -.5, -.5), ('relaxed', 20., -.5, -.5),
    ):
      with self.subTest(personality=personality, speed=speed, model_accel=model_accel):
        self.sm['selfdriveState'].personality = personality
        self.sm['mapCruiseState'].e2eFallback = True
        self.sm['carState'].vEgo = speed
        self.set_model_speed(speed)
        self.sm['modelV2'].action.desiredAcceleration = model_accel
        self.planner = LongitudinalPlanner(self.cp, init_v=speed)
        plan = self.tick()
        self.assertEqual(str(plan.longitudinalPlanSource), 'e2e')
        self.assertAlmostEqual(plan.aTarget, expected, places=5)

  def test_aggressive_keeps_chill_stop_and_slowing_inputs(self):
    self.sm['selfdriveState'].personality = 'aggressive'
    self.sm['modelV2'].action.desiredAcceleration = -.5
    self.assertGreater(self.tick().aTarget, -.5)  # ordinary cruise
    self.assertEqual(str(self.sink.plan.longitudinalPlanSource), 'cruise')
    predicted = endpoint_model()
    self.sm['carState'].vEgo = 8.
    self.planner = LongitudinalPlanner(self.cp, init_v=8.)
    model = self.sm['modelV2']
    model.position.x, model.position.y = predicted.position.x, predicted.position.y
    model.velocity.x, model.velocity.y = predicted.velocity.x, predicted.velocity.y
    with patch.object(self.planner.stop_target, 'update', wraps=self.planner.stop_target.update) as stop, \
         patch.object(self.planner.e2e_assist, 'update', wraps=self.planner.e2e_assist.update) as assist:
      plan = self.tick()
    self.assertTrue(plan.e2eStopActive)
    self.assertFalse(plan.slcE2eFallback)
    self.assertLessEqual(plan.aTarget, -.5)
    self.assertEqual(stop.call_args.kwargs['e2e_accel'], -.5)
    self.assertEqual(assist.call_args.kwargs['e2e_accel'], -.5)

  def test_chill_uses_second_lead_forecast_and_falls_back_when_incomplete(self):
    lead = self.sm['radarState'].leadTwo
    lead.present, lead.modelProb, lead.dRel, lead.vLead = True, .99, 30., 10.
    self.assertTrue(self.tick().hasLead)
    radar_only_obstacle = self.planner.mpc.params[-1, 2]

    # The model predicts 10 m of travel and a 10 m/s drop from the tracked state.
    forecast = self.sm['modelV2'].init('leadsV3', 2)[1]
    forecast.x = [40., 50., 50., 50., 50., 50.]
    forecast.v = [11., 1., 1., 1., 1., 1.]
    self.assertFalse(self.tick().slcE2eFallback)
    self.assertAlmostEqual(self.planner.mpc.params[-1, 2], 40.)
    self.assertGreater(radar_only_obstacle, 40.)

    for velocities in ([11.], [11., float('nan'), 1., 1., 1., 1.]):
      forecast.v = velocities
      self.tick()
      self.assertAlmostEqual(self.planner.mpc.params[-1, 2], radar_only_obstacle)

  def test_stale_invalid_or_disabled_mode_request_cannot_acknowledge_a_ceiling_raise(self):
    state = self.sm['mapCruiseState']
    state.e2eFallback = True
    self.assertTrue(self.tick().slcE2eFallback)
    self.now += .9
    self.assertFalse(self.tick(refresh_map=False).slcE2eFallback)
    self.sm.valid['mapCruiseState'] = False
    self.assertFalse(self.tick().slcE2eFallback)
    self.sm.valid['mapCruiseState'] = True
    state.state = 'off'
    self.assertFalse(self.tick().slcE2eFallback)
    state.state = 'waiting'
    state.automaticE2e = False
    self.assertFalse(self.tick().slcE2eFallback)

  def test_model_only_slow_lead_gate_at_high_ego_speed_and_original_brake_debounce(self):
    self.sm['modelV2'].action.desiredAcceleration = -2.
    lead = self.sm['radarState'].leadOne
    lead.present, lead.dRel, lead.vLead, lead.vRel = True, 80., 10., -10.
    lead.aLeadTau, lead.modelProb = 1.5, .99
    model_lead = self.sm['modelV2'].init('leadsV3', 1)[0]
    model_lead.prob, model_lead.v = .99, [60. / 3.6]
    for _ in range(6):
      self.assertFalse(self.tick().e2eAssistActive)
    model_lead.v = [54.9 / 3.6]
    self.assertFalse(self.tick().e2eAssistActive)
    self.assertFalse(self.tick().e2eAssistActive)
    self.assertTrue(self.tick().e2eAssistActive)
    self.sm['carState'].gasPressed = True
    self.assertFalse(self.tick().e2eAssistActive)

  def test_virtual_stop_enters_at_qualified_horizon_with_physical_lead_clearance_and_releases(self):
    self.sm['carState'].vEgo = 8.
    self.planner = LongitudinalPlanner(self.cp, init_v=8.)
    predicted = endpoint_model(residual_speed=.7)
    m = self.sm['modelV2']
    m.position.x, m.position.y = predicted.position.x, predicted.position.y
    m.velocity.x, m.velocity.y = predicted.velocity.x, predicted.velocity.y
    m.action.desiredAcceleration = -1.
    lead = self.sm['radarState'].leadOne
    lead.present, lead.dRel, lead.vLead, lead.vRel = True, 15., 10., 2.
    lead.aLeadTau, lead.modelProb = 1.5, .99
    plan = self.tick()
    self.assertFalse(plan.e2eStopActive)
    self.assertTrue(plan.stopTarget.approaching)
    lead.dRel = 30.  # a lead running the light is beyond the distinct model stop
    plan = self.tick()
    self.assertTrue(plan.e2eStopActive)
    self.assertAlmostEqual(plan.e2eStopDistance, 16., places=4)
    self.assertFalse(plan.shouldStop)  # approaching, not an immediate standstill command
    np.testing.assert_allclose(self.planner.mpc.params[:, 6], 22.)
    self.set_model_speed(8.)
    m.action.desiredAcceleration = .5
    plan = self.tick()
    self.assertFalse(plan.e2eStopActive)
    self.assertFalse(plan.stopTarget.holding)
    np.testing.assert_allclose(self.planner.mpc.params[:, 6], 1e4)

  def test_driver_override_or_invalid_model_discards_virtual_obstacle(self):
    predicted = endpoint_model()
    m = self.sm['modelV2']
    m.position.x, m.position.y = predicted.position.x, predicted.position.y
    m.velocity.x, m.velocity.y = predicted.velocity.x, predicted.velocity.y
    self.assertTrue(self.tick().e2eStopActive)
    self.sm['carState'].brakePressed = True
    self.assertFalse(self.tick().e2eStopActive)
    self.sm['carState'].brakePressed = False
    self.assertTrue(self.tick().e2eStopActive)
    self.sm.valid['modelV2'] = False
    self.assertFalse(self.tick().e2eStopActive)

  def test_native_personalities_and_stronger_real_lead_are_retained_in_fallback(self):
    self.assertEqual([get_T_FOLLOW(p) for p in (log.LongitudinalPersonality.relaxed, log.LongitudinalPersonality.standard,
                                                        log.LongitudinalPersonality.aggressive)], [1.75, 1.45, 1.25])
    self.sm['mapCruiseState'].e2eFallback = True
    self.sm['modelV2'].action.desiredAcceleration = .3
    lead = self.sm['radarState'].leadOne
    lead.present, lead.dRel, lead.vLead, lead.vRel = True, 20., 2., -18.
    lead.aLeadTau, lead.modelProb = 1.5, .99
    for _ in range(8):
      plan = self.tick()
    self.assertLess(plan.aTarget, -.5)
    self.assertEqual(str(plan.longitudinalPlanSource), 'lead0')
    self.assertFalse(plan.e2eAssistActive)

  def test_no_limit_initial_set_and_later_missing_map_ack_recovery(self):
    cruise = VCruiseHelper(self.cp)
    cs = self.sm['carState']
    settings = SLCSettings(e2e_ceiling_kph=100)
    cruise.update_v_cruise(cs, False, True, map_enabled=True, automatic_e2e=True,
                           now=self.now, slc_settings=settings)
    cruise.initialize_v_cruise(cs, 40)
    self.assertEqual(cruise.v_cruise_kph, 100)

    def update_cruise(limit, ready=False):
      cruise.update_v_cruise(cs, True, True, map_enabled=True, automatic_e2e=True,
                            map_speed=limit, map_gps_time=self.now, now=self.now,
                            e2e_ready=ready, slc_settings=settings)
      cs.vCruise = cruise.v_cruise_kph
      selector, status = cruise.map_cruise, self.sm['mapCruiseState']
      status.state = selector.state(True)
      status.automaticE2e, status.e2eFallback = selector.automatic_e2e, selector.e2e_fallback

    for _ in range(44):
      update_cruise(60. / 3.6)
      self.tick()
    self.assertEqual(cruise.v_cruise_kph, 60.)
    self.assertFalse(self.tick().slcE2eFallback)
    for _ in range(24):
      update_cruise(None)
      self.tick()
    self.assertTrue(self.sink.plan.slcE2eFallback)
    self.assertEqual(cruise.v_cruise_kph, 60.)  # no ACK delivered to card yet
    ack = type('Ack', (dict,), {})({'longitudinalPlan': self.sink.plan})
    ack.services = ['longitudinalPlan']
    ack.valid = {'longitudinalPlan': True}
    ack.logMonoTime = {'longitudinalPlan': int(self.now * 1e9)}
    ack.recv_time = {'longitudinalPlan': self.now}
    update_cruise(None, fallback_e2e_ready(ack, self.now))
    self.assertEqual(cruise.v_cruise_kph, 60.)  # E2E acknowledgement does not raise an existing SET
    for _ in range(44):
      update_cruise(80. / 3.6)
      self.tick()
    self.assertEqual(cruise.v_cruise_kph, 80.)
    self.assertFalse(self.sink.plan.slcE2eFallback)

  def test_absent_mode_service_retains_chill(self):
    self.sm.services.remove('mapCruiseState')
    self.assertFalse(slc_e2e_fallback(self.sm, self.now))
    self.assertFalse(self.tick().slcE2eFallback)


if __name__ == '__main__':
  unittest.main()
