import math
import unittest
from types import SimpleNamespace as NS

from openpilot.selfdrive.controls.lib.conditional_experimental import (
  ConditionalExperimental, MapApproach, activation_distance, map_approach, model_intent, slc_fallback_request,
)
from openpilot.selfdrive.controls.lib.longitudinal_mode import LongitudinalMode, selected_mode, set_longitudinal_mode, automatic_e2e_selected
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.ui.onroad.conditional_icon import conditional_ring_state, no_speed_e2e_active


def model(speed=15., brake=1.5):
  times = ModelConstants.T_IDXS
  vs = [max(0., speed - brake * t) for t in times]
  ts = [min(t, speed / brake) if brake > 0 else t for t in times]
  xs = [speed * t - .5 * brake * t**2 for t in ts]
  return NS(velocity=NS(x=vs, y=[0.] * len(vs)), position=NS(x=xs, y=[0.] * len(xs)))


class FakeSM(dict):
  def __init__(self, **messages):
    super().__init__(messages)
    self.services = list(messages)
    self.valid = dict.fromkeys(messages, True)
    self.recv_frame = dict.fromkeys(messages, 10)
    self.recv_time = dict.fromkeys(messages, 100.)
    self.logMonoTime = dict.fromkeys(messages, int(100e9))


class TestConditionalExperimental(unittest.TestCase):
  def setUp(self):
    self.policy = ConditionalExperimental(.05, .5)
    self.now = 101.

  def step(self, **kwargs):
    self.now += .05
    args = {'enabled': True, 'eligible': True, 'approach': MapApproach(True, 10, 80., 101.), 'now': self.now,
            'model': model(), 'v_ego': 15., 'v_set': 15., 'personality': 1,
            'regular_accel': .5, 'regular_stop': False, 'e2e_accel': -1., 'e2e_stop': False}
    args.update(kwargs)
    return self.policy.update(**args)

  def arm(self, **kwargs):
    self.step(approach=MapApproach(True, 10, 80., 100.), e2e_accel=0.)
    return self.step(**kwargs)

  def test_direct_entry_without_model_slowing_or_brake_delay(self):
    self.assertEqual(self.arm(e2e_accel=.2, model=model(brake=0)), (.2, False, True))
    self.assertTrue(self.policy.active)
    self.assertEqual(self.policy.state, 'active')
    self.assertFalse(self.policy.intent.slowing)
    self.assertEqual(self.step(), (-1., False, True))

  def test_enabled_is_independent_of_contribution(self):
    self.assertEqual(self.arm(regular_accel=-3), (-3, False, False))
    self.assertTrue(self.policy.active)
    self.assertEqual(self.policy.state, 'active')
    self.assertEqual(self.step(regular_accel=-3, e2e_stop=True), (-3, True, True))
    self.assertEqual(self.step(regular_accel=-3, regular_stop=True, e2e_stop=True), (-3, True, False))

  def test_two_advancing_map_observations_required_for_initial_entry(self):
    for _ in range(100):
      self.assertEqual(self.step(), (.5, False, False))
    self.assertFalse(self.policy.armed)
    self.step(approach=MapApproach(True, 10, 80., 102.))
    self.assertTrue(self.policy.active)

  def test_nearby_junction_handoff_keeps_e2e_while_new_target_confirms(self):
    self.arm()
    self.now = 102.
    for _ in range(20):
      self.step(approach=MapApproach(True, 20, 60., 102.))
      self.assertTrue(self.policy.active)
      self.assertEqual(self.policy.target_id, 10)
      self.assertEqual(self.policy.reason, 'junctionHandoff')
    self.step(approach=MapApproach(True, 20, 45., 103.))
    self.assertTrue(self.policy.active)
    self.assertEqual(self.policy.target_id, 20)
    self.assertEqual(self.policy.gap_time, 0.)

  def test_handoff_after_one_second_match_dropout_stays_continuous(self):
    self.arm()
    for _ in range(20):
      self.step(approach=MapApproach(can_hold=True))
      self.assertTrue(self.policy.active)
    for _ in range(20):
      self.step(approach=MapApproach(True, 20, 60., 102.))
      self.assertTrue(self.policy.active)
    self.step(approach=MapApproach(True, 20, 45., 103.))
    self.assertTrue(self.policy.active)
    self.assertEqual(self.policy.target_id, 20)

  def test_matching_interruption_cannot_arm_or_create_stop(self):
    for _ in range(100):
      self.assertEqual(self.step(approach=MapApproach(can_hold=True), e2e_stop=True), (.5, False, False))
      self.assertFalse(self.policy.armed)

  def test_hold_expires_and_cannot_be_extended_by_alternating_candidates(self):
    for changing in (False, True):
      self.setUp()
      self.arm()
      for i in range(50):
        approach = MapApproach(True, 20+i%2, 60., self.now+.05) if changing else MapApproach(can_hold=True)
        self.step(approach=approach, v_ego=1.)
        if i < 35:
          self.assertTrue(self.policy.armed)
      self.assertFalse(self.policy.armed)
      self.assertFalse(self.policy.active)

  def test_hold_distance_and_real_gps_age_are_independent_limits(self):
    self.arm()
    for _ in range(25):
      self.step(approach=MapApproach(can_hold=True), v_ego=35.)
    self.assertFalse(self.policy.armed)  # more than 40 m, less than 2.25 s
    self.setUp()
    self.arm()
    self.step(approach=MapApproach(can_hold=True), now=104.1)
    self.assertFalse(self.policy.armed)

  def test_same_target_recovery_does_not_restart_confirmation(self):
    self.arm()
    for _ in range(20):
      self.step(approach=MapApproach(can_hold=True))
    self.step(approach=MapApproach(True, 10, 50., 102.))
    self.assertTrue(self.policy.active)
    self.assertEqual(self.policy.reason, 'junction')

  def test_fresh_candidate_gps_counts_for_position_age_but_not_confirmation(self):
    self.arm()
    self.step(now=102.4)  # established target, real GPS still at 101
    self.step(approach=MapApproach(True, 20, 60., 102.5), now=102.6)
    self.step(approach=MapApproach(True, 20, 60., 102.5), now=104.3, v_ego=1.)
    self.assertTrue(self.policy.active)
    self.assertEqual(self.policy.target_id, 10)  # new target still unconfirmed
    self.step(approach=MapApproach(True, 20, 60., 102.5), now=104.7, v_ego=1.)
    self.assertFalse(self.policy.active)  # repeated observation cannot renew grace

  def test_hold_uses_elapsed_time_even_if_planner_updates_are_delayed(self):
    self.arm()
    self.step(approach=MapApproach(can_hold=True), now=103.4, v_ego=0.)
    self.assertFalse(self.policy.active)

  def test_definite_clear_road_far_target_and_stale_input_clear_without_grace(self):
    for approach in (MapApproach(), MapApproach(True), MapApproach(True, 20, 300., 102.)):
      self.setUp()
      self.arm()
      self.step(approach=approach, e2e_stop=True)
      self.assertFalse(self.policy.armed)
      self.assertFalse(self.policy.active)
      self.assertFalse(self.policy.stop_requested)

  def test_range_latches_as_speed_and_set_shrink(self):
    self.arm()
    self.step(v_ego=2., v_set=2., model=model(speed=2), approach=MapApproach(True, 10, 35., 102.))
    self.assertLess(self.policy.activation_distance, 35.)
    self.assertTrue(self.policy.active)

  def test_set_speed_keeps_range_during_slowing_and_standstill(self):
    for speed in (15., 8., 0.):
      self.policy.reset()
      for timestamp in (100., 101.):
        self.step(v_ego=speed, v_set=60 / 3.6, personality=0, model=model(speed=speed, brake=0),
                  approach=MapApproach(True, 10, 100., timestamp), e2e_accel=.2)
      self.assertTrue(self.policy.active)
      self.assertAlmostEqual(self.policy.activation_distance, activation_distance(60 / 3.6, 0., 0, .5))

  def test_current_set_speed_and_profile_determine_entry_range(self):
    for set_speed, profile, expected in ((40., 0, False), (60., 0, True), (50., 0, False), (50., 2, True)):
      with self.subTest(set_speed=set_speed, profile=profile):
        self.policy.reset()
        for timestamp in (100., 101.):
          self.step(v_set=set_speed / 3.6, personality=profile, approach=MapApproach(True, 10, 100., timestamp))
        self.assertEqual(self.policy.armed, expected)
        self.assertEqual(self.policy.activation_speed, set_speed / 3.6)

  def test_unavailable_set_speed_clears_arming_and_assistance(self):
    for v_set in (None, 0., -1., math.nan, math.inf, 255.):
      self.arm()
      self.assertEqual(self.step(v_set=v_set), (.5, False, False))
      self.assertEqual(self.policy.reason, 'invalidSetSpeed')
      self.assertFalse(self.policy.armed)
      self.assertFalse(self.policy.active)

  def test_model_go_remains_enabled_and_slews_return_to_acceleration(self):
    previous, _, _ = self.arm()
    for _ in range(50):
      current, stop, _ = self.step(e2e_accel=.3, model=model(brake=0))
      self.assertLessEqual(current - previous, .05 + 1e-8)
      self.assertFalse(stop)
      self.assertTrue(self.policy.active)
      previous = current
    self.assertEqual(current, .3)

  def test_stop_holds_on_map_loss_only_while_model_still_requests_it_at_rest(self):
    self.arm(v_ego=0., model=model(speed=0), e2e_accel=0., e2e_stop=True)
    for _ in range(100):
      _, stop, active = self.step(v_ego=0., model=model(speed=0), e2e_accel=0., e2e_stop=True, approach=MapApproach())
      self.assertTrue(stop)
      self.assertTrue(active)
      self.assertTrue(self.policy.active)
    _, stop, _ = self.step(v_ego=0., model=model(speed=0), e2e_accel=.3, e2e_stop=False, approach=MapApproach())
    self.assertFalse(stop)
    self.assertFalse(self.policy.active)

  def test_map_loss_while_moving_releases_and_cannot_create_stop(self):
    previous, _, _ = self.arm()
    for _ in range(50):
      current, stop, _ = self.step(approach=MapApproach(), e2e_stop=True)
      self.assertFalse(stop)
      self.assertLessEqual(current - previous, .05 + 1e-8)
      previous = current
    self.assertEqual(current, .5)
    self.assertFalse(self.policy.active)

  def test_override_or_invalid_model_immediately_clear_assistance(self):
    bad = model()
    bad.velocity.x[3] = math.nan
    for args in ({'eligible': False}, {'model': bad}, {'e2e_accel': math.inf}):
      self.arm()
      self.assertEqual(self.step(**args), (.5, False, False))
      self.assertFalse(self.policy.active)
      self.assertTrue(self.policy.armed)
      self.assertEqual(self.policy.state, 'inRange')
      self.assertEqual(self.step(), (-1., False, True))

  def test_disabled_or_invalid_kinematics_clear_all_state(self):
    for args in ({'enabled': False}, {'v_ego': math.nan}, {'v_ego': -1.}, {'now': math.nan}):
      self.arm()
      self.assertEqual(self.step(**args), (.5, False, False))
      self.assertFalse(self.policy.active)
      self.assertFalse(self.policy.armed)

  def test_pass_distance_matches_map_lookbehind_and_jitter_cannot_rearm(self):
    self.arm()
    for distance in (-12.1, -19.9, -20.):
      self.step(approach=MapApproach(True, 10, distance, 102.))
      self.assertTrue(self.policy.active)
    self.step(approach=MapApproach(True, 10, -20.1, 103.))
    self.assertFalse(self.policy.armed)
    for i in range(50):
      self.step(approach=MapApproach(True, 10, 2., 104.+i))
      self.assertFalse(self.policy.armed)

  def test_cannot_enter_for_target_out_of_range_or_already_behind(self):
    for distance in (300., -1.):
      self.policy.reset()
      for timestamp in range(100, 120):
        self.assertEqual(self.step(approach=MapApproach(True, 10, distance, timestamp)), (.5, False, False))

  def test_older_observation_discards_qualification_without_grace(self):
    self.arm()
    self.step(approach=MapApproach(True, 10, 50., 100.5))
    self.assertFalse(self.policy.armed)
    self.assertFalse(self.policy.active)

  def test_no_speed_and_junction_are_independent_or_conditions(self):
    # No qualified junction: missing speed alone enables the model candidate.
    self.assertEqual(self.step(fallback=True, approach=MapApproach()), (-1., False, True))
    self.assertTrue(self.policy.active)
    self.assertEqual(self.policy.reason, 'noSpeedLimit')
    self.step(fallback=False, approach=MapApproach())
    self.assertFalse(self.policy.active)
    # Known map speed: a qualified junction independently enables E2E.
    self.arm()
    self.assertTrue(self.policy.active)
    self.assertEqual(self.policy.reason, 'junction')
    self.step(fallback=True)
    self.assertTrue(self.policy.active)
    self.step(fallback=False)
    self.assertTrue(self.policy.active)
    self.assertEqual(self.policy.reason, 'junction')
    self.step(approach=MapApproach(True))
    self.assertFalse(self.policy.active)

  def test_map_recovery_requalifies_junction_against_restored_set(self):
    for distance, expected in ((180., False), (40., True)):
      self.setUp()
      for timestamp in (100., 101.):
        self.step(fallback=True, v_set=105/3.6, approach=MapApproach(True,10,distance,timestamp))
      self.assertTrue(self.policy.armed)
      self.step(fallback=False, v_set=40/3.6, approach=MapApproach(True,10,distance,102.))
      self.assertEqual(self.policy.active, expected)

  def test_no_speed_only_stop_releases_when_map_returns_without_junction(self):
    self.step(fallback=True, approach=MapApproach(), v_ego=0., model=model(speed=0), e2e_stop=True, e2e_accel=0.)
    _, stop, _ = self.step(fallback=False, approach=MapApproach(True), v_ego=0., model=model(speed=0), e2e_stop=True, e2e_accel=0.)
    self.assertFalse(stop)
    self.assertFalse(self.policy.active)


class TestDistanceAndIntent(unittest.TestCase):
  def test_profile_speed_acceleration_order(self):
    for speed in (5., 15., 30.):
      distances = [activation_distance(speed, 0., p, .5) for p in range(3)]
      self.assertLess(distances[0], distances[1])
      self.assertLess(distances[1], distances[2])
      for profile in range(3):
        self.assertLess(activation_distance(speed, -1, profile, .5), activation_distance(speed, 0, profile, .5))
        self.assertLess(activation_distance(speed, 0, profile, .5), activation_distance(speed, 1, profile, .5))
        self.assertLess(activation_distance(speed, 0, profile, .5), activation_distance(speed + 5, 0, profile, .5))
    self.assertEqual(activation_distance(0, 0, 1, .5), 20.)
    self.assertEqual(activation_distance(-.03, -1.3, 1, .5), 20.)
    self.assertEqual(activation_distance(15, -5, 1, .5), activation_distance(15, -1.8, 1, .5))

  def test_formula_agrees_with_independent_time_step_simulation(self):
    for v in (5., 15., 30.):
      for a in (-1., 0., 1.):
        for p, (brake, jerk) in enumerate(((2.2, 1.2), (1.8, 1.), (1.4, .8))):
          speed, position, elapsed, accel = v, 0., 0., a
          dt = .0005
          while speed > 0:
            if elapsed >= 1.5:
              accel = max(-brake, accel - jerk * dt)
            next_speed = max(0., speed + accel * dt)
            position += (speed + next_speed) * .5 * dt
            elapsed += dt
            speed = next_speed
          self.assertAlmostEqual(activation_distance(v, a, p, .5), max(20., position + 12.), delta=.04)

  def test_short_path_alone_is_not_stopping_evidence(self):
    m = model(brake=0)
    m.position.x = [v * .1 for v in m.position.x]
    intent = model_intent(m, 15)
    self.assertTrue(intent.valid)
    self.assertFalse(intent.slowing)
    self.assertEqual(intent.stop_distance, -1.)

  def test_turn_is_not_a_stop(self):
    m = model(brake=0)
    m.velocity.x = [15 * math.cos(t / 10 * math.pi / 2) for t in ModelConstants.T_IDXS]
    m.velocity.y = [15 * math.sin(t / 10 * math.pi / 2) for t in ModelConstants.T_IDXS]
    intent = model_intent(m, 15)
    self.assertFalse(intent.slowing)
    self.assertEqual(intent.stop_distance, -1.)

  def test_sustained_stop_distance_and_single_point_noise(self):
    intent = model_intent(model(brake=2), 15)
    self.assertTrue(intent.slowing)
    self.assertAlmostEqual(intent.stop_distance, 56.25, delta=.5)
    m = model(brake=0)
    m.velocity.x[20] = 0
    self.assertEqual(model_intent(m, 15).stop_distance, -1.)
    self.assertFalse(model_intent(m, 15).slowing)
    m.velocity.x = m.velocity.x[:-1]
    self.assertFalse(model_intent(m, 15).valid)


class TestMapFreshnessAndHUD(unittest.TestCase):
  def test_only_topological_junction_targets_arm_new_policy(self):
    for kind in ('stopSign', 'trafficLight', 'junction'):
      msg = NS(kind=kind, nodeId=7, distance=50., gpsMonoTime=int(99e9), positionMonoTime=int(99e9), reason='target')
      self.assertEqual(map_approach(FakeSM(mapTrafficControl=msg), 100.).valid, kind == 'junction')

  def test_map_health_is_independent_of_speed_limit(self):
    msg = NS(kind='junction', nodeId=7, distance=50., gpsMonoTime=int(99e9), positionMonoTime=int(99e9), reason='target')
    sm = FakeSM(mapTrafficControl=msg, mapSpeedLimit=NS())
    sm.valid['mapSpeedLimit'] = False
    self.assertEqual(map_approach(sm, 100.), MapApproach(True, 7, 50., 99.))
    msg.positionMonoTime = int(100e9)
    self.assertEqual(map_approach(sm, 100.).observation_time, 99.)  # projection is not another GPS confirmation
    sm.valid['mapTrafficControl'] = False
    self.assertFalse(map_approach(sm, 100.).valid)
    sm.valid['mapTrafficControl'] = True
    self.assertFalse(map_approach(sm, 102.).valid)
    msg.gpsMonoTime = int(90e9)
    self.assertFalse(map_approach(sm, 100.).valid)
    self.assertFalse(map_approach(FakeSM(), 100.).valid)

  def test_fresh_matching_interruption_is_the_only_holdable_missing_target(self):
    for reason in ('noRoadMatch', 'roadAlignment', 'ambiguousRoad', 'ambiguousFork',
                   'noPosition', 'gpsStale', 'cacheUnavailable', 'noJunction'):
      msg = NS(kind='none', nodeId=0, distance=0., gpsMonoTime=0, positionMonoTime=0, reason=reason)
      sm = FakeSM(mapTrafficControl=msg)
      sm.valid['mapTrafficControl'] = False
      approach = map_approach(sm, 100.)
      self.assertFalse(approach.valid)
      self.assertEqual(approach.can_hold, reason in ('noRoadMatch', 'roadAlignment', 'ambiguousRoad', 'ambiguousFork'))
      self.assertFalse(map_approach(sm, 102.).can_hold)

  def test_only_enabled_conditional_e2e_shows_orange(self):
    policy = NS(armed=True, contributing=False, e2eEnabled=False, state='inRange')
    sm = FakeSM(longitudinalPlan=NS(conditionalExperimental=policy, e2eAssistActive=True),
                selfdriveState=NS(conditionalExperimental=True, experimentalMode=False, enabled=True),
                carControl=NS(longActive=True), carState=NS(gasPressed=False, brakePressed=False))
    self.assertEqual(conditional_ring_state(sm, 1, 100.), 'ready')
    policy.e2eEnabled, policy.state = True, 'active'
    self.assertEqual(conditional_ring_state(sm, 1, 100.), 'active')
    self.assertEqual(conditional_ring_state(sm, 1, 101.), 'ready')
    sm['carState'].gasPressed = True
    self.assertEqual(conditional_ring_state(sm, 1, 100.), 'ready')
    sm['carState'].gasPressed = False
    sm['carState'].brakePressed = True
    self.assertEqual(conditional_ring_state(sm, 1, 100.), 'ready')
    sm['carState'].brakePressed = False
    sm['selfdriveState'].enabled = False
    sm.valid['carControl'] = False
    self.assertEqual(conditional_ring_state(sm, 1, 100.), 'ready')
    sm['selfdriveState'].experimentalMode = True
    self.assertEqual(conditional_ring_state(sm, 1, 100.), 'ready')

  def test_mode_cycle_and_nonoverlapping_parameter_writes(self):
    class Params:
      values = {'ExperimentalMode': False, 'ConditionalExperimentalMode': False}
      def put_bool(self, key, value, block=True):
        self.values[key] = value
        assert not all(self.values.values())
    params = Params()
    for _ in range(3):
      for mode in (LongitudinalMode.conditional, LongitudinalMode.experimental, LongitudinalMode.voacc):
        set_longitudinal_mode(params, mode)
        self.assertEqual(selected_mode(params.values['ExperimentalMode'], params.values['ConditionalExperimentalMode']), mode)


class TestNoSpeedIcon(unittest.TestCase):
  def setUp(self):
    self.sm = FakeSM(
      mapCruiseState=NS(automaticE2e=True, e2eFallback=True, state='paused'),
      longitudinalPlan=NS(conditionalExperimental=NS(e2eEnabled=True, state='active', reason='noSpeedLimit')),
      selfdriveState=NS(conditionalExperimental=True, experimentalMode=False, enabled=True),
      carControl=NS(longActive=True), carState=NS(gasPressed=False, brakePressed=False))

  def test_confirmed_fallback_shows_icon_even_with_paused_slc(self):
    self.assertTrue(no_speed_e2e_active(self.sm, 1, 100.))

  def test_inactive_other_conditions_and_recovered_map_cannot_show_override(self):
    cases = [('mapCruiseState', None, 'automaticE2e', False), ('mapCruiseState', None, 'e2eFallback', False),
             ('mapCruiseState', None, 'state', 'off'), ('mapCruiseState', None, 'state', 'unsupported'),
             ('longitudinalPlan', 'conditionalExperimental', 'e2eEnabled', False),
             ('longitudinalPlan', 'conditionalExperimental', 'state', 'ready'),
             ('longitudinalPlan', 'conditionalExperimental', 'reason', 'junction'),
             ('selfdriveState', None, 'conditionalExperimental', False), ('selfdriveState', None, 'experimentalMode', True),
             ('selfdriveState', None, 'enabled', False), ('carControl', None, 'longActive', False),
             ('carState', None, 'gasPressed', True), ('carState', None, 'brakePressed', True)]
    for service, child, attr, value in cases:
      with self.subTest(service=service, attr=attr, value=value):
        self.setUp()
        message = getattr(self.sm[service], child) if child else self.sm[service]
        setattr(message, attr, value)
        self.assertFalse(no_speed_e2e_active(self.sm, 1, 100.))

  def test_invalid_stale_or_previous_drive_messages_cannot_show_override(self):
    for service in self.sm.services:
      for attr, value in (('valid', False), ('recv_frame', 0), ('recv_time', 99.), ('logMonoTime', int(99e9))):
        with self.subTest(service=service, attr=attr):
          self.setUp()
          getattr(self.sm, attr)[service] = value
          self.assertFalse(no_speed_e2e_active(self.sm, 1, 100.))


class TestSLCState(unittest.TestCase):
  def test_simulator_messages_without_service_metadata_cannot_establish_map_conditions(self):
    sm = {'mapCruiseState': NS(automaticE2e=True, e2eFallback=True),
          'mapTrafficControl': NS(kind='junction', nodeId=10, distance=1.)}
    self.assertEqual(slc_fallback_request(sm, 100., False), (False, False))
    self.assertEqual(map_approach(sm, 100.), MapApproach())

  def sm(self, **kwargs):
    msg = NS(state='waiting', automaticE2e=True, e2eFallback=True, setSpeed=105 / 3.6)
    for k, v in kwargs.items():
      setattr(msg, k, v)
    return FakeSM(mapCruiseState=msg, carState=NS(vCruise=105.0))

  def test_fresh_slc_request_and_usable_limit(self):
    sm = self.sm()
    self.assertEqual(slc_fallback_request(sm, 100.0, False), (True, True))
    sm['mapCruiseState'].e2eFallback = False
    sm['mapCruiseState'].state = 'active'
    self.assertEqual(slc_fallback_request(sm, 100.0, False), (True, False))
    # This reader handles the no-speed branch; map_approach qualifies junctions separately.
    sm['mapTrafficControl'] = NS(kind='junction', nodeId=10, distance=1.0)
    self.assertEqual(slc_fallback_request(sm, 100.0, False), (True, False))

  def test_recovery_waits_for_updated_carstate_set(self):
    sm = self.sm(e2eFallback=False, state='active', setSpeed=50 / 3.6)
    self.assertEqual(slc_fallback_request(sm, 100.0, True), (True, True))
    sm['carState'].vCruise = 50.0
    self.assertEqual(slc_fallback_request(sm, 100.0, True), (True, False))

  def test_stale_status_cannot_start_fallback_or_release_established_fallback(self):
    sm = self.sm()
    self.assertEqual(slc_fallback_request(sm, 102.0, False), (False, False))
    self.assertEqual(slc_fallback_request(sm, 102.0, True), (True, True))
    sm.valid['mapCruiseState'] = False
    self.assertEqual(slc_fallback_request(sm, 100.0, False), (False, False))
    self.assertEqual(slc_fallback_request(FakeSM(), 100.0, True), (True, True))

  def test_fresh_off_unsupported_or_unconfirmed_mode_clears_fallback(self):
    for args in ({'state': 'off'}, {'state': 'unsupported'}, {'automaticE2e': False}):
      self.assertEqual(slc_fallback_request(self.sm(**args), 100.0, True), (False, False))

  def test_conditional_requires_opt_in_and_mode_cycle_preserves_slc(self):
    class Params:
      def __init__(self):
        self.values = {'ExperimentalMode': False, 'ConditionalExperimentalMode': False, 'MapCruiseEnabled': True, 'ExperimentalModeConfirmed': True}

      def put_bool(self, key, value, block=True):
        self.values[key] = value
        assert not (self.values['ExperimentalMode'] and self.values['ConditionalExperimentalMode'])

      def get_bool(self, key):
        return self.values.get(key, False)

      def get(self, key, return_default=False):
        return self.values.get(key, False)

    p = Params()
    cp = NS(openpilotLongitudinalControl=True, pcmCruise=False, notCar=False, passive=False)
    self.assertFalse(automatic_e2e_selected(p, cp, False))  # SLC alone is ordinary ACC
    for slc in (True, False):
      p.put_bool('MapCruiseEnabled', slc)
      for mode in (LongitudinalMode.voacc, LongitudinalMode.conditional, LongitudinalMode.experimental, LongitudinalMode.voacc):
        set_longitudinal_mode(p, mode)
        full = p.get_bool('ExperimentalMode')
        automatic = automatic_e2e_selected(p, cp, full)
        self.assertEqual(selected_mode(full, automatic), mode)
        self.assertEqual(p.get_bool('MapCruiseEnabled'), slc)
    p.put_bool('MapCruiseEnabled', True)
    self.assertFalse(automatic_e2e_selected(p, cp, False))
    set_longitudinal_mode(p, LongitudinalMode.conditional)
    self.assertTrue(automatic_e2e_selected(p, cp, False))
    p.put_bool('ExperimentalModeConfirmed', False)
    self.assertFalse(automatic_e2e_selected(p, cp, False))
    p.put_bool('ExperimentalModeConfirmed', True)
    for key in ('pcmCruise', 'notCar', 'passive'):
      setattr(cp, key, True)
      self.assertFalse(automatic_e2e_selected(p, cp, False))
      setattr(cp, key, False)
    cp.openpilotLongitudinalControl = False
    self.assertFalse(automatic_e2e_selected(p, cp, False))


if __name__ == '__main__':
  unittest.main()
