import math
import unittest
from types import SimpleNamespace as NS

from openpilot.selfdrive.controls.lib.conditional_experimental import (
  ConditionalExperimental, MapApproach, activation_distance, map_approach, model_intent,
)
from openpilot.selfdrive.controls.lib.longitudinal_mode import LongitudinalMode, selected_mode, set_longitudinal_mode
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.ui.onroad.conditional_icon import conditional_ring_state


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

  def step(self, **kwargs):
    args = {'enabled': True, 'eligible': True, 'approach': MapApproach(True, 10, 80., 101.), 'model': model(),
            'v_ego': 15., 'a_ego': 0., 'personality': 1, 'regular_accel': .5, 'regular_stop': False, 'e2e_accel': -1., 'e2e_stop': False}
    args.update(kwargs)
    return self.policy.update(**args)

  def arm(self, **kwargs):
    self.step(approach=MapApproach(True, 10, 80., 100.), e2e_accel=0.)
    return self.step(**kwargs)

  def enter(self, **kwargs):
    self.arm(**kwargs)
    for _ in range(6):
      result = self.step(**kwargs)
    return result

  def test_map_range_and_model_evidence_are_separate(self):
    self.arm(e2e_accel=.2, model=model(brake=0))
    self.assertTrue(self.policy.armed)
    self.assertEqual(self.policy.state, 'inRange')
    for _ in range(100):
      self.assertEqual(self.step(e2e_accel=.2, model=model(brake=0)), (.5, False, False))

  def test_two_advancing_map_observations_required(self):
    for _ in range(100):
      self.assertEqual(self.step(), (.5, False, False))
    self.assertFalse(self.policy.armed)
    self.step(approach=MapApproach(True, 10, 80., 102.))
    self.assertTrue(self.policy.armed)

  def test_sustained_model_slowing_and_material_difference_required(self):
    self.arm(e2e_accel=0)
    self.assertEqual(self.step(), (.5, False, False))
    self.step(e2e_accel=0)
    for _ in range(10):
      self.assertEqual(self.step(regular_accel=-.9), (-.9, False, False))
    self.assertEqual(self.enter(), (-1., False, True))
    self.assertEqual(self.policy.state, 'assisting')

  def test_always_preserves_stronger_regular_braking(self):
    self.enter()
    self.assertEqual(self.step(regular_accel=-3), (-3, False, False))
    self.assertEqual(self.policy.state, 'inRange')

  def test_range_latches_as_speed_and_distance_shrink(self):
    self.enter()
    self.step(v_ego=2., model=model(speed=2), approach=MapApproach(True, 10, 35., 102.))
    self.assertLess(self.policy.activation_distance, 35.)
    self.assertTrue(self.policy.armed)
    self.assertTrue(self.policy.braking)

  def test_model_go_releases_and_slews_return_to_acceleration(self):
    previous, _, _ = self.enter()
    for _ in range(50):
      current, stop, _ = self.step(e2e_accel=.3, model=model(brake=0))
      self.assertLessEqual(current - previous, .05 + 1e-8)
      self.assertFalse(stop)
      if current > 0:
        self.assertNotEqual(self.policy.state, 'assisting')
      previous = current
    self.assertEqual(current, .5)
    self.assertFalse(self.policy.braking)
    self.assertTrue(self.policy.armed)

  def test_stop_holds_on_map_dropout_only_with_fresh_model_request(self):
    self.enter()
    for _ in range(50):
      _, stop, active = self.step(v_ego=0, model=model(speed=0), e2e_accel=0., e2e_stop=True, approach=MapApproach())
      self.assertTrue(stop)
      self.assertTrue(active)
    _, stop, _ = self.step(v_ego=0, model=model(speed=0), e2e_accel=.3, e2e_stop=False, approach=MapApproach())
    self.assertFalse(stop)

  def test_map_loss_while_moving_releases_and_cannot_create_stop(self):
    self.enter()
    previous = -1.
    for _ in range(50):
      current, stop, _ = self.step(approach=MapApproach(), e2e_stop=True)
      self.assertFalse(stop)
      self.assertLessEqual(current - previous, .05 + 1e-8)
      previous = current
    self.assertEqual(current, .5)
    self.assertFalse(self.policy.armed)

  def test_override_disabled_or_invalid_model_clear_immediately(self):
    bad = model()
    bad.velocity.x[3] = math.nan
    for args in ({'eligible': False}, {'enabled': False}, {'model': bad}, {'a_ego': math.nan}, {'e2e_accel': math.inf}):
      self.enter()
      self.assertEqual(self.step(**args), (.5, False, False))
      self.assertFalse(self.policy.braking)
      self.assertFalse(self.policy.armed)

  def test_passed_changed_or_missing_target_releases(self):
    for approach in (MapApproach(True, 10, -15., 102.), MapApproach(True, 20, 70., 102.), MapApproach(True)):
      self.policy.reset()
      self.enter()
      self.step(approach=approach)
      self.assertFalse(self.policy.braking)
      self.assertFalse(self.policy.armed)

  def test_cannot_enter_for_target_out_of_range_or_already_passed(self):
    for distance in (300., -1.):
      self.policy.reset()
      for timestamp in range(100, 120):
        self.assertEqual(self.step(approach=MapApproach(True, 10, distance, timestamp)), (.5, False, False))

  def test_passed_target_cannot_rearm_from_position_jitter(self):
    self.enter()
    self.step(approach=MapApproach(True, 10, -15., 102.))
    for i in range(100):
      self.step(approach=MapApproach(True, 10, 2., 103. + i))
      self.assertFalse(self.policy.armed)
      self.assertFalse(self.policy.braking)

  def test_older_map_observation_discards_qualification(self):
    self.enter()
    self.step(approach=MapApproach(True, 10, 50., 100.5))
    self.assertFalse(self.policy.armed)
    self.assertFalse(self.policy.braking)

  def test_much_later_predicted_stop_does_not_confirm_this_target(self):
    self.arm(approach=MapApproach(True, 10, 5., 101.), model=model(brake=1.8))
    for _ in range(10):
      self.assertEqual(self.step(approach=MapApproach(True, 10, 5., 101.), model=model(brake=1.8)), (.5, False, False))


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
  def test_map_health_is_independent_of_speed_limit(self):
    msg = NS(kind='stopSign', nodeId=7, distance=50., gpsMonoTime=int(99e9), positionMonoTime=int(99e9))
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

  def test_only_junction_contribution_shows_amber(self):
    policy = NS(armed=True, contributing=False, state='inRange')
    sm = FakeSM(longitudinalPlan=NS(conditionalExperimental=policy, e2eAssistActive=True),
                selfdriveState=NS(conditionalExperimental=True, experimentalMode=False, enabled=True),
                carControl=NS(longActive=True), carState=NS(gasPressed=False, brakePressed=False))
    self.assertEqual(conditional_ring_state(sm, 1, 100.), 'inRange')
    policy.contributing, policy.state = True, 'assisting'
    self.assertEqual(conditional_ring_state(sm, 1, 100.), 'assisting')
    self.assertEqual(conditional_ring_state(sm, 1, 101.), 'ready')
    sm['carState'].gasPressed = True
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


if __name__ == '__main__':
  unittest.main()
