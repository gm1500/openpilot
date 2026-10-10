import math
import unittest

from openpilot.selfdrive.controls.lib.e2e_stop import E2EStopTarget
from openpilot.selfdrive.controls.lib.model_intent import model_intent
from openpilot.selfdrive.controls.lib.stop_horizon import StableStopHorizon, horizon_stop
from openpilot.selfdrive.controls.tests.test_e2e_stop import model
from openpilot.selfdrive.modeld.constants import ModelConstants


def endpoint_model(distance=16., residual_speed=0.):
  prediction = model(speed=8., brake=2.)
  prediction.position.x = [x * distance / 16. for x in prediction.position.x]
  prediction.velocity.x = [max(residual_speed, v) for v in prediction.velocity.x]
  return prediction


class TestHorizonShape(unittest.TestCase):
  def test_compact_tail_has_time_remaining_and_preserves_existing_stop_position(self):
    prediction = endpoint_model()
    result = horizon_stop(prediction, 8.)
    self.assertEqual(result.distance, model_intent(prediction, 8.).stop_distance)
    self.assertEqual(result.endpoint, 16.)
    self.assertGreater(result.remaining_time, 1.5)

  def test_geometry_can_qualify_without_near_zero_speed_or_stop_flag(self):
    prediction = endpoint_model(residual_speed=.7)
    self.assertEqual(model_intent(prediction, 8.).stop_distance, -1.)
    self.assertGreater(horizon_stop(prediction, 8.).distance, 15.)

  def test_finite_horizon_creeping_and_velocity_only_stop_do_not_qualify(self):
    moving_geometry = model(speed=8., brake=0.)
    moving_geometry.velocity.x = endpoint_model().velocity.x
    for prediction, speed in ((model(brake=0.), 15.), (model(speed=.2, brake=0.), .2),
                              (model(speed=8., brake=.8), 8.), (moving_geometry, 8.)):
      self.assertEqual(horizon_stop(prediction, speed).distance, -1.)

  def test_turn_with_flat_forward_component_is_not_a_stop(self):
    prediction = model(speed=8., brake=0.)
    prediction.position.x, prediction.position.y = prediction.position.y, prediction.position.x
    prediction.velocity.x, prediction.velocity.y = prediction.velocity.y, prediction.velocity.x
    self.assertEqual(horizon_stop(prediction, 8.).distance, -1.)

  def test_stop_then_launch_is_not_a_terminal_stop(self):
    prediction = endpoint_model()
    for i, t in enumerate(ModelConstants.T_IDXS):
      if t > 7.:
        prediction.position.x[i] += (t - 7.)**2
        prediction.velocity.x[i] = 2. * (t - 7.)
    self.assertEqual(horizon_stop(prediction, 8.).distance, -1.)

  def test_standstill_and_invalid_predictions(self):
    self.assertEqual(horizon_stop(model(speed=0.), 0.).distance, 0.)
    for bad in ([], [math.nan] * 33, [math.inf] * 33):
      prediction = endpoint_model()
      prediction.position.x = bad
      self.assertEqual(horizon_stop(prediction, 8.).distance, -1.)


class TestHorizonStability(unittest.TestCase):
  def setUp(self):
    self.horizon = StableStopHorizon()

  def step(self, frame, distance=None, **kwargs):
    now = 100. + frame * .05
    self.horizon.update(endpoint_model(16. - frame * .4 if distance is None else distance),
                        **({'now': now, 'model_time': now, 'v_ego': 8.} | kwargs))
    return self.horizon

  def test_requires_observed_time_and_motion_compensation(self):
    for frame in range(6):
      self.assertFalse(self.step(frame).stable)
    self.assertTrue(self.step(6).stable)
    self.assertLess(self.horizon.spread, 1e-6)

  def test_repeated_frames_cannot_confirm_and_stale_frames_clear(self):
    for frame in range(6):
      self.assertFalse(self.step(frame, model_time=100.).stable)
    self.assertFalse(self.step(8, model_time=100.).stable)
    self.assertEqual(self.horizon.observation.distance, -1.)

  def test_endpoint_moving_with_car_and_jumping_endpoint_do_not_confirm(self):
    for frame in range(30):
      self.assertFalse(self.step(frame, distance=16.).stable)
    self.horizon.reset()
    for frame in range(30):
      self.assertFalse(self.step(frame, distance=16. - frame * .4 + (2. if frame % 2 else 0.)).stable)

  def test_small_noise_and_observation_age_are_compensated(self):
    for frame in range(10):
      # The path is from 100 ms earlier: it still contains 0.8 m more travel.
      h = self.step(frame, distance=16.8 - frame * .4 + (.1 if frame % 2 else -.1),
                    model_time=99.9 + frame * .05)
    self.assertTrue(h.stable)
    self.assertLess(h.spread, .21)

  def test_gap_regression_withdrawal_and_invalid_data_clear_confirmation(self):
    for kwargs in ({'now': 101., 'model_time': 101.}, {'model_time': 100.1}, {'v_ego': math.nan}):
      self.horizon.reset()
      for frame in range(7):
        self.step(frame)
      self.assertFalse(self.step(7, **kwargs).stable)
    for frame in range(7):
      self.step(frame)
    self.horizon.update(model(brake=0.), now=100.35, model_time=100.35, v_ego=8.)
    self.assertFalse(self.horizon.stable)
    self.assertEqual(self.horizon.observation.distance, -1.)


class TestStableHorizonTarget(unittest.TestCase):
  def setUp(self):
    self.target = E2EStopTarget()

  def step(self, frame, **kwargs):
    now = 100. + frame * .05
    args = {'eligible': True, 'model': endpoint_model(16. - frame * .4), 'now': now, 'model_time': now,
            'v_ego': 8., 'e2e_accel': -1., 'e2e_stop': False}
    self.target.update(**(args | kwargs))
    return self.target

  def acquire(self, **kwargs):
    for frame in range(7):
      self.step(frame, **kwargs)
    self.assertTrue(self.target.active)

  def test_existing_trajectory_still_activates_immediately_without_offset(self):
    t = self.step(0)
    self.assertTrue(t.active)
    self.assertFalse(t.holding)
    self.assertEqual(t.distance, t.model_distance)
    self.assertAlmostEqual(t.distance, model_intent(endpoint_model(), 8.).stop_distance)

  def test_geometric_stop_can_activate_without_velocity_stop(self):
    for frame in range(7):
      t = self.step(frame, model=endpoint_model(16. - frame * .4, residual_speed=.7))
      if frame < 6:
        self.assertFalse(t.active)
    self.assertTrue(t.active)
    self.assertTrue(t.requested)
    self.assertFalse(t.holding)
    previous = t.distance
    t = self.step(7)  # near-zero confirmation refines the SAME tracked target
    self.assertTrue(t.active)
    self.assertLessEqual(t.distance, previous - .4 + 1e-6)
    self.assertAlmostEqual(t.distance, previous - .4)  # confirmation cannot push the stop farther away

  def test_geometry_jitter_keeps_confirmed_near_zero_stop_but_withdrawal_releases(self):
    self.acquire()
    prediction = endpoint_model(13.2)
    prediction.position.x[-1] += 2.  # compact tail fails; near-zero intent still holds
    self.assertTrue(self.step(7, model=prediction).active)
    self.assertFalse(self.target.horizon.stable)
    self.assertFalse(self.step(8, model=model(brake=0.), e2e_accel=.5).active)
    self.assertFalse(self.target.holding)
    self.assertEqual(self.target.distance, -1.)

  def test_explicit_stop_holds_immediately_without_inventing_a_position(self):
    t = self.step(0, model=model(brake=0.), v_ego=0., e2e_stop=True)
    self.assertTrue(t.requested)
    self.assertTrue(t.holding)
    self.assertFalse(t.active)

  def test_brake_hold_and_departure(self):
    self.acquire()
    self.assertTrue(self.step(7, model=model(speed=0.), v_ego=0., e2e_stop=True).holding)
    prediction = model(speed=0.)
    prediction.velocity.x = [max(0., 2 * (t - .85)) for t in ModelConstants.T_IDXS]
    prediction.position.x = [max(0., t - .85)**2 for t in ModelConstants.T_IDXS]
    self.assertFalse(self.step(8, model=prediction, v_ego=0., e2e_accel=.5).requested)

  def test_duplicate_lead_is_suppressed_but_distinct_stop_can_qualify(self):
    for frame in range(10):
      self.assertFalse(self.step(frame, lead_stop_distance=16. - frame * .4).active)
    self.target.reset()
    self.acquire(lead_stop_distance=30.)
    self.assertFalse(self.step(7, lead_stop_distance=13.).active)
    self.assertFalse(self.target.requested)

  def test_disengagement_pedal_and_invalid_input_eligibility_clear_history(self):
    self.acquire()
    self.assertFalse(self.step(7, eligible=False).active)
    self.assertFalse(self.target.horizon.stable)
    self.assertFalse(self.step(8, model=endpoint_model(residual_speed=.7)).active)


if __name__ == '__main__':
  unittest.main()
