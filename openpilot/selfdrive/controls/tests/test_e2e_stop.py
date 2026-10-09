import math
import unittest
from types import SimpleNamespace as NS

from openpilot.selfdrive.controls.lib.e2e_stop import E2EStopTarget
from openpilot.selfdrive.modeld.constants import ModelConstants


def model(speed=15., brake=1.5):
  times = ModelConstants.T_IDXS
  velocities = [max(0., speed - brake * t) for t in times]
  travel_times = [min(t, speed / brake) if brake > 0 else t for t in times]
  positions = [speed * t - .5 * brake * t**2 for t in travel_times]
  return NS(velocity=NS(x=velocities, y=[0.] * len(times)), position=NS(x=positions, y=[0.] * len(times)))


class TestE2EStopTarget(unittest.TestCase):
  def setUp(self):
    self.target = E2EStopTarget()
    self.now = 100.

  def step(self, **kwargs):
    self.now += .05
    args = {'eligible': True, 'model': model(speed=8., brake=2.), 'now': self.now, 'model_time': self.now,
            'v_ego': 8., 'e2e_accel': -1., 'e2e_stop': False}
    self.target.update(**(args | kwargs))
    return self.target

  def test_future_stop_creates_target_1p5m_earlier_without_immediate_hold(self):
    t = self.step()
    self.assertTrue(t.active)
    self.assertFalse(t.holding)
    self.assertAlmostEqual(t.model_distance, 16., delta=.1)
    self.assertAlmostEqual(t.distance, t.model_distance - 1.5)

  def test_overlapping_lead_stop_does_not_create_duplicate_or_apply_margin(self):
    raw_stop = self.step(v_ego=0.).model_distance
    for lead_stop in (0., 8., raw_stop, raw_stop + 1.9, raw_stop + 2.):
      with self.subTest(lead_stop=lead_stop):
        self.target.reset()
        t = self.step(v_ego=0., lead_stop_distance=lead_stop)
        self.assertFalse(t.active)
        self.assertFalse(t.requested)
        self.assertFalse(t.holding)
    # Raw model stop is 16 m. Its shifted 14.5 m target is not used to
    # manufacture separation from a real lead's 16 m stopping position.

  def test_distinct_earlier_stop_coexists_with_real_lead(self):
    t = self.step(v_ego=0., lead_stop_distance=20.)
    self.assertTrue(t.active)
    self.assertAlmostEqual(t.distance, 14.5, delta=.1)
    self.assertAlmostEqual(t.model_distance, 16., delta=.1)

  def test_lead_stop_matching_has_hysteresis(self):
    raw_stop = self.step(v_ego=0.).model_distance
    self.target.reset()
    self.assertTrue(self.step(v_ego=0., lead_stop_distance=raw_stop + 2.1).active)
    for separation in (1.9, 1.2, 1.8, 1.1):
      self.assertTrue(self.step(v_ego=0., lead_stop_distance=raw_stop + separation).active)
    self.assertFalse(self.step(v_ego=0., lead_stop_distance=raw_stop + 1.).active)
    self.assertFalse(self.step(v_ego=0., lead_stop_distance=raw_stop + 1.9).active)
    self.assertTrue(self.step(v_ego=0., lead_stop_distance=raw_stop + 2.1).active)

  def test_stop_reacquires_from_current_model_when_lead_moves_beyond_it(self):
    self.assertFalse(self.step(lead_stop_distance=10.).active)
    self.assertTrue(self.step(lead_stop_distance=20.).active)
    self.assertFalse(self.step(lead_stop_distance=10.).active)
    prediction = model(speed=6., brake=2.)  # new stop is 9 m, not the previous 16 m
    t = self.step(model=prediction, lead_stop_distance=20.)
    self.assertTrue(t.active)
    self.assertAlmostEqual(t.distance, 7.5, delta=.1)

  def test_stopped_lead_covers_explicit_model_hold(self):
    t = self.step(model=model(speed=0.), v_ego=0., e2e_stop=True, lead_stop_distance=0.)
    self.assertFalse(t.active)
    self.assertFalse(t.requested)
    self.assertFalse(t.holding)
    # A lead beyond a separate stop must not release that stop's hold.
    t = self.step(model=model(speed=0.), v_ego=0., e2e_stop=True, lead_stop_distance=10.)
    self.assertTrue(t.active)
    self.assertTrue(t.holding)

  def test_unknown_stop_position_with_lead_does_not_invent_separate_stop(self):
    t = self.step(model=model(brake=0.), e2e_stop=True, lead_stop_distance=10.)
    self.assertFalse(t.requested)
    self.assertFalse(t.holding)
    self.step(v_ego=0., lead_stop_distance=25.)
    t = self.step(model=model(brake=0.), v_ego=0., e2e_stop=True, lead_stop_distance=25.)
    self.assertTrue(t.active)
    self.assertTrue(t.holding)

  def test_margin_is_applied_once_and_nearby_target_does_not_go_negative(self):
    initial = self.step(v_ego=0.).distance
    for _ in range(200):
      self.assertAlmostEqual(self.step(v_ego=0.).distance, initial)
    self.target.reset()
    t = self.step(model=model(speed=1., brake=1.), v_ego=1.)
    self.assertLess(t.model_distance, 1.5)
    self.assertGreaterEqual(t.model_distance, 0.)
    self.assertEqual(t.distance, 0.)
    self.assertFalse(t.holding)  # still moving; margin must not force standstill

  def test_margin_and_model_age_compensation_both_apply(self):
    t = self.step(model_time=self.now - .05)
    self.assertAlmostEqual(t.distance, t.model_distance - 8. * .1 - 1.5)

  def test_explicit_stop_without_geometry_does_not_invent_obstacle(self):
    t = self.step(model=model(brake=0.), e2e_stop=True)
    self.assertTrue(t.requested)
    self.assertTrue(t.holding)
    self.assertFalse(t.active)
    self.assertEqual(t.distance, -1.)

  def test_held_position_survives_explicit_stop_without_trajectory_location(self):
    self.step()
    before = self.target.distance
    t = self.step(model=model(brake=0.), e2e_stop=True)
    self.assertTrue(t.active)
    self.assertAlmostEqual(t.distance, before - .4)

  def test_non_stop_slowing_and_single_zero_do_not_create_target(self):
    noisy = model(brake=0.)
    noisy.velocity.x[20] = 0.
    for prediction in (model(), model(brake=0.), noisy):
      self.assertFalse(self.step(model=prediction).requested)

  def test_target_advances_by_measured_travel_and_cannot_recede(self):
    self.step()
    distance = self.target.distance
    for _ in range(20):
      t = self.step()  # model erroneously keeps placing the stop 16 m ahead
      self.assertAlmostEqual(t.distance, distance - .4)
      distance = t.distance

  def test_nearer_updates_are_filtered_and_timestamp_age_is_compensated(self):
    self.step()
    initial = self.target.distance
    prediction = model(speed=8., brake=2.)
    prediction.position.x = [x / 2 for x in prediction.position.x]
    t = self.step(model=prediction)
    self.assertGreater(t.distance, t.model_distance)
    self.assertLess(t.distance, initial - .4)
    self.target.reset()
    self.assertAlmostEqual(self.step(model_time=self.now).distance, initial - .4)

  def test_symmetric_jitter_does_not_accumulate_as_earlier_stop_margin(self):
    initial = self.step(v_ego=0.).distance
    for i in range(200):
      prediction = model(speed=8., brake=2.)
      scale = (initial + (.2 if i % 2 else -.2)) / initial
      prediction.position.x = [x * scale for x in prediction.position.x]
      self.step(model=prediction, v_ego=0.)
    self.assertGreater(self.target.distance, initial - .05)

  def test_fresh_withdrawal_releases_position_and_hold_immediately(self):
    self.step(model=model(speed=0.), v_ego=0., e2e_stop=True)
    t = self.step(model=model(brake=0.), e2e_accel=.5)
    self.assertFalse(t.requested)
    self.assertFalse(t.active)
    self.assertFalse(t.holding)
    self.assertEqual(t.distance, -1.)

  def test_departure_releases_even_with_a_stationary_trajectory_prefix(self):
    self.step(model=model(speed=0.), v_ego=0., e2e_stop=True)
    prediction = model(speed=0.)
    prediction.velocity.x = [max(0., 2 * (t - .85)) for t in ModelConstants.T_IDXS]
    prediction.position.x = [max(0., t - .85)**2 for t in ModelConstants.T_IDXS]
    t = self.step(model=prediction, v_ego=0., e2e_accel=.5)
    self.assertFalse(t.requested)
    self.assertFalse(t.holding)

  def test_positive_action_alone_does_not_release_a_future_stop(self):
    self.assertTrue(self.step(e2e_accel=.5).active)
    self.assertTrue(self.step(model=model(speed=0.), v_ego=0., e2e_accel=.5).requested)

  def test_slow_departure_does_not_recreate_the_stationary_prefix_target(self):
    self.step(model=model(speed=0.), v_ego=0., e2e_stop=True)
    for factor in (.95, 1.05, .9, 1.1):
      prediction = model(speed=0.)
      prediction.velocity.x = [factor * max(0., .8 * (t - .85)) for t in ModelConstants.T_IDXS]
      prediction.position.x = [factor * .4 * max(0., t - .85)**2 for t in ModelConstants.T_IDXS]
      self.assertFalse(self.step(model=prediction, v_ego=0., e2e_accel=.3).requested)

  def test_hold_at_target_and_no_negative_distance_after_passing(self):
    for _ in range(60):
      self.step()
    self.assertEqual(self.target.distance, 0.)
    self.assertFalse(self.target.holding)
    self.assertTrue(self.step(v_ego=0.).holding)

  def test_invalid_or_ineligible_data_clear_all_history(self):
    bad = model(speed=8., brake=2.)
    bad.velocity.x[5] = math.nan
    incomplete = model(speed=8., brake=2.)
    incomplete.position.x = []
    for override in ({'eligible': False}, {'model': bad}, {'model': incomplete}, {'v_ego': math.nan},
                     {'v_ego': -.2}, {'e2e_accel': math.nan}, {'model_time': 0.}, {'model_time': 10000.}):
      with self.subTest(override=override):
        self.step()
        t = self.step(**override)
        self.assertFalse(t.active)
        self.assertFalse(t.holding)
        self.assertEqual(t.distance, -1.)

  def test_repeated_frame_only_advances_travel_and_stale_frame_expires(self):
    self.step()
    stamp, distance = self.target.timestamp, self.target.distance
    self.step(model_time=stamp)
    self.assertAlmostEqual(self.target.distance, distance - .4)
    self.now += .3
    self.assertFalse(self.step(model_time=stamp).active)

  def test_regressing_model_or_clock_resets(self):
    self.step()
    self.assertFalse(self.step(model_time=self.target.timestamp - .01).active)
    self.step()
    self.assertFalse(self.step(now=self.now - 1.).active)

  def test_far_prediction_has_no_virtual_obstacle(self):
    prediction = model(speed=30., brake=4.)
    prediction.position.x = [x * 2 for x in prediction.position.x]
    self.assertFalse(self.step(model=prediction, v_ego=30.).active)


if __name__ == '__main__':
  unittest.main()
