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


def slow_pass_model(distance=8., acceleration=-1.):
  """Slow nearly to a stop around distance, then predict a later departure."""
  times = ModelConstants.T_IDXS
  velocities = [4. - (3.2 / 3) * t if t <= 3. else .8 if t <= 5. else .8 + .8 * (t - 5.) for t in times]
  positions = [4. * t - .5 * (3.2 / 3) * t**2 if t <= 3. else
               7.2 + .8 * (t - 3.) if t <= 5. else 8.8 + .8 * (t - 5.) + .4 * (t - 5.)**2 for t in times]
  return NS(velocity=NS(x=velocities, y=[0.] * len(times)),
            position=NS(x=[p * distance / 8. for p in positions], y=[0.] * len(times)),
            action=NS(desiredAcceleration=acceleration))


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

  def test_future_stop_uses_model_position_without_offset_or_immediate_hold(self):
    t = self.step()
    self.assertTrue(t.active)
    self.assertFalse(t.holding)
    self.assertAlmostEqual(t.model_distance, 16., delta=.1)
    self.assertAlmostEqual(t.distance, t.model_distance)

  def test_overlapping_lead_stop_does_not_create_duplicate(self):
    raw_stop = self.step(v_ego=0.).model_distance
    for lead_stop in (0., 8., raw_stop, raw_stop + 4.9, raw_stop + 5., raw_stop + 6.):
      with self.subTest(lead_stop=lead_stop):
        self.target.reset()
        t = self.step(v_ego=0., lead_stop_distance=lead_stop)
        self.assertFalse(t.active)
        self.assertTrue(t.requested)  # E2E action remains available; no duplicate obstacle
        self.assertFalse(t.holding)

  def test_distinct_earlier_stop_coexists_with_real_lead(self):
    t = self.step(v_ego=0., lead_stop_distance=25.)
    self.assertTrue(t.active)
    self.assertAlmostEqual(t.distance, 16., delta=.1)
    self.assertAlmostEqual(t.model_distance, 16., delta=.1)

  def test_lead_stop_matching_has_hysteresis(self):
    raw_stop = self.step(v_ego=0.).model_distance
    self.target.reset()
    self.assertTrue(self.step(v_ego=0., lead_stop_distance=raw_stop + 6.1).active)
    for separation in (5.9, 5.2, 5.8, 5.1):
      self.assertTrue(self.step(v_ego=0., lead_stop_distance=raw_stop + separation).active)
    self.assertFalse(self.step(v_ego=0., lead_stop_distance=raw_stop + 5.).active)
    self.assertFalse(self.step(v_ego=0., lead_stop_distance=raw_stop + 5.9).active)
    self.assertTrue(self.step(v_ego=0., lead_stop_distance=raw_stop + 6.1).active)

  def test_virtual_stop_stays_before_physical_lead_even_if_its_projected_stop_is_farther(self):
    stop_distance = self.step(v_ego=0.).model_distance
    for lead_distance in (1., 5., 15., stop_distance, stop_distance + 4., stop_distance + 6.):
      self.target.reset()
      self.assertFalse(self.step(v_ego=0., lead_stop_distance=100., lead_distance=lead_distance).active)
    # Once a running lead clears the stop by the original entry margin, the
    # qualified model stop acquires at its original location, not behind it.
    t = self.step(v_ego=0., lead_stop_distance=100., lead_distance=stop_distance + 6.1)
    self.assertTrue(t.active)
    self.assertLess(t.distance, stop_distance + .1)
    self.assertTrue(self.step(v_ego=0., lead_stop_distance=100., lead_distance=stop_distance + 5.1).active)
    self.assertFalse(self.step(v_ego=0., lead_stop_distance=100., lead_distance=stop_distance + 5.).active)

  def test_stop_reacquires_from_current_model_when_lead_moves_beyond_it(self):
    self.assertFalse(self.step(lead_stop_distance=10.).active)
    self.assertTrue(self.step(lead_stop_distance=25.).active)
    self.assertFalse(self.step(lead_stop_distance=10.).active)
    prediction = model(speed=6., brake=2.)  # new stop is 9 m, not the previous 16 m
    t = self.step(model=prediction, lead_stop_distance=25.)
    self.assertTrue(t.active)
    self.assertAlmostEqual(t.distance, 9., delta=.1)

  def test_stopped_lead_covers_explicit_model_hold(self):
    t = self.step(model=model(speed=0.), v_ego=0., e2e_stop=True, lead_stop_distance=0.)
    self.assertFalse(t.active)
    self.assertTrue(t.requested)
    self.assertTrue(t.holding)
    # A lead beyond a separate stop must not release that stop's hold.
    t = self.step(model=model(speed=0.), v_ego=0., e2e_stop=True, lead_stop_distance=10.)
    self.assertTrue(t.active)
    self.assertTrue(t.holding)
    t = self.step(model=model(speed=0.), v_ego=0., e2e_stop=True, lead_stop_distance=5.)
    self.assertFalse(t.active)
    self.assertTrue(t.requested)
    self.assertTrue(t.holding)
    self.assertEqual((t.distance, t.filtered_distance), (-1., -1.))

  def test_unknown_stop_position_with_lead_does_not_invent_separate_stop(self):
    t = self.step(model=model(brake=0.), e2e_stop=True, lead_stop_distance=10.)
    self.assertTrue(t.requested)
    self.assertTrue(t.holding)
    self.assertFalse(t.active)
    self.step(v_ego=0., lead_stop_distance=25.)
    t = self.step(model=model(brake=0.), v_ego=0., e2e_stop=True, lead_stop_distance=25.)
    self.assertTrue(t.active)
    self.assertTrue(t.holding)

  def test_stationary_target_does_not_drift_and_nearby_target_retains_its_distance(self):
    initial = self.step(v_ego=0.).distance
    for _ in range(200):
      self.assertAlmostEqual(self.step(v_ego=0.).distance, initial)
    self.target.reset()
    t = self.step(model=model(speed=1., brake=1.), v_ego=1.)
    self.assertLess(t.model_distance, 1.0)
    self.assertGreater(t.model_distance, 0.)
    self.assertAlmostEqual(t.distance, t.model_distance)
    self.assertFalse(t.holding)  # still moving

  def test_model_age_compensates_for_vehicle_travel(self):
    t = self.step(model_time=self.now - .05)
    self.assertAlmostEqual(t.distance, t.model_distance - 8. * .1)

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

  def test_near_stop_with_later_departure_retains_only_a_previously_qualified_target(self):
    prediction = slow_pass_model()
    self.assertFalse(self.step(model=prediction, v_ego=4.).active)
    self.assertTrue(self.step(model=model(speed=4., brake=1.), v_ego=4.).active)
    before, observed_at = self.target.distance, self.target.last_stop_observation
    t = self.step(model=prediction, v_ego=4.)
    self.assertTrue(t.requested)
    self.assertTrue(t.active)
    self.assertFalse(t.holding)
    self.assertAlmostEqual(t.distance, before - .2)
    self.assertEqual(t.last_stop_observation, observed_at)
    self.assertLess(t.model_distance, 0.)  # continuation did not invent a new observation

  def test_continuation_does_not_renew_its_deadline(self):
    self.step(model=model(speed=4., brake=1.), v_ego=4.)
    observed_at = self.target.last_stop_observation
    # Keep the predicted slowdown aligned with the approaching held location.
    for _ in range(39):
      t = self.step(model=slow_pass_model(distance=max(.5, self.target.distance - .2)), v_ego=4.)
      self.assertTrue(t.active)
      self.assertEqual(t.last_stop_observation, observed_at)
    self.now += .051
    t = self.step(model=slow_pass_model(distance=.5), v_ego=4.)
    self.assertFalse(t.requested)
    self.assertFalse(t.active)
    self.assertIsNone(t.last_stop_observation)
    self.assertFalse(self.step(model=slow_pass_model(distance=.5), v_ego=4.).active)

  def test_repeated_continuation_frame_cannot_outlive_deadline(self):
    self.step(model=model(speed=4., brake=1.), v_ego=4.)
    for _ in range(39):
      prediction = slow_pass_model(distance=max(.5, self.target.distance - .2))
      self.assertTrue(self.step(model=prediction, v_ego=4.).active)
    stamp = self.target.timestamp
    self.now += .051
    self.assertFalse(self.step(model=prediction, model_time=stamp, v_ego=4.).active)

  def test_continuation_requires_braking_and_near_stop_at_held_position(self):
    moving = model(speed=4., brake=0.)
    moving.action = NS(desiredAcceleration=-1.)
    isolated_dip = slow_pass_model()
    isolated_dip.velocity.x = [4.] * len(ModelConstants.T_IDXS)
    isolated_dip.velocity.x[20] = .8
    for prediction in (slow_pass_model(acceleration=.1), slow_pass_model(acceleration=-.1),
                       slow_pass_model(acceleration=math.nan), slow_pass_model(distance=20.), moving, isolated_dip):
      with self.subTest(prediction=prediction):
        self.target.reset()
        self.step(model=model(speed=4., brake=1.), v_ego=4.)
        t = self.step(model=prediction, v_ego=4.)
        self.assertFalse(t.requested)
        self.assertFalse(t.active)

  def test_continuation_preserves_lead_override_and_freshness_guards(self):
    for override in ({'eligible': False}, {'lead_distance': 5.}, {'lead_stop_distance': 5.},
                     {'model_time': 0.}, {'now': self.now + 1.}):
      with self.subTest(override=override):
        self.target.reset()
        self.step(model=model(speed=4., brake=1.), v_ego=4.)
        self.assertTrue(self.step(model=slow_pass_model(), v_ego=4.).active)
        t = self.step(model=slow_pass_model(), v_ego=4., **override)
        self.assertFalse(t.active)
        self.assertFalse(t.holding)

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
