import math
import unittest

import numpy as np

from openpilot.selfdrive.controls.lib.vision_lead_tracker import VisionLeadObservation, VisionLeadTracker


def observation(distance=60.0, lateral=0.0, std=1.0, probability=1.0):
  return VisionLeadObservation(distance, lateral, std, probability)


def lead(distance=60.0, speed=24.0, ego=25.0, acceleration=0.0, present=True, radar=False):
  return {
    'dRel': distance,
    'yRel': 0.0,
    'vLead': speed,
    'vLeadK': speed,
    'vRel': speed - ego,
    'aLeadK': acceleration,
    'aLeadTau': 0.3,
    'modelProb': 1.0,
    'present': present,
    'radar': radar,
    'radarTrackId': -1,
  }


class TestVisionLeadTracker(unittest.TestCase):
  def setUp(self):
    self.tracker = VisionLeadTracker()
    self.time = 1.0

  def step(self, obs=None, leads=None, ego=25.0, count=1, valid=True):
    obs = obs if obs is not None else [observation(), observation(90.0)]
    leads = leads if leads is not None else [lead(), lead(90.0)]
    for _ in range(count):
      self.time += 0.05
      output = self.tracker.update(obs, leads, self.time, ego, valid)
    return output

  def test_private_history_never_creates_lead(self):
    output = self.step([observation(probability=0.4)] * 2, [{'present': False}] * 2, count=120)
    self.assertEqual(output, [{'present': False}] * 2)
    self.assertTrue(self.tracker.slots[0].ready)

  def test_prewarmed_track_is_ready_at_first_acceptance(self):
    self.step([observation(probability=0.4)] * 2, [{'present': False}] * 2, count=120)
    output = self.step([observation()] * 2, [lead(speed=22.0)] * 2)
    self.assertAlmostEqual(output[0]['vLead'], 25.0, delta=0.02)

  def test_cold_acquisition_uses_original(self):
    original = [lead(speed=22.0), lead(90.0)]
    self.assertEqual(self.step(leads=original), original)

  def test_constant_distance_removes_speed_bias(self):
    output = self.step(count=200)
    self.assertAlmostEqual(output[0]['vLead'], 25.0, delta=0.02)
    self.assertAlmostEqual(output[0]['vRel'], 0.0, delta=0.02)
    self.assertEqual(output[0]['vLead'], output[0]['vLeadK'])

  def test_established_state_is_independent_of_model_speed(self):
    a, b = VisionLeadTracker(), VisionLeadTracker()
    for i in range(200):
      oa = a.update([observation()] * 2, [lead(speed=22.0)] * 2, 1 + i * 0.05, 25.0)
      ob = b.update([observation()] * 2, [lead(speed=28.0)] * 2, 1 + i * 0.05, 25.0)
    np.testing.assert_allclose(a.slots[0].x, b.slots[0].x)
    self.assertAlmostEqual(oa[0]['vLead'], ob[0]['vLead'], places=8)
    self.assertGreater(oa[0]['vLead'] - 22.0, 1.5)

  def test_ego_acceleration_is_compensated(self):
    gap, previous_ego = 100.0, 20.0
    for i in range(200):
      ego = 20 + i * 0.025
      gap += (25.0 - 0.5 * (ego + previous_ego)) * 0.05
      output = self.step([observation(gap)] * 2, [lead(gap, speed=25.0, ego=ego)] * 2, ego=ego)
      previous_ego = ego
    self.assertAlmostEqual(output[0]['vLead'], 25.0, delta=0.03)

  def test_increasing_gap_measures_faster_lead(self):
    for i in range(200):
      gap = 60 + i * 0.1
      output = self.step([observation(gap)] * 2, [lead(gap)] * 2)
    self.assertAlmostEqual(output[0]['vLead'], 27.0, delta=0.03)

  def test_duplicate_hypotheses_share_speed_without_double_counting(self):
    single = VisionLeadTracker()
    for _ in range(160):
      self.step([observation()] * 2, [lead(speed=23), lead(speed=27)])
      single.update([observation(), observation(probability=0)], [lead(), {'present': False}], self.time, 25.0)
    self.assertIs(self.tracker.slots[0], self.tracker.slots[1])
    np.testing.assert_allclose(self.tracker.slots[0].P, single.slots[0].P)
    out = self.step([observation()] * 2, [lead(speed=23), lead(speed=27)])
    self.assertEqual(out[0]['vLead'], out[1]['vLead'])

  def test_distinct_cars_have_distinct_speeds(self):
    for i in range(180):
      d = 90 + i * 0.1
      out = self.step([observation(), observation(d)], [lead(), lead(d)])
    self.assertIsNot(self.tracker.slots[0], self.tracker.slots[1])
    self.assertAlmostEqual(out[0]['vLead'], 25, delta=0.03)
    self.assertAlmostEqual(out[1]['vLead'], 27, delta=0.03)

  def test_slot_swap_preserves_object_history(self):
    self.step(count=120)
    tracks = list(self.tracker.slots)
    self.step([observation(90), observation()], [lead(90), lead()])
    self.assertIs(self.tracker.slots[0], tracks[1])
    self.assertIs(self.tracker.slots[1], tracks[0])
    self.assertTrue(all(t.ready for t in self.tracker.slots))

  def test_split_does_not_copy_history_to_new_vehicle(self):
    self.step([observation()] * 2, [lead()] * 2, count=120)
    old = self.tracker.slots[0]
    self.step([observation(), observation(90)], [lead(), lead(90)])
    self.assertIs(self.tracker.slots[0], old)
    self.assertFalse(self.tracker.slots[1].ready)
    self.step([observation()] * 2, [lead()] * 2)
    self.assertIs(self.tracker.slots[0], self.tracker.slots[1])
    self.assertEqual(len(self.tracker.tracks), 1)

  def test_brief_duplicate_separation_keeps_warmed_speed(self):
    self.step([observation(std=10)] * 2, [lead(speed=23)] * 2, count=160)
    shared = self.tracker.slots[0]
    self.assertTrue(shared.ready)
    # Route 26c/12: 2.4 m longitudinal / 0.44 m lateral separation split a warm track.
    for distance, lateral in ((57.6, 0.44), (58.2, 0.32), (58.3, 0.28), (60.0, 0.0)):
      original = [lead(distance, speed=23), lead(speed=23)]
      out = self.step([observation(distance, lateral=lateral, std=10), observation(std=10)], original)
      self.assertIs(self.tracker.slots[0], shared)
      self.assertIs(self.tracker.slots[1], shared)
      self.assertGreater(out[0]['vLead'], 24.8)
      self.assertEqual(out[0]['vLead'], out[1]['vLead'])
      for before, after in zip(original, out, strict=True):
        for key in before.keys() - {'vLead', 'vLeadK', 'vRel'}:
          self.assertEqual(after[key], before[key])

  def test_group_hysteresis_does_not_merge_new_distinct_tracks(self):
    self.step([observation(), observation(62)], [lead(), lead(62)], count=160)
    self.assertIsNot(self.tracker.slots[0], self.tracker.slots[1])
    self.assertTrue(all(track.ready for track in self.tracker.slots))

  def test_group_hysteresis_releases_distance_and_lateral_splits(self):
    for distance, lateral in ((56.9, 0.0), (57.6, 0.51)):
      with self.subTest(distance=distance, lateral=lateral):
        self.tracker.reset()
        self.step([observation()] * 2, [lead()] * 2, count=120)
        shared = self.tracker.slots[0]
        original = [lead(distance, speed=20), lead()]
        out = self.step([observation(distance, lateral=lateral), observation()], original)
        self.assertEqual(out[0], original[0])
        self.assertIsNot(self.tracker.slots[0], shared)
        self.assertFalse(self.tracker.slots[0].ready)
        self.assertIs(self.tracker.slots[1], shared)

  def test_group_hysteresis_requires_both_observations_to_match(self):
    self.step([observation()] * 2, [lead()] * 2, count=120)
    shared = self.tracker.slots[0]
    original = [lead(52, speed=20), lead(54)]
    out = self.step([observation(52), observation(54)], original)
    self.assertEqual(out, original)
    self.assertNotIn(shared, self.tracker.slots)
    self.assertIsNot(self.tracker.slots[0], self.tracker.slots[1])

  def test_braking_guard_is_immediate_during_retained_group(self):
    for speed, acceleration in ((20, -1), (13, 0)):
      with self.subTest(speed=speed, acceleration=acceleration):
        self.tracker.reset()
        self.step([observation()] * 2, [lead()] * 2, count=120)
        shared = self.tracker.slots[0]
        out = self.step([observation(57.6, lateral=0.44), observation()],
                        [lead(57.6, speed=speed, acceleration=acceleration), lead()])
        self.assertIs(self.tracker.slots[0], shared)
        self.assertIs(self.tracker.slots[1], shared)
        self.assertLessEqual(out[0]['vLead'], speed)
        self.assertLessEqual(out[1]['vLead'], speed)

  def test_confidence_dip_keeps_history(self):
    self.step(count=120)
    old = self.tracker.slots[0]
    self.step([observation(probability=0.7), observation(90)], count=10)
    self.assertIs(self.tracker.slots[0], old)
    self.assertTrue(old.ready)

  def test_loss_never_publishes_stale_lead(self):
    self.step(count=120)
    out = self.step([observation(probability=0)] * 2, [{'present': False}] * 2)
    self.assertEqual(out, [{'present': False}] * 2)
    self.assertEqual(self.tracker.slots, [None, None])

  def test_cut_in_discards_old_speed(self):
    self.step(count=120)
    original = [lead(35, speed=20), lead(90)]
    out = self.step([observation(35), observation(90)], original)
    self.assertEqual(out[0], original[0])
    self.assertFalse(self.tracker.slots[0].ready)

  def test_braking_guard_acts_immediately_and_keeps_learning(self):
    self.step(count=120)
    old = self.tracker.slots[0]
    out = self.step(leads=[lead(speed=22, acceleration=-1), lead(90)])
    self.assertLessEqual(out[0]['vLead'], 22)
    self.assertIs(self.tracker.slots[0], old)
    self.assertTrue(old.ready)

  def test_short_time_to_collision_is_not_masked(self):
    self.step([observation(20)] * 2, [lead(20)] * 2, count=120)
    out = self.step([observation(20)] * 2, [lead(20, speed=20)] * 2)
    self.assertLessEqual(out[0]['vLead'], 20)

  def test_braking_recovery_does_not_jump_to_lagging_speed(self):
    gap, previous_speed = 80.0, 25.0
    recovery_error = []
    for i in range(400):
      t = i * 0.05
      speed = 25 - 3 * min(max(t - 10, 0), 2)
      accel = -3.0 if 10 <= t < 12 else 0.0
      gap += (0.5 * (speed + previous_speed) - 25) * 0.05
      out = self.step([observation(gap)] * 2, [lead(gap, speed=speed, acceleration=accel)] * 2)
      if accel < 0:
        self.assertLessEqual(out[0]['vLead'], speed + 1e-8)
      if t >= 12:
        recovery_error.append(out[0]['vLead'] - speed)
      previous_speed = speed
    self.assertLess(max(recovery_error), 0.6)

  def test_brief_gap_retains_private_history(self):
    self.step(count=120)
    old = self.tracker.slots[0]
    self.step([observation(probability=0), observation(90)], count=2)
    self.step()
    self.assertIs(self.tracker.slots[0], old)

  def test_long_gap_discards_private_history(self):
    self.step(count=120)
    old = self.tracker.slots[0]
    self.step([observation(probability=0), observation(90)], count=5)
    self.step()
    self.assertIsNot(self.tracker.slots[0], old)
    self.assertFalse(self.tracker.slots[0].ready)

  def test_rejected_shared_track_is_not_kept_by_missing_slot(self):
    self.step([observation()] * 2, [lead()] * 2, count=120)
    old = self.tracker.slots[0]
    # A sequence within the association window can still fail the innovation test.
    old.x[0] = 100.0
    self.step([observation(), observation(probability=0)], [lead(), {'present': False}])
    self.assertNotIn(old, self.tracker.tracks)
    self.assertNotIn(old, self.tracker.history_slots)

  def test_radar_low_speed_and_close_range_are_original(self):
    for ego, gap, radar in [(25.0, 60.0, True), (10.0, 60.0, False), (25.0, 8.0, False)]:
      with self.subTest(ego=ego, gap=gap, radar=radar):
        self.tracker.reset()
        original = [lead(gap, ego=ego, radar=radar)] * 2
        out = self.step([observation(gap)] * 2, original, ego, count=120)
        self.assertEqual(out, original)

  def test_invalid_input_and_bad_time_reset(self):
    for timestamp, ego, valid in [(math.nan, 25.0, True), (1.0, math.nan, True), (1.0, 25.0, False), (1.0, 25.0, True), (100.0, 25.0, True)]:
      self.step(count=120)
      original = [lead()] * 2
      out = self.tracker.update([observation()] * 2, original, timestamp, ego, valid)
      self.assertEqual(out, original)
      self.assertEqual(self.tracker.tracks, [])

  def test_invalid_uncertainty_uses_original(self):
    for std in [math.nan, 0.0, -1.0, 51.0]:
      self.step(count=120)
      original = [lead()] * 2
      self.assertEqual(self.step([observation(std=std)] * 2, original), original)

  def test_covariance_stays_finite_symmetric_and_positive(self):
    rng = np.random.default_rng(3)
    for _ in range(600):
      self.step([observation(60 + rng.normal(0, 0.2))] * 2, [lead()] * 2)
      cov = self.tracker.slots[0].P
      self.assertTrue(np.isfinite(cov).all())
      np.testing.assert_allclose(cov, cov.T, atol=1e-12)
      self.assertGreaterEqual(np.linalg.eigvalsh(cov).min(), -1e-10)


if __name__ == '__main__':
  unittest.main()
