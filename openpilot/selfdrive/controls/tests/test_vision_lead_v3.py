import unittest

import numpy as np

from openpilot.selfdrive.controls.lib.vision_lead_tracker import VisionLeadTracker, _DistanceTrack
from openpilot.selfdrive.controls.tests.test_vision_lead_tracker import lead, observation


class TestVisionLeadV3(unittest.TestCase):
  def setUp(self):
    self.tracker = VisionLeadTracker()
    self.time = 1.0

  def step(self, gap=90.0, speed=28.0, ego=30.0, std=3.0, probability=1.0, acceleration=0.0, **kwargs):
    self.time += 0.05
    obs = [observation(gap, std=std, probability=probability)] * 2
    leads = [lead(gap, speed, ego, acceleration, **kwargs)] * 2
    return self.tracker.update(obs, leads, self.time, ego), leads

  def approach(self):
    for i in range(29):
      out, original = self.step(gap=90.0 - 0.5 * i)
      if self.tracker.slots[0].mode == 'pre-closing':
        return out, original
    self.fail('Consistent rapid closing never entered pre-closing')

  def test_pre_closing_is_bounded_downward_and_keeps_maturity_gate(self):
    seen = []
    for i in range(42):
      out, original = self.step(gap=90.0 - 0.5 * i)
      track = self.tracker.slots[0]
      if track.mode == 'pre-closing':
        seen.append(track.age)
        self.assertFalse(track.ready)
        self.assertGreaterEqual(track.age, 0.7)
        self.assertLess(out[0]['vLead'], original[0]['vLead'])
        self.assertGreaterEqual(out[0]['vLead'], max(0.0, original[0]['vLead'] - 3.0))
        self.assertLessEqual(original[0]['vLead'] - out[0]['vLead'], 0.35 * (original[0]['vLead'] - track.x[1]) + 1e-8)
        self.assertEqual(out[0]['dRel'], original[0]['dRel'])
        self.assertEqual(out[0]['aLeadK'], original[0]['aLeadK'])
      if track.ready:
        self.assertGreaterEqual(track.age, 1.5)
    self.assertTrue(seen)
    self.assertLess(min(seen), 1.5)
    self.assertEqual(track.mode, 'distance')

  def test_pre_closing_cue_ramps_without_changing_kalman_learning(self):
    reference = VisionLeadTracker()
    previous_offset = 0.0
    for i in range(29):
      gap = 90.0 - 0.5 * i
      self.step(gap=gap)
      reference.update([observation(gap, std=3.0)] * 2, [lead(gap, speed=20.0, ego=30.0)] * 2, self.time, 30.0)
      track = self.tracker.slots[0]
      np.testing.assert_array_equal(track.x, reference.slots[0].x)
      np.testing.assert_array_equal(track.P, reference.slots[0].P)
      self.assertLessEqual(track.pre_closing_offset - previous_offset, 0.100001)
      previous_offset = track.pre_closing_offset

  def test_non_closing_and_low_confidence_history_do_not_activate(self):
    for rate, probability in ((0.0, 1.0), (2.0, 1.0), (-10.0, 0.9), (-1.0, 1.0)):
      with self.subTest(rate=rate, probability=probability):
        self.setUp()
        for i in range(29):
          out, original = self.step(gap=90.0 + rate * i * 0.05, probability=probability)
          self.assertEqual(out, original)

  def test_one_confident_frame_does_not_promote_weak_history(self):
    for i in range(20):
      self.step(gap=90.0 - i * 0.5, probability=0.8)
    out, original = self.step(gap=80.0, probability=1.0)
    self.assertEqual(out, original)

  def test_isolated_range_spike_does_not_create_closing_cue(self):
    for i in range(29):
      out, original = self.step(gap=86.0 if i == 18 else 90.0)
      self.assertEqual(out, original)

  def test_alternating_noisy_range_does_not_create_closing_cue(self):
    for i in range(29):
      out, original = self.step(gap=90.0 + (1.8 if i % 2 else -1.8))
      self.assertEqual(out, original)

  def test_pre_closing_never_lifts_a_slower_model_speed(self):
    self.approach()
    track = self.tracker.slots[0]
    out, original = self.step(gap=track.distance - 0.5, speed=15.0)
    self.assertEqual(out, original)

  def test_pre_closing_cannot_overshoot_a_changed_distance_target(self):
    for i in range(29):
      self.step(gap=90.0 - 0.5 * i)
    track = self.tracker.slots[0]
    self.assertGreater(track.pre_closing_offset, 0.5)
    model_speed = float(track.x[1]) + 0.2
    out, original = self.step(gap=track.distance - 0.5, speed=model_speed)
    self.assertLessEqual(original[0]['vLead'] - out[0]['vLead'], 0.35 * max(model_speed - float(track.x[1]), 0.0) + 1e-8)

  def test_pre_closing_preserves_each_hypothesis_model_baseline(self):
    seen = False
    for i in range(29):
      gap = 90.0 - 0.5 * i
      self.time += 0.05
      obs = [observation(gap, std=3.0)] * 2
      leads = [lead(gap, 28.0, 30.0), lead(gap, 24.0, 30.0)]
      out = self.tracker.update(obs, leads, self.time, 30.0)
      if self.tracker.slots[0].mode == 'pre-closing':
        seen = True
        offsets = [before['vLead'] - after['vLead'] for before, after in zip(leads, out, strict=True)]
        self.assertAlmostEqual(offsets[0], offsets[1])
        self.assertLessEqual(max(offsets), 3.0)
        self.assertGreater(out[0]['vLead'], 25.0)
    self.assertTrue(seen)

  def test_braking_and_ttc_guards_are_immediate_during_pre_closing(self):
    for speed, acceleration in ((28.0, -0.6), (10.0, 0.0)):
      with self.subTest(speed=speed, acceleration=acceleration):
        self.setUp()
        self.approach()
        gap = self.tracker.slots[0].distance - 0.5
        out, original = self.step(gap=gap, speed=speed, acceleration=acceleration)
        self.assertEqual(out, original)
        self.assertEqual(self.tracker.slots[0].mode, 'braking')
        self.assertEqual(self.tracker.slots[0].pre_closing_offset, 0.0)

  def test_cold_braking_recovery_does_not_blend_unpublished_minimum(self):
    obs = [observation(90.0, std=15.0)] * 2
    braking = [lead(90, 25.0, 30.0, -0.6), lead(90, 20.0, 30.0, -0.6)]
    recovery = [lead(90, 28.0, 30.0), lead(90, 23.0, 30.0)]
    self.assertEqual(self.tracker.update(obs, braking, 1.0, 30.0), braking)
    self.assertEqual(self.tracker.update(obs, recovery, 1.05, 30.0), recovery)
    self.assertFalse(self.tracker.slots[0].output_active)

  def test_cut_in_and_missing_lead_clear_pre_closing(self):
    for kind in ('cut-in', 'missing'):
      with self.subTest(kind=kind):
        self.setUp()
        self.approach()
        old = self.tracker.slots[0]
        if kind == 'cut-in':
          out, original = self.step(gap=40.0, speed=20.0)
          self.assertIsNot(self.tracker.slots[0], old)
        else:
          out, original = self.step(gap=old.distance, present=False)
        self.assertEqual(out, original)
        self.assertEqual(self.tracker.slots[0].pre_closing_offset, 0.0)

  def test_unsupported_paths_remain_original(self):
    for ego, gap, radar in ((15.0, 90.0, False), (30.0, 9.0, False), (30.0, 90.0, True)):
      with self.subTest(ego=ego, gap=gap, radar=radar):
        self.setUp()
        for _ in range(60):
          out, original = self.step(gap=gap, ego=ego, radar=radar)
          self.assertEqual(out, original)

  def test_range_state_reanchors_on_every_bypass(self):
    track = _DistanceTrack(observation(60.0), 1.0, 30.0)
    track.dt = 0.05
    self.assertEqual(track.filtered_range(60.0, 30.0, 30.0, True), 60.0)
    self.assertAlmostEqual(track.filtered_range(61.0, 30.0, 30.0, True), 60.6)
    self.assertEqual(track.filtered_range(55.0, 25.0, 30.0, False), 55.0)
    self.assertEqual(track.filtered_range(54.0, 25.0, 30.0, True), 54.0)

  def test_missing_observation_discards_range_output_but_keeps_private_track(self):
    for _ in range(120):
      self.step(gap=60.0, speed=30.0)
    old = self.tracker.slots[0]
    self.tracker.update([observation(probability=0.0)] * 2, [{'present': False}] * 2, self.time + 0.05, 30.0)
    self.assertFalse(old.range_active)
    self.assertFalse(old.output_active)
    self.time += 0.05
    out, _ = self.step(gap=58.0, speed=30.0)
    self.assertIs(self.tracker.slots[0], old)
    self.assertEqual(out[0]['dRel'], 58.0)

  def test_shared_range_updates_once_and_preserves_slot_offsets(self):
    for _ in range(120):
      self.step(gap=60.0, speed=30.0)
    obs = [observation(60.7), observation(61.3)]
    leads = [lead(60.7, 30.0, 30.0), lead(61.3, 30.0, 30.0)]
    out = self.tracker.update(obs, leads, self.time + 0.05, 30.0)
    track = self.tracker.slots[0]
    self.assertIs(track, self.tracker.slots[1])
    predicted = 60.0 + (out[0]['vLead'] - 30.0) * 0.05
    self.assertAlmostEqual(track.filtered_distance, predicted + 0.6 * (61.0 - predicted))
    self.assertAlmostEqual(out[1]['dRel'] - out[0]['dRel'], 0.6)


if __name__ == '__main__':
  unittest.main()
