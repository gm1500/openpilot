import math
import unittest

from openpilot.selfdrive.controls.lib.vision_lead_tracker import VisionLeadObservation, VisionLeadTracker


class TestLeadContinuity(unittest.TestCase):
  def setUp(self):
    self.tracker = VisionLeadTracker()
    self.time = -0.05

  def step(self, distance, lateral=0., ego=30., speed=29., probability=.99, std=5., yaw=0.,
           second=None, single=False, acceleration=0., valid=True, dt=.05):
    self.time += dt
    a = VisionLeadObservation(distance, lateral, std, probability)
    b = second if second is not None else a
    if single:
      b = VisionLeadObservation(0., 0., 5., 0.)
    observations = [a, b]
    raw = [dict(present=True, radar=False, dRel=o.distance, yRel=o.lateral, vLead=speed, vLeadK=speed,
                vRel=speed - ego, aLeadK=acceleration) if o.valid() and o.probability > .5 else dict(present=False)
           for o in observations]
    out = self.tracker.update(observations, raw, self.time, ego, valid, yaw_rate=yaw)
    for original, lead in zip(raw, out, strict=True):
      self.assertEqual(original['present'], lead['present'])
      if original['present']:
        self.assertEqual(original['aLeadK'], lead['aLeadK'])
    return out

  def test_far_single_slower_lead_keeps_history_despite_model_disagreement(self):
    for single in (False, True):
      with self.subTest(single=single):
        self.setUp()
        first = None
        for i in range(100):
          out = self.step(145. - 8. * i * .05, single=single)
          track = self.tracker.slots[0]
          if first is None:
            first = track
          self.assertIs(track, first)
          self.assertAlmostEqual(track.age, i * .05)
        self.assertTrue(track.continuity.continuous)
        self.assertLess(out[0]['vLead'], 23.)

  def test_noisy_120_to_80_approach_gets_bounded_early_range_cue(self):
    previous = 0.
    cue = []
    for i in range(70):
      out = self.step(120. - 40. / 3.6 * i * .05 + .15 * math.sin(i), ego=120. / 3.6, speed=112. / 3.6, std=12.)
      track = self.tracker.slots[0]
      self.assertLessEqual(track.single_closing_offset, 6.)
      self.assertLessEqual(track.single_closing_offset - previous, 2.5 * .05 + 1e-8)
      previous = track.single_closing_offset
      cue.append(previous)
      if i < 16:
        self.assertEqual(previous, 0.)
    self.assertGreater(max(cue), 4.)
    self.assertLess(out[0]['vLead'] * 3.6, 95.)

  def test_distinct_hypotheses_do_not_qualify_single_target(self):
    for i in range(80):
      other = VisionLeadObservation(100., 3., 5., .99)
      self.step(145. - 8. * i * .05, second=other)
      for track in self.tracker.slots:
        self.assertFalse(track.continuity.continuous)
        self.assertEqual(track.single_closing_offset, 0.)

  def test_slot_swap_keeps_two_separate_histories(self):
    for i in range(80):
      other = VisionLeadObservation(80., 3., 5., .99)
      self.step(100., second=other, speed=30.)
    first, second = self.tracker.slots
    self.step(80., lateral=3., second=VisionLeadObservation(100., 0., 5., .99), speed=30.)
    self.assertIs(self.tracker.slots[0], second)
    self.assertIs(self.tracker.slots[1], first)

  def test_new_slower_cut_in_does_not_inherit_old_speed(self):
    for i in range(80):
      self.step(145. - 8. * i * .05)
    old = self.tracker.slots[0]
    out = self.step(45., lateral=1., speed=12., acceleration=-1.)
    self.assertIsNot(self.tracker.slots[0], old)
    self.assertEqual(self.tracker.slots[0].age, 0.)
    self.assertEqual(out[0]['vLead'], 12.)
    self.assertEqual(out[0]['dRel'], 45.)
    self.assertEqual(self.tracker.slots[0].single_closing_offset, 0.)

  def test_cut_out_does_not_borrow_previous_slow_history(self):
    for i in range(80):
      self.step(100. - 8. * i * .05, speed=22.)
    old = self.tracker.slots[0]
    out = self.step(130., speed=32.)
    self.assertIsNot(self.tracker.slots[0], old)
    self.assertEqual(out[0]['vLead'], 32.)
    self.assertEqual(self.tracker.slots[0].single_closing_offset, 0.)

  def test_gradual_lateral_cut_in_does_not_protect_disagreement(self):
    resets = 0
    for i in range(80):
      self.step(145. - 8. * min(i * .05, 2.), lateral=3. - .8 * min(i * .05, 2.))
      track = self.tracker.slots[0]
      resets += i > 0 and track.age == 0.
      if i < 40:
        self.assertFalse(track.continuity.continuous)
    self.assertGreater(resets, 0)

  def test_yaw_compensated_motion_and_missing_yaw(self):
    for yaw in (-.02, .02):
      with self.subTest(yaw=yaw):
        self.setUp()
        lateral = 0.
        for i in range(35):
          distance = 100. - 5. * i * .05
          if i:
            lateral += yaw * (distance + .125) * .05
          self.step(distance, lateral=lateral, yaw=yaw, speed=25.)
        self.assertTrue(self.tracker.slots[0].continuity.continuous)
        self.step(distance, lateral=lateral, yaw=None)
        self.assertFalse(self.tracker.slots[0].continuity.continuous)
        self.assertEqual(self.tracker.slots[0].single_closing_offset, 0.)

  def test_noise_low_confidence_and_opening_do_not_make_closing_cue(self):
    for scenario in ('noise', 'confidence', 'opening', 'steady'):
      with self.subTest(scenario=scenario):
        self.setUp()
        for i in range(80):
          distance = 100. - 5. * i * .05
          if scenario == 'noise':
            distance = 100. + 2.2 * (-1)**i
          elif scenario == 'opening':
            distance = 100. + 3. * i * .05
          elif scenario == 'steady':
            distance = 100.
          self.step(distance, probability=.8 if scenario == 'confidence' else .99)
          self.assertEqual(self.tracker.slots[0].single_closing_offset, 0.)

  def test_gap_and_invalid_data_cannot_reuse_continuity(self):
    for i in range(40):
      self.step(120. - 8. * i * .05)
    self.assertTrue(self.tracker.slots[0].continuity.continuous)
    self.step(103., dt=.15)
    self.assertFalse(self.tracker.slots[0].continuity.continuous)
    self.step(102., valid=False)
    self.assertEqual(self.tracker.slots, [None, None])

  def test_model_braking_still_wins(self):
    for i in range(40):
      self.step(120. - 8. * i * .05)
    out = self.step(104., speed=10., acceleration=-2.)
    self.assertEqual(out[0]['vLead'], 10.)
    self.assertEqual(out[0]['aLeadK'], -2.)
    self.assertEqual(self.tracker.slots[0].single_closing_offset, 0.)

  def test_inactive_cue_does_not_cap_established_speed_to_model(self):
    for _ in range(100):
      out = self.step(90., speed=29.)
    self.assertEqual(self.tracker.slots[0].single_closing_offset, 0.)
    self.assertGreater(out[0]['vLead'], 29.8)


if __name__ == '__main__':
  unittest.main()
