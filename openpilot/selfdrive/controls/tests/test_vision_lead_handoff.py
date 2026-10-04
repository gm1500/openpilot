import unittest

from openpilot.selfdrive.controls.lib.vision_lead_tracker import VisionLeadObservation, VisionLeadTracker


class TestVisionLeadHandoff(unittest.TestCase):
  def run_track(self, distance, velocity=29., accel=0., probability=.99, ego=30.):
    tracker = VisionLeadTracker()
    results = []
    for i in range(100):
      t = .05 * i
      gap = distance(t)
      speed = velocity(t) if callable(velocity) else velocity
      a = accel(t) if callable(accel) else accel
      observation = VisionLeadObservation(gap, 0., 5., probability)
      raw = {'present': True, 'radar': False, 'dRel': gap, 'vLead': speed, 'vLeadK': speed, 'vRel': speed-ego, 'aLeadK': a}
      output = tracker.update([observation, observation], [raw, dict(raw)], t, ego)
      results.append((t, gap, output, tracker.slots[0].age))
    return results

  def test_cut_in_reanchors_without_hiding_current_range(self):
    rows = self.run_track(lambda t: 85. - 15. * min(t, 2.))
    resets = [r for r in rows[1:] if r[3] == 0.]
    self.assertTrue(resets)
    for _, gap, outputs, _ in resets:
      for lead in outputs:
        self.assertEqual(lead['dRel'], gap)
        self.assertEqual(lead['vLead'], 29.)
    self.assertGreater(min(r[2][0]['vLead'] for r in rows), 25.)
    self.assertGreater(rows[-1][3], 1.5)  # stable replacement reacquires distance mode

  def test_consistent_closing_is_not_a_target_change(self):
    rows = self.run_track(lambda t: 85. - 15. * t, velocity=15.)
    self.assertFalse(any(r[3] == 0. for r in rows[1:]))

  def test_short_gap_does_not_reset_from_model_disagreement(self):
    rows = self.run_track(lambda t: 40. - 5. * t)
    self.assertFalse(any(r[3] == 0. for r in rows[1:]))

  def test_model_braking_blocks_handoff_reset(self):
    rows = self.run_track(lambda t: 85. - 10. * t, accel=-.6)
    self.assertFalse(any(r[3] == 0. for r in rows[1:]))

  def test_stopped_lead_after_reset_gets_immediate_model_constraint(self):
    rows = self.run_track(lambda t: 85. - 15. * min(t, 2.), velocity=lambda t: 29. if t < 1. else 0.)
    self.assertTrue(any(r[3] == 0. for r in rows[1:20]))
    for t, gap, outputs, _ in rows:
      if t >= 1.:
        for lead in outputs:
          self.assertEqual(lead['vLead'], 0.)
          self.assertAlmostEqual(lead['dRel'], gap)

  def test_unsupported_or_missing_lead_is_not_created(self):
    tracker = VisionLeadTracker()
    obs = VisionLeadObservation(60., 0., 5., .99)
    missing = [{'present': False}] * 2
    for i in range(50):
      self.assertEqual(tracker.update([obs, obs], missing, i*.05, 30.), missing)
    raw = [{'present': True, 'radar': True, 'dRel': 60., 'vLead': 20., 'aLeadK': 0.}] * 2
    self.assertEqual(tracker.update([obs, obs], raw, 2.5, 30.), raw)


if __name__ == '__main__':
  unittest.main()
