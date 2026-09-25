import unittest

import numpy as np

from openpilot.selfdrive.controls.lib.vision_lead_tracker import VisionLeadTracker
from openpilot.selfdrive.controls.tests.test_vision_lead_tracker import lead, observation


class TestVisionLeadContinuity(unittest.TestCase):
  EGO = 31.4

  def setUp(self):
    self.tracker = VisionLeadTracker()
    self.time = 1.0

  def step(self, obs, leads, dt=0.05):
    self.time += dt
    return self.tracker.update(obs, leads, self.time, self.EGO)

  def warm(self, std=2.5, count=400):
    for _ in range(count):
      self.step([observation(45.5, std=std)] * 2, [lead(45.5, speed=30, ego=self.EGO)] * 2)
    self.step([observation(45.5, std=8.8)] * 2, [lead(45.5, speed=30, ego=self.EGO)] * 2)
    return self.tracker.slots[0]

  def shock(self, distance=34.55, lateral=0.06, std=19.3, probability=0.995, speed=27.9, acceleration=0.0):
    obs = [observation(distance, lateral, std, probability), observation(42.62, 1.8, 28.46, 0.98)]
    leads = [lead(distance, speed, self.EGO, acceleration), lead(42.62, 30.66, self.EGO)]
    return obs, leads

  def test_uncertain_primary_handoff_keeps_fresh_private_state(self):
    old = self.warm()
    before = old.output
    obs, leads = self.shock()
    out = self.step(obs, leads)
    fresh = self.tracker.slots[0]
    self.assertIsNot(fresh, old)
    self.assertNotIn(old, self.tracker.tracks)
    self.assertEqual(fresh.age, 0.0)
    self.assertEqual(fresh.P[1, 1], 1000.0)
    self.assertFalse(fresh.ready)
    self.assertAlmostEqual(out[0]['vLead'], before)
    self.assertEqual(out[1], leads[1])
    for original, changed in zip(leads, out, strict=True):
      for key in original.keys() - {'vLead', 'vLeadK', 'vRel'}:
        self.assertEqual(changed[key], original[key])
    self.assertAlmostEqual(out[0]['vRel'], out[0]['vLead'] - self.EGO)

  def test_uncertain_slower_cut_in_reaches_model_within_one_second(self):
    old = self.warm()
    before = old.output
    obs, leads = self.shock()
    speeds = [self.step(obs, leads)[0]['vLead']]
    for i in range(1, 62):
      obs, leads = self.shock(distance=34.55 - 3.5 * i * 0.05)
      speeds.append(self.step(obs, leads)[0]['vLead'])
    self.assertEqual(speeds[0], before)
    self.assertTrue(all(27.9 - 1e-9 <= v <= before for v in speeds))
    np.testing.assert_allclose(speeds[20:], 27.9, atol=1e-8)
    self.assertLess(max(abs(np.diff(speeds))), 0.2)

  def test_changing_model_output_cannot_overshoot_handoff_endpoints(self):
    before = self.warm().output
    for i, speed in enumerate((27.9, 28.3, 27.6, 29.2, 30.2, 29.6)):
      obs, leads = self.shock(distance=34.55 - i * 0.1, speed=speed)
      value = self.step(obs, leads)[0]['vLead']
      self.assertGreaterEqual(value, min(before, speed))
      self.assertLessEqual(value, max(before, speed))

  def test_repeated_range_resets_cannot_renew_handoff(self):
    self.warm()
    self.step(*self.shock())
    obs, leads = self.shock(distance=41.0, std=45.0)
    self.assertEqual(self.step(obs, leads), leads)
    self.assertFalse(self.tracker.slots[0].ready)

  def test_braking_stopped_and_closing_leads_bypass_handoff(self):
    for speed, acceleration in ((27.9, -0.6), (0.0, 0.0), (27.0, 0.0)):
      with self.subTest(speed=speed, acceleration=acceleration):
        self.setUp()
        self.warm()
        obs, leads = self.shock(speed=speed, acceleration=acceleration)
        self.assertEqual(self.step(obs, leads), leads)

  def test_new_braking_or_closing_evidence_cancels_active_handoff(self):
    for speed, acceleration in ((27.9, -0.6), (26.0, 0.0), (0.0, 0.0)):
      with self.subTest(speed=speed, acceleration=acceleration):
        self.setUp()
        self.warm()
        self.step(*self.shock())
        obs, leads = self.shock(distance=34.4, speed=speed, acceleration=acceleration)
        # The 26 m/s case has a closing time between five and eight seconds.
        self.assertEqual(self.step(obs, leads)[0], leads[0])

  def test_confident_cut_in_and_large_lateral_or_range_change_are_original(self):
    cases = [dict(std=2.5), dict(lateral=0.3), dict(distance=29.0, speed=self.EGO), dict(probability=0.89)]
    for changes in cases:
      with self.subTest(changes=changes):
        self.setUp()
        self.warm()
        obs, leads = self.shock(**changes)
        self.assertEqual(self.step(obs, leads), leads)

  def test_high_uncertainty_alone_does_not_enable_handoff(self):
    self.warm(std=19.3)
    self.step([observation(45.5, std=19.3)] * 2, [lead(45.5, speed=30, ego=self.EGO)] * 2)
    obs, leads = self.shock()
    self.assertEqual(self.step(obs, leads), leads)

  def test_ordinary_match_keeps_history_and_cannot_lend_it_to_new_car(self):
    old = self.warm()
    obs, leads = self.shock()
    obs[1] = observation(45.5, std=8.8)
    leads[1] = lead(45.5, speed=30, ego=self.EGO)
    out = self.step(obs, leads)
    self.assertEqual(out[0], leads[0])
    self.assertIs(self.tracker.slots[1], old)

  def test_two_plausible_recipients_are_ambiguous(self):
    self.warm()
    obs = [observation(34.55, 0.02, 19.3), observation(56.45, 0.04, 19.3)]
    leads = [lead(34.55, speed=30, ego=self.EGO), lead(56.45, speed=30, ego=self.EGO)]
    self.assertEqual(self.step(obs, leads), leads)

  def test_unassigned_track_cannot_handoff_into_another_previous_slot(self):
    for _ in range(400):
      self.step([observation(45.5), observation(80)],
                [lead(45.5, speed=30, ego=self.EGO), lead(80, speed=30, ego=self.EGO)])
    self.step([observation(45.5, std=8.8), observation(80)],
              [lead(45.5, speed=30, ego=self.EGO), lead(80, speed=30, ego=self.EGO)])
    obs = [observation(80), observation(34.55, 0.06, 19.3)]
    leads = [lead(80, speed=30, ego=self.EGO), lead(34.55, speed=27.9, ego=self.EGO)]
    self.assertEqual(self.step(obs, leads)[1], leads[1])

  def test_immature_track_does_not_handoff(self):
    self.warm(count=70)
    obs, leads = self.shock()
    self.assertEqual(self.step(obs, leads), leads)

  def test_missing_radar_and_unsupported_leads_do_not_handoff(self):
    for kind in ('missing', 'radar', 'close', 'far', 'nonfinite'):
      with self.subTest(kind=kind):
        self.setUp()
        self.warm()
        obs, leads = self.shock()
        if kind == 'missing':
          leads[0] = {'present': False}
        elif kind == 'radar':
          leads[0]['radar'] = True
        elif kind == 'close':
          leads[0]['dRel'] = 9.0
        elif kind == 'far':
          leads[0]['dRel'] = 151.0
        else:
          leads[0]['aLeadK'] = float('inf')
        self.assertEqual(self.step(obs, leads), leads)

  def test_invalid_old_state_or_stale_observation_cannot_handoff(self):
    for kind in ('position', 'covariance', 'innovation', 'stale'):
      with self.subTest(kind=kind):
        self.setUp()
        old = self.warm()
        if kind == 'position':
          old.x[0] = float('nan')
        elif kind == 'covariance':
          old.P[0, 0] = float('nan')
        elif kind == 'innovation':
          old.x[0] = 90.0
        obs, leads = self.shock()
        self.assertEqual(self.step(obs, leads, dt=0.15 if kind == 'stale' else 0.05), leads)


if __name__ == '__main__':
  unittest.main()
