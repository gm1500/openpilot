import unittest

import numpy as np

from openpilot.selfdrive.controls.lib.vision_lead_kalman import VisionLeadKalman


def make_lead(distance=40.0, ego=25.0, speed=24.25, acceleration=0.0):
  return {'dRel': distance, 'yRel': 0.0, 'vRel': speed-ego, 'vLead': speed, 'vLeadK': speed,
          'aLeadK': acceleration, 'aLeadTau': 0.3, 'modelProb': 1.0, 'present': True, 'radar': False, 'radarTrackId': -1}


class TestVisionLeadKalman(unittest.TestCase):
  def setUp(self):
    self.filt = VisionLeadKalman()

  def establish(self, seconds=12.0):
    out = None
    for t in np.arange(0, seconds, 0.05):
      out = self.filt.update(make_lead(), float(t), 25.0, 1.0, 2.0)
    return out

  def test_constant_gap_removes_model_speed_offset(self):
    out = self.establish()
    self.assertTrue(self.filt.active)
    self.assertAlmostEqual(out['vLead'], 25.0, delta=0.02)
    self.assertAlmostEqual(out['vRel'], 0.0, delta=0.02)
    self.assertEqual(out['vLead'], out['vLeadK'])

  def test_ego_motion_is_not_mistaken_for_lead_acceleration(self):
    # A constant-speed lead while ego accelerates by 2.4 m/s, inside the guards.
    for t in np.arange(0, 12, 0.05):
      ego = 24.0 + 0.2*t
      distance = 60.0 + t - 0.1*t*t
      out = self.filt.update(make_lead(distance, ego, 24.25), float(t), ego, 1.0, 2.0)
    self.assertTrue(self.filt.active)
    self.assertAlmostEqual(out['vLead'], 25.0, delta=0.04)
    self.assertAlmostEqual(out['vRel'], 25.0-ego, delta=0.04)

  def test_opening_gap_uses_positive_relative_speed(self):
    for t in np.arange(0, 12, 0.05):
      out = self.filt.update(make_lead(40.0+0.8*t, speed=25.0), float(t), 25.0, 1.0, 2.0)
    self.assertAlmostEqual(out['vRel'], 0.8, delta=0.02)

  def test_noisy_distance_does_not_become_raw_derivative(self):
    rng = np.random.default_rng(1)
    speeds, derivatives = [], []
    previous = 40.0
    for t in np.arange(0, 30, 0.05):
      distance = 40.0 + rng.normal(0.0, 0.6)
      out = self.filt.update(make_lead(distance), float(t), 25.0, 1.0, 2.0)
      if t > 5:
        speeds.append(out['vLead'])
        derivatives.append((distance-previous)/0.05)
      previous = distance
    self.assertLess(np.std(speeds), 0.5)
    self.assertLess(np.std(speeds), 0.1*np.std(derivatives))
    self.assertAlmostEqual(np.mean(speeds), 25.0, delta=0.15)

  def test_warmup_returns_original_lead(self):
    for t in np.arange(0, 2.0, 0.05):
      lead = make_lead()
      self.assertIs(self.filt.update(lead, float(t), 25.0, 1.0, 2.0), lead)

  def test_distance_acceleration_and_flags_are_preserved(self):
    self.establish()
    lead = make_lead(40.2, acceleration=0.2)
    before = lead.copy()
    out = self.filt.update(lead, 12.0, 25.0, 1.0, 2.0)
    self.assertEqual(lead, before)
    for k in lead:
      if k not in ('vRel', 'vLead', 'vLeadK'):
        self.assertEqual(out[k], lead[k])

  def test_both_directions_of_correction_are_bounded(self):
    for speed in (22.6, 29.0):
      self.filt.reset()
      for t in np.arange(0, 10, 0.05):
        lead = make_lead(speed=speed)
        out = self.filt.update(lead, float(t), 25.0, 1.0, 2.0)
        self.assertLessEqual(abs(out['vLead']-speed), self.filt.MAX_CORRECTION+1e-10)

  def test_braking_acceleration_immediately_returns_model(self):
    self.establish()
    lead = make_lead(acceleration=-2.0)
    self.assertIs(self.filt.update(lead, 12.0, 25.0, 1.0, 2.0), lead)
    self.assertIsNone(self.filt.x)
    self.assertEqual(self.filt.reason, 'braking')

  def test_rapid_closing_and_stopped_lead_return_model(self):
    for speed in (22.0, 0.0):
      self.establish()
      lead = make_lead(speed=speed)
      self.assertIs(self.filt.update(lead, 12.0, 25.0, 1.0, 2.0), lead)
      self.assertIsNone(self.filt.x)

  def test_near_stop_and_short_gap_return_model(self):
    for ego, distance in ((0.0, 40.0), (14.9, 40.0), (25.0, 9.9)):
      self.establish()
      lead = make_lead(distance, ego=ego)
      self.assertIs(self.filt.update(lead, 12.0, ego, 1.0, 2.0), lead)
      self.assertIsNone(self.filt.x)

  def test_radar_is_unchanged_and_clears_vision_state(self):
    self.establish()
    lead = dict(make_lead(), radar=True, radarTrackId=1)
    self.assertIs(self.filt.update(lead, 12.0, 25.0, 1.0, 2.0), lead)
    self.assertIsNone(self.filt.x)

  def test_loss_and_reacquisition_have_no_old_speed_memory(self):
    self.establish()
    lead = {'present': False}
    self.assertIs(self.filt.update(lead, 12.0, 25.0, 0.0, 2.0), lead)
    new = make_lead(60.0, speed=27.0)
    self.assertIs(self.filt.update(new, 12.05, 25.0, 1.0, 2.0), new)
    self.assertEqual(self.filt.x[1], 27.0)
    self.assertEqual(self.filt.age, 0.0)

  def test_raw_confidence_overrides_held_lead_probability(self):
    self.establish()
    lead = make_lead()
    self.assertIs(self.filt.update(lead, 12.0, 25.0, 0.90, 2.0), lead)
    self.assertIsNone(self.filt.x)

  def test_distance_lateral_and_model_speed_discontinuities(self):
    for changes in ({'dRel': 55.0}, {'yRel': 1.0}, {'vLead': 29.0}):
      self.filt.reset()
      self.establish()
      lead = dict(make_lead(), **changes)
      self.assertIs(self.filt.update(lead, 12.0, 25.0, 1.0, 2.0), lead)
      self.assertEqual(self.filt.reason, 'lead change')
      self.assertIsNone(self.filt.x)

  def test_invalid_distance_uncertainty_and_nonfinite_state(self):
    for sigma in (0.0, -1.0, 9.0, float('nan'), float('inf')):
      self.establish()
      lead = make_lead()
      self.assertIs(self.filt.update(lead, 12.0, 25.0, 1.0, sigma), lead)
      self.assertIsNone(self.filt.x)
    for field in ('dRel', 'yRel', 'vLead', 'aLeadK'):
      self.establish()
      lead = dict(make_lead(), **{field: float('nan')})
      self.assertIs(self.filt.update(lead, 12.0, 25.0, 1.0, 2.0), lead)
      self.assertIsNone(self.filt.x)

  def test_stale_duplicate_and_backwards_time_reset(self):
    for timestamp in (11.95, 11.0, 12.5):
      self.filt.reset()
      self.establish()
      lead = make_lead()
      self.assertIs(self.filt.update(lead, timestamp, 25.0, 1.0, 2.0), lead)
      self.assertIsNone(self.filt.x)

  def test_covariance_is_finite_symmetric_and_positive(self):
    rng = np.random.default_rng(2)
    timestamp = 0.0
    for _ in range(1000):
      timestamp += rng.uniform(0.03, 0.08)
      self.filt.update(make_lead(40.0+rng.normal(0, 0.3)), timestamp, 25.0, 1.0, 2.0)
      self.assertTrue(np.all(np.isfinite(self.filt.P)))
      np.testing.assert_allclose(self.filt.P, self.filt.P.T, atol=1e-12)
      self.assertGreaterEqual(np.linalg.eigvalsh(self.filt.P).min(), -1e-12)

  def test_future_samples_do_not_change_already_emitted_speed(self):
    # Two streams have the same prefix but different future distance histories.
    a, b = VisionLeadKalman(), VisionLeadKalman()
    prefix_a, prefix_b = [], []
    for t in np.arange(0, 10, 0.05):
      prefix_a.append(a.update(make_lead(), float(t), 25.0, 1.0, 2.0)['vLead'])
      prefix_b.append(b.update(make_lead(), float(t), 25.0, 1.0, 2.0)['vLead'])
    for t in np.arange(10, 12, 0.05):
      a.update(make_lead(40+(t-10)), float(t), 25.0, 1.0, 2.0)
      b.update(make_lead(40-(t-10)), float(t), 25.0, 1.0, 2.0)
    np.testing.assert_array_equal(prefix_a, prefix_b)
    self.assertGreater(a.x[1], b.x[1])


if __name__ == '__main__':
  unittest.main()
