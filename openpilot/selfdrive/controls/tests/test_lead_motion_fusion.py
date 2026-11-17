import math
import gzip
import json
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

from openpilot.selfdrive.controls.lib.lead_motion_fusion import LeadMotionFusion
from openpilot.selfdrive.controls.lib.vision_lead_tracker import VisionLeadObservation


def sample(distance, speed=30., probability=.99, std=5., lateral=0., ego=33., acceleration=0.):
  times = [0., 2., 4., 6., 8., 10.]
  model = SimpleNamespace(t=times, x=[distance + 1.52 + speed * t for t in times],
                          xStd=[std] * 6, prob=probability)
  obs = VisionLeadObservation(distance, lateral, std, probability)
  lead = {'present': True, 'radar': False, 'dRel': distance, 'yRel': lateral, 'vLead': speed, 'vLeadK': speed,
              'vRel': speed - ego, 'aLeadK': acceleration, 'aLeadTau': .3, 'modelProb': probability}
  return model, obs, lead


class TestLeadMotionFusion(unittest.TestCase):
  def setUp(self):
    self.filter = LeadMotionFusion()
    self.time = -.05

  def step(self, distance, second=None, single=False, valid=True, yaw=0., ego=33., dt=.05, **kwargs):
    self.time += dt
    m, obs, lead = sample(distance, ego=ego, **kwargs)
    m2, obs2, lead2 = sample(**second, ego=ego) if second else (m, obs, lead)
    if single:
      obs2 = VisionLeadObservation(0., 0., 1., 0.)
      lead2 = {'present': False}
    raw = [lead, lead2]
    out = self.filter.update([m, m2], [obs, obs2], raw, raw, self.time, ego, yaw, valid)
    for before, after in zip(raw, out, strict=True):
      self.assertEqual(before['present'], after['present'])
      if before['present']:
        for field in ('dRel', 'yRel', 'aLeadK', 'aLeadTau'):
          self.assertEqual(before[field], after[field])
        self.assertLessEqual(after['vLead'], before['vLead'])
        self.assertGreaterEqual(after['vLead'], before['vLead'] - self.filter.MAX_CORRECTION - 1e-8)
    return out

  def warm_closing(self):
    for i in range(70):
      out = self.step(140. - 10. * i * .05, speed=32., std=6.)
    self.assertGreater(self.filter.correction, 0.)
    return out

  def test_noisy_closing_establishes_bounded_cue(self):
    previous = 0.
    active = []
    for i in range(100):
      self.step(145. - 10. * i * .05 + 1.8 * math.sin(1.7 * i), speed=32., std=8.)
      self.assertLessEqual(self.filter.correction - previous, .100001)
      previous = self.filter.correction
      active.append(previous)
    self.assertGreater(max(active), 1.)

  def test_one_slot_and_duplicate_slots_have_same_confidence(self):
    results = []
    for single in (False, True):
      self.setUp()
      for i in range(80):
        self.step(140. - 10. * i * .05, speed=32., single=single)
      results.append((self.filter.correction, self.filter.fit.error, self.filter.future_weight))
    for a, b in zip(*results, strict=True):
      self.assertAlmostEqual(a, b)

  def test_isolated_outlier_revokes_output_but_keeps_private_history(self):
    self.warm_closing()
    count = len(self.filter.history)
    self.step(140. - 10. * 3.5 + 12., speed=32., std=6.)
    self.assertEqual(self.filter.correction, 0.)
    self.assertEqual(self.filter.reason, 'range_quarantine')
    self.assertEqual(len(self.filter.history), count)
    for i in range(8):
      self.step(140. - 10. * (3.55 + i * .05), speed=32., std=6.)
    self.assertGreater(self.filter.correction, 0.)

  def test_persistent_small_handoff_does_not_inherit_old_velocity(self):
    self.warm_closing()
    for _i in range(20):
      self.step(112., speed=33., std=2.)
      self.assertEqual(self.filter.correction, 0.)

  def test_cut_in_with_lateral_change_starts_fresh(self):
    self.warm_closing()
    out = self.step(45., lateral=1., speed=12., acceleration=-1.)
    self.assertEqual(out[0]['vLead'], 12.)
    self.assertEqual(self.filter.correction, 0.)
    self.assertFalse(self.filter.history)

  def test_lane_change_into_empty_lane_cannot_create_lead(self):
    self.warm_closing()
    m, obs, raw = sample(70.)
    out = self.filter.update([m, m], [obs, obs], [{'present': False}] * 2, [{'present': False}] * 2,
                             self.time + .05, 33., 0.)
    self.assertEqual(out, [{'present': False}] * 2)
    self.assertEqual(self.filter.correction, 0.)
    self.assertFalse(self.filter.history)

  def test_distinct_targets_and_slot_swap_do_not_share_history(self):
    self.warm_closing()
    self.step(105., speed=32., second={'distance': 65., 'lateral': 3., 'speed': 22.})
    self.assertFalse(self.filter.history)
    self.step(65., speed=22., lateral=3., second={'distance': 105., 'speed': 32.})
    self.assertFalse(self.filter.history)

  def test_gradual_lateral_motion_disables_closing_cue(self):
    for i in range(80):
      self.step(140. - 8. * i * .05, lateral=.8 * i * .05, speed=32.)
      self.assertEqual(self.filter.correction, 0.)

  def test_steady_opening_and_wrong_future_do_not_cause_braking(self):
    for rate in (0., 4.):
      self.setUp()
      for i in range(100):
        self.step(80. + rate * i * .05 + .5 * math.sin(i), speed=25., std=2.)
        self.assertEqual(self.filter.correction, 0.)

  def test_braking_stopped_and_low_speed_bypass(self):
    for values in [{'speed': 18., 'acceleration': -1.}, {'speed': 0.}, {'ego': 10., 'speed': 7.}]:
      self.setUp()
      self.warm_closing()
      self.step(40., **values)
      self.assertEqual(self.filter.correction, 0.)

  def test_bad_inputs_and_time_gap_revoke(self):
    for values in [{'valid': False}, {'yaw': None}, {'yaw': math.nan}, {'dt': .3}, {'std': math.inf}]:
      self.setUp()
      self.warm_closing()
      self.step(105., **values)
      self.assertEqual(self.filter.correction, 0.)

  def test_future_prior_never_reduces_range_uncertainty(self):
    self.warm_closing()
    self.assertGreater(self.filter.future_weight, 0.)
    self.assertLessEqual(self.filter.future_weight, .25)
    self.assertGreaterEqual(self.filter.speed_upper, self.filter.fit.speed)

  def test_correlated_depth_drift_without_model_closing_is_vetoed(self):
    for seed in range(10):
      self.setUp()
      rng = np.random.default_rng(seed)
      noise = 0.
      for _i in range(200):
        noise = .95 * noise + rng.normal(0., 1.2)
        self.step(100. + noise, speed=33., std=8.)
        self.assertEqual(self.filter.correction, 0.)

  def test_range_wave_without_model_closing_is_vetoed(self):
    for i in range(200):
      self.step(100. + 5. * math.sin(i * .05 * .9), speed=33., std=8.)
      self.assertEqual(self.filter.correction, 0.)

  @unittest.skipUnless((Path(__file__).parent / 'fixtures' / 'lead_future_approach.json.gz').exists(),
                       'optional private replay fixture unavailable')
  def test_recorded_approach_preserves_range_and_publishes_bounded_cue(self):
    path = Path(__file__).parent / 'fixtures' / 'lead_future_approach.json.gz'
    with gzip.open(path, 'rt') as f:
      rows = json.load(f)
    active = []
    for r in rows:
      out = self.filter.update([SimpleNamespace(**m) for m in r['models']],
                               [VisionLeadObservation(**o) for o in r['observations']],
                               r['raw'], r['baseline'], r['t'], r['ego'], r['yaw'])
      for before, after in zip(r['baseline'], out, strict=True):
        self.assertEqual(before['present'], after['present'])
        if before['present']:
          for field in ('dRel', 'yRel', 'aLeadK', 'aLeadTau'):
            self.assertEqual(before[field], after[field])
      if self.filter.correction > 0:
        active.append((r['t'], self.filter.correction))
    self.assertTrue(active)
    self.assertLess(active[0][0], 10.)
    self.assertLessEqual(max(correction for _, correction in active), self.filter.MAX_CORRECTION)


if __name__ == '__main__':
  unittest.main()
