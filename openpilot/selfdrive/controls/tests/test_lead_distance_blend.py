import math
import unittest

from openpilot.selfdrive.controls.lib.lead_distance_blend import LeadDistanceBlend
from openpilot.selfdrive.controls.lib.vision_lead_tracker import VisionLeadObservation
from openpilot.selfdrive.controls.tests.test_lead_motion_fusion import sample


class TestDistanceBlend(unittest.TestCase):
  def setUp(self):
    self.experiment = LeadDistanceBlend()
    self.time = -.05

  def step(self, distance=100., speed=35., future=None, vstd=3., **kwargs):
    self.time += .05
    model, obs, raw = sample(distance, speed=speed, std=2., **kwargs)
    model.vStd = [vstd] * 6
    if future is not None:
      model.x = [distance + 1.52 + future * t for t in model.t]
    before = raw.copy()
    out = self.experiment.update([model, model], [obs, obs], [raw, raw], [raw, raw], self.time, 33., 0.)
    self.assertEqual(raw, before)
    for lead in out:
      for field in ('present', 'dRel', 'yRel', 'aLeadK', 'aLeadTau'):
        self.assertEqual(lead[field], raw[field])
    return out

  def warm(self):
    for _ in range(100):
      out = self.step()
    self.assertEqual(self.experiment.reason, 'blend')
    return out

  def test_direct_velocity_constrains_range_estimate(self):
    out = self.warm()
    self.assertGreater(out[0]['vLead'], self.experiment.range_speed)
    self.assertLess(out[0]['vLead'], 35.)

  def test_future_is_bounded_prior_and_duplicate_slots_match(self):
    self.experiment = LeadDistanceBlend(use_future=True)
    for _ in range(100):
      out = self.step(future=30.)
    self.assertGreater(self.experiment.future_weight, 0.)
    self.assertLessEqual(self.experiment.future_weight, .20)
    self.assertEqual(out[0]['vLead'], out[1]['vLead'])
    self.assertGreaterEqual(out[0]['vLead'], 30.)
    self.assertLessEqual(out[0]['vLead'], 35.)

  def test_disappearing_lead_clears_history(self):
    self.warm()
    model, obs, _ = sample(100.)
    absent = [{'present': False}] * 2
    out = self.experiment.update([model, model], [obs, obs], absent, absent, self.time + .05, 33., 0.)
    self.assertEqual(out, absent)
    self.assertIsNone(self.experiment.motion.kalman)

  def test_explicit_braking_bypasses_blend_immediately(self):
    self.warm()
    out = self.step(distance=80., speed=22., acceleration=-1.)
    self.assertEqual(out[0]['vLead'], 22.)
    self.assertIsNone(self.experiment.motion.kalman)

  def test_invalid_model_velocity_uncertainty_bypasses(self):
    for value in (math.nan, math.inf, 0., -1.):
      self.setUp()
      self.warm()
      out = self.step(vstd=value)
      self.assertEqual(out[0]['vLead'], 35.)

  def test_second_distinct_target_does_not_inherit_blend(self):
    self.warm()
    m, obs, raw = sample(100., speed=35.)
    m.vStd = [3.] * 6
    other = dict(raw, dRel=60., yRel=3., vLead=22.)
    out = self.experiment.update([m, m], [obs, VisionLeadObservation(60., 3., 2., .99)],
                                 [raw, other], [raw, other], self.time + .05, 33., 0.)
    self.assertEqual(out, [raw, other])
    self.assertIsNone(self.experiment.motion.kalman)

  def test_closing_guard_does_not_raise_baseline_during_supported_closing(self):
    self.experiment = LeadDistanceBlend(closing_guard=True)
    guarded = False
    for i in range(120):
      t = i * .05
      m, obs, raw = sample(120. - 8. * t, speed=33., std=2.)
      m.vStd = [3.] * 6
      baseline = dict(raw, vLead=23., vLeadK=23., vRel=-10.)
      out = self.experiment.update([m, m], [obs, obs], [raw, raw], [baseline, baseline], t, 33., 0.)
      motion = self.experiment.motion
      if motion.fit is not None and self.experiment.reason == 'blend':
        closing = 33. - motion.kalman.x[1] - 2. * motion.fit.error
        if closing > .5 and obs.distance < 12. * closing:
          guarded = True
          self.assertLessEqual(out[0]['vLead'], baseline['vLead'])
    self.assertTrue(guarded)

  def test_small_long_range_buffer_keeps_the_same_range_state(self):
    control = LeadDistanceBlend('buffer_0')
    buffered = LeadDistanceBlend('buffer_10')
    changed = False
    for i in range(120):
      m, obs, raw = sample(140., speed=35., std=2.)
      m.vStd = [1.] * 6
      args = ([m, m], [obs, obs], [raw, raw], [raw, raw], i * .05, 33., 0.)
      before = control.update(*args)
      after = buffered.update(*args)
      self.assertTrue((control.motion.kalman.x == buffered.motion.kalman.x).all())
      self.assertTrue((control.motion.kalman.P == buffered.motion.kalman.P).all())
      delta = after[0]['vLead'] - before[0]['vLead']
      self.assertLessEqual(abs(delta), .500001)
      self.assertLessEqual(abs(delta), .10 * abs(raw['vLead'] - before[0]['vLead']) + 1e-8)
      changed |= abs(delta) > .01
    self.assertTrue(changed)

  def test_small_buffer_has_no_near_range_contribution(self):
    control = LeadDistanceBlend('buffer_0')
    buffered = LeadDistanceBlend('buffer_15')
    for i in range(100):
      m, obs, raw = sample(70., speed=35., std=2.)
      m.vStd = [1.] * 6
      args = ([m, m], [obs, obs], [raw, raw], [raw, raw], i * .05, 33., 0.)
      self.assertEqual(control.update(*args), buffered.update(*args))
