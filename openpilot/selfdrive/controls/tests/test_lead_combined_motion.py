import math
import unittest

import numpy as np

from openpilot.selfdrive.controls.lib.lead_combined_motion import FutureVelocityKalman, LeadCombinedMotion
from openpilot.selfdrive.controls.tests.test_lead_motion_fusion import sample


class TestFutureVelocityKalman(unittest.TestCase):
  def test_forecast_jitter_is_filtered_without_frame_count_confidence(self):
    kf = FutureVelocityKalman()
    raw, filtered = [], []
    for i in range(120):
      speed = 30. + 1.5 * (-1)**i
      kf.update(speed, 2., i * .05)
      self.assertGreaterEqual(kf.velocity_variance, 4.)
      self.assertGreater(np.linalg.eigvalsh(kf.P).min(), 0.)
      if i >= 40:
        raw.append(speed)
        filtered.append(kf.x[0])
    self.assertLess(np.std(filtered), np.std(raw) / 2.)
    self.assertLessEqual(kf.updates, 25)

  def test_gap_reinitializes_future_state(self):
    kf = FutureVelocityKalman()
    for i in range(40):
      kf.update(30., 2., i * .05)
    self.assertFalse(kf.update(20., 3., 3.))
    self.assertEqual(kf.x[0], 20.)
    self.assertEqual(kf.updates, 1)

  def test_invalid_prediction_revokes_state(self):
    for speed, std in ((math.nan, 2.), (80., 2.), (30., 25.), (30., 0.)):
      kf = FutureVelocityKalman()
      kf.update(30., 2., 0.)
      self.assertFalse(kf.update(speed, std, .05))
      self.assertIsNone(kf.x)


class TestCombinedMotion(unittest.TestCase):
  @staticmethod
  def inputs(i, distance=None, forecast=26.):
    distance = 120. - 5. * i * .05 if distance is None else distance
    model, observation, raw = sample(distance, speed=32., std=2.)
    model.vStd = [1.] * 6
    model.x = [distance + 1.52 + forecast * t for t in model.t]
    return [model, model], [observation, observation], [raw, raw], [raw, raw], i * .05, 33., 0.

  def test_future_filter_is_active_and_buffer_does_not_replace_range_history(self):
    range_only = LeadCombinedMotion(False, False)
    future = LeadCombinedMotion(True, False)
    combined = LeadCombinedMotion(True, True)
    for i in range(100):
      args = self.inputs(i)
      outputs = [e.update(*args) for e in (range_only, future, combined)]
      for e in (future, combined):
        np.testing.assert_array_equal(e.motion.kalman.x, range_only.motion.kalman.x)
        np.testing.assert_array_equal(e.motion.kalman.P, range_only.motion.kalman.P)
      for out in outputs:
        for lead in out:
          for field in ('present', 'dRel', 'yRel', 'aLeadK', 'aLeadTau'):
            self.assertEqual(lead[field], args[3][0][field])
    self.assertTrue(combined.motion.future_available)
    self.assertGreater(combined.future_weight, 0.)
    self.assertGreater(combined.motion.forecast.updates, 3)
    self.assertNotEqual(outputs[0][0]['vLead'], outputs[1][0]['vLead'])
    self.assertLessEqual(abs(outputs[2][0]['vLead'] - outputs[1][0]['vLead']), .5)

  def test_near_range_has_future_fusion_but_no_model_buffer(self):
    future = LeadCombinedMotion(True, False)
    combined = LeadCombinedMotion(True, True)
    for i in range(80):
      args = self.inputs(i, distance=70.)
      self.assertEqual(future.update(*args), combined.update(*args))
    self.assertGreater(combined.future_weight, 0.)
    self.assertEqual(combined.model_weight, 0.)

  def test_empty_lane_clears_both_kalman_streams(self):
    experiment = LeadCombinedMotion()
    for i in range(50):
      experiment.update(*self.inputs(i))
    models, observations, _, _, t, ego, yaw = self.inputs(50)
    absent = [{'present': False}] * 2
    self.assertEqual(experiment.update(models, observations, absent, absent, t, ego, yaw), absent)
    self.assertIsNone(experiment.motion.kalman)
    self.assertIsNone(experiment.motion.forecast.x)

  def test_braking_signal_bypasses_fusion_immediately(self):
    experiment = LeadCombinedMotion()
    for i in range(50):
      experiment.update(*self.inputs(i))
    models, observations, raw, _, t, ego, yaw = self.inputs(50)
    leads = [dict(x, aLeadK=-1., vLead=20., vLeadK=20., vRel=-13.) for x in raw]
    self.assertEqual(experiment.update(models, observations, leads, leads, t, ego, yaw), leads)
    self.assertIsNone(experiment.motion.forecast.x)
