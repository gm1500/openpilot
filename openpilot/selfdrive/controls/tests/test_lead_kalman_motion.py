import unittest

import numpy as np

from openpilot.selfdrive.controls.lib.lead_kalman_motion import KalmanLeadMotion
from openpilot.selfdrive.controls.tests.test_lead_motion_fusion import sample


class TestKalmanLeadMotion(unittest.TestCase):
  def drive(self, experiment, count=100):
    for i in range(count):
      t = i * .05
      m, obs, raw = sample(140. - 8. * t, speed=32., std=5.)
      out = experiment.update([m, m], [obs, obs], [raw, raw], [raw, raw], t, 33., 0.)
      self.assertEqual(out[0]['dRel'], raw['dRel'])
      self.assertEqual(out[0]['aLeadK'], raw['aLeadK'])
      self.assertTrue(np.isfinite(experiment.kalman.x).all())
      self.assertGreater(np.linalg.eigvalsh(experiment.kalman.P).min(), 0.)

  def test_future_prior_updates_private_kalman_state(self):
    with_future = KalmanLeadMotion()
    range_only = KalmanLeadMotion(use_future=False)
    self.drive(with_future)
    self.drive(range_only)
    self.assertIsNotNone(with_future.future_time)
    self.assertIsNone(range_only.future_time)
    self.assertGreater(abs(with_future.kalman.x[1] - range_only.kalman.x[1]), .001)

  def test_empty_lane_clears_private_kalman(self):
    experiment = KalmanLeadMotion()
    self.drive(experiment)
    m, obs, _ = sample(90.)
    leads = [{'present': False}] * 2
    self.assertEqual(experiment.update([m, m], [obs, obs], leads, leads, 5., 33., 0.), leads)
    self.assertIsNone(experiment.kalman)

  def test_time_gap_discards_old_kalman_covariance(self):
    experiment = KalmanLeadMotion()
    self.drive(experiment)
    m, obs, raw = sample(90.)
    experiment.update([m, m], [obs, obs], [raw, raw], [raw, raw], 6., 33., 0.)
    self.assertEqual(len(experiment.history), 1)
    self.assertEqual(experiment.kalman.age, 0.)
    self.assertEqual(experiment.correction, 0.)
