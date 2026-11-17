import unittest

import numpy as np

from openpilot.selfdrive.controls.lib.lead_responsive_motion import LeadResponsiveMotion
from openpilot.selfdrive.controls.tests.test_lead_motion_fusion import sample


class TestResponsiveLeadMotion(unittest.TestCase):
  @staticmethod
  def inputs(t, distance, speed=30., baseline_speed=None):
    model, observation, raw = sample(distance, speed=speed, std=2.)
    model.vStd = [1.] * 6
    baseline = raw if baseline_speed is None else dict(raw, vLead=baseline_speed, vLeadK=baseline_speed, vRel=baseline_speed - 33.)
    return [model, model], [observation, observation], [raw, raw], [baseline, baseline], t, 33., 0.

  def test_confirmed_deceleration_adapts_without_resetting_history(self):
    adaptive = LeadResponsiveMotion(use_future=False, use_buffer=False)
    fixed = LeadResponsiveMotion(use_future=False, use_buffer=False, adaptive_noise=False)
    fast = []
    for i in range(121):
      t = i * .05
      braking_time = max(0., t - 2.)
      distance = 120. - 3. * t - braking_time**2
      true_speed = 30. - 2. * braking_time
      args = self.inputs(t, distance, speed=min(30., true_speed + 4.))
      adaptive.update(*args)
      fixed.update(*args)
      fast.append(adaptive.motion.fast_response)
      self.assertGreater(np.linalg.eigvalsh(adaptive.motion.kalman.P).min(), 0.)
    self.assertTrue(any(fast))
    self.assertGreater(adaptive.motion.kalman.age, 5.9)
    self.assertLess(abs(adaptive.motion.kalman.x[1] - true_speed), abs(fixed.motion.kalman.x[1] - true_speed))

  def test_steady_motion_and_one_outlier_do_not_select_fast_response(self):
    for outlier in (False, True):
      experiment = LeadResponsiveMotion()
      for i in range(120):
        t = i * .05
        distance = 120. - 3. * t + (10. if outlier and i == 65 else 0.)
        experiment.update(*self.inputs(t, distance))
        self.assertFalse(experiment.motion.fast_response)

  def test_current_model_corroboration_rejects_depth_drift(self):
    experiment = LeadResponsiveMotion()
    for i in range(120):
      t = i * .05
      args = self.inputs(t, 140. - 2. * t, speed=33.)
      self.assertEqual(experiment.update(*args), args[3])

  def test_existing_lower_lead_speed_is_never_raised(self):
    experiment = LeadResponsiveMotion()
    for i in range(100):
      t = i * .05
      args = self.inputs(t, 140. - 5. * t, speed=32., baseline_speed=24.)
      self.assertEqual(experiment.update(*args), args[3])

  def test_wrong_current_and_future_velocity_need_observed_closing(self):
    experiment = LeadResponsiveMotion()
    for i in range(120):
      args = self.inputs(i * .05, 140., speed=32.3)
      for model in args[0]:
        model.x = [141.52 + 22. * t for t in model.t]
      self.assertEqual(experiment.update(*args), args[3])

  def test_additional_slowing_enters_gradually_and_preserves_fields(self):
    experiment = LeadResponsiveMotion()
    previous = 0.
    for i in range(90):
      t = i * .05
      args = self.inputs(t, 145. - 8. * t, speed=32.)
      output = experiment.update(*args)
      correction = args[3][0]['vLead'] - output[0]['vLead']
      self.assertGreaterEqual(correction, 0.)
      self.assertLessEqual(correction - previous, experiment.CORRECTION_RATE * .05 + 1e-8)
      previous = correction
      for field in ('present', 'dRel', 'yRel', 'aLeadK', 'aLeadTau'):
        self.assertEqual(output[0][field], args[3][0][field])
    self.assertGreater(correction, 3.)

  def test_lead_absence_discards_both_filters_and_correction(self):
    experiment = LeadResponsiveMotion()
    for i in range(80):
      args = self.inputs(i * .05, 140. - 5. * i * .05, speed=32.)
      experiment.update(*args)
    models, observations, _, _, t, ego, yaw = self.inputs(4., 120.)
    absent = [{'present': False}] * 2
    self.assertEqual(experiment.update(models, observations, absent, absent, t, ego, yaw), absent)
    self.assertEqual(experiment.extra_slowing, 0.)
    self.assertIsNone(experiment.motion.kalman)
    self.assertIsNone(experiment.motion.forecast.x)

  def test_serialized_model_lists_reach_the_combined_filter(self):
    from openpilot.cereal import log

    experiment = LeadResponsiveMotion()
    for i in range(80):
      models, observations, raw, baseline, t, ego, yaw = self.inputs(i * .05, 140. - 8. * i * .05, speed=32.)
      message = log.ModelDataV2.new_message()
      for destination, model in zip(message.init('leadsV3', 2), models, strict=True):
        destination.t, destination.x, destination.xStd = model.t, model.x, model.xStd
        destination.vStd, destination.prob = model.vStd, model.prob
      with log.ModelDataV2.from_bytes(message.to_bytes()) as decoded:
        output = experiment.update(decoded.leadsV3, observations, raw, baseline, t, ego, yaw)
    self.assertEqual(experiment.reason, 'blend')
    self.assertLess(output[0]['vLead'], baseline[0]['vLead'])
    self.assertGreater(experiment.motion.forecast.updates, 3)
