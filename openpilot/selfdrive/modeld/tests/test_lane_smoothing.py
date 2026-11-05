import unittest

import numpy as np

from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.lane_policy import (
  MAX_CORRECTION, RELEASE_STEP, LanePolicy, anchor_correction, soften_correction,
)


def model_output(offset=0.3, width=3.6, left_prob=0.99, right_prob=0.99, curvature=0.0):
  x = np.array(ModelConstants.X_IDXS)
  path = 0.5 * curvature * x ** 2
  plan = np.zeros((1, len(x), ModelConstants.PLAN_WIDTH))
  plan[0, :, 0], plan[0, :, 1] = x, path
  lines = np.zeros((1, 4, len(x), ModelConstants.LANE_LINES_WIDTH))
  lines[0, 1, :, 0] = path + offset - width / 2
  lines[0, 2, :, 0] = path + offset + width / 2
  probs = np.zeros((1, 8))
  probs[0, 3], probs[0, 5] = left_prob, right_prob
  return {'plan': plan, 'lane_lines': lines, 'lane_lines_prob': probs, 'desire_state': np.zeros((1, 8))}


class TestLaneSmoothing(unittest.TestCase):
  def setUp(self):
    self.policy = LanePolicy()

  def update(self, model=None, base=0.001, speed=25.0, **kwargs):
    options = {'lane_policy_enabled': True}
    options.update(kwargs)
    return self.policy.update(model_output() if model is None else model, base, speed, **options)

  def settle(self, model=None, **kwargs):
    for _ in range(400):
      result = self.update(model, **kwargs)
    self.assertTrue(self.policy.active)
    return result

  def test_upstream_half_second_filter_response(self):
    # Independent recurrence: at 20 Hz, a 0.5 s time constant gives alpha = 1/11.
    for base in (-0.001, 0.0, 0.001):
      with self.subTest(base=base):
        self.policy.reset()
        expected = 0.0
        for target in 0.0003 * np.sin(np.arange(200) * 0.4):
          expected += (target - expected) / 11.0
          actual, guarded = self.policy._guard_correction(target, base)
          self.assertAlmostEqual(actual, expected, places=15)
          self.assertFalse(guarded)

  def test_curve_correction_reversal_has_no_forced_zero_pause(self):
    target = 0.0003
    for _ in range(400):
      self.policy._guard_correction(target, 0.001)
    samples = [self.policy._guard_correction(-target, 0.001) for _ in range(30)]
    values = np.array([value for value, _ in samples])
    self.assertTrue(all(not guarded for _, guarded in samples))
    self.assertTrue(np.all(np.diff(values) < 0.0))
    self.assertTrue(np.all(values != 0.0))
    self.assertGreater(values[0], 0.0)
    self.assertLess(values[-1], 0.0)

  def test_actual_turn_reversal_still_clears_opposing_correction(self):
    self.policy._track_turn(0.001)
    for _ in range(100):
      self.policy._guard_correction(0.0003, 0.001)
    self.policy._track_turn(-0.001)
    for _ in range(7):
      value, guarded = self.policy._guard_correction(0.0003, -0.001)
      self.assertTrue(guarded)
      self.assertEqual(value, 0.0)
      self.assertEqual(self.policy.correction_filter.x, 0.0)
    # Corrections following the new turn direction remain available.
    value, _ = self.policy._guard_correction(-0.0003, -0.001)
    self.assertLess(value, 0.0)

  def test_original_steady_centering_strength_across_speeds_and_curves(self):
    for speed in (50 / 3.6, 80 / 3.6, 100 / 3.6, 120 / 3.6):
      for curvature in (-0.001, 0.0, 0.001):
        for offset in (-0.3, 0.3):
          with self.subTest(speed=speed, curvature=curvature, offset=offset):
            self.policy.reset()
            model = model_output(offset=offset, curvature=curvature)
            center = model['lane_lines'][0, 1:3, :, 0].mean(axis=0)
            expected = anchor_correction(model, center, np.array(ModelConstants.X_IDXS), speed)[0]
            result = self.settle(model, base=curvature, speed=speed)
            self.assertAlmostEqual(result - curvature, soften_correction(expected), places=12)
            self.assertEqual(self.policy.bias, 0.0)

  def test_base_model_curvature_is_not_filtered(self):
    centered = model_output(offset=0.0)
    self.settle(centered)
    for base in (0.0012, 0.0005, -0.001, -0.002, 0.0):
      self.assertAlmostEqual(self.update(centered, base=base), base, places=15)

  def test_continuous_strong_confidence_required_to_arm(self):
    for _ in range(14):
      self.assertEqual(self.update(), 0.001)
    self.assertFalse(self.policy.active)
    self.assertEqual(self.update(model_output(left_prob=0.91)), 0.001)
    for _ in range(14):
      self.assertEqual(self.update(), 0.001)
    self.assertGreater(self.update(), 0.001)
    self.assertTrue(self.policy.active)

  def test_confidence_thresholds_and_width_checks(self):
    for _ in range(15):
      self.update(model_output(left_prob=0.92, right_prob=0.92))
    self.assertTrue(self.policy.active)
    self.update(model_output(left_prob=0.70, right_prob=0.70))
    self.assertTrue(self.policy.active)
    self.assertEqual(self.update(model_output(left_prob=0.69, right_prob=0.69)), 0.001)
    self.assertFalse(self.policy.active)
    for width in (2.5, 5.3):
      for _ in range(20):
        self.assertEqual(self.update(model_output(width=width)), 0.001)
      self.assertFalse(self.policy.active)

  def test_one_line_hold_expires_and_clears_filter(self):
    self.settle()
    one_line = model_output(right_prob=0.0)
    for _ in range(19):
      self.update(one_line)
      self.assertTrue(self.policy.active)
      self.assertTrue(self.policy.holding_line)
    for _ in range(2):
      result = self.update(one_line)
    self.assertEqual(result, 0.001)
    self.assertFalse(self.policy.active)
    self.assertEqual(self.policy.correction_filter.x, 0.0)

  def test_fallback_clears_filter_and_rearms_without_stale_correction(self):
    lane_change = model_output()
    lane_change['desire_state'][0, 3] = 0.11
    cases = [
      (model_output(), {'lane_policy_enabled': False}),
      (model_output(), {'blinkers_active': True}),
      (lane_change, {}),
      (model_output(left_prob=0.0, right_prob=0.0), {}),
    ]
    for model, options in cases:
      with self.subTest(options=options, desire=model['desire_state'].tolist()):
        self.settle()
        self.assertGreater(self.policy.correction_filter.x, 0.0)
        self.assertEqual(self.update(model, **options), 0.001)
        self.assertEqual(self.policy.correction_filter.x, 0.0)
        self.assertEqual(self.policy.correction, 0.0)
        self.assertFalse(self.policy.active)
        for _ in range(15):
          self.assertAlmostEqual(self.update(model_output(offset=0.0)), 0.001, places=15)
        self.assertTrue(self.policy.active)

  def test_bad_path_falls_back_and_clears_filter(self):
    self.settle()
    bad_path = model_output()
    bad_path['plan'][0, :, 0] = np.nan
    self.assertEqual(self.update(bad_path), 0.001)
    self.assertFalse(self.policy.active)
    self.assertEqual(self.policy.correction_filter.x, 0.0)

  def test_lane_reversals_do_not_reset_motion_feedback(self):
    self.settle(model_output(offset=0.0, curvature=0.001), lateral_active=True)
    x = np.array(ModelConstants.X_IDXS)
    corrections = []
    for i in range(80):
      model = model_output(offset=0.0, curvature=0.001)
      # Lanes and near geometry stay fixed; only the E2E path shape varies.
      model['plan'][0, :, 1] += 0.5 * (0.0003 * np.sin(i * 0.4)) * x ** 2
      corrections.append(self.update(model, lateral_active=True) - 0.001)
      self.assertIsNotNone(self.policy.motion_time)
      self.assertEqual(self.policy.turn_release, 0.0)
      self.assertTrue(self.policy.active)
    self.assertGreater(max(corrections), 0.0)
    self.assertLess(min(corrections), 0.0)

  def test_learned_bias_still_builds_and_driver_override_resets_it(self):
    self.settle(model_output(offset=0.1), lateral_active=True)
    self.assertGreater(self.policy.bias, 0.0)
    self.assertGreater(self.policy.correction, self.policy.correction_filter.x)
    self.update(model_output(offset=0.1), lateral_active=True, steering_pressed=True)
    self.assertEqual(self.policy.bias, 0.0)

  def test_correction_and_per_frame_bounds(self):
    previous = 0.0
    for offset in (10.0, -10.0, 10.0):
      for _ in range(100):
        result = self.update(model_output(offset=offset))
        correction = result - 0.001
        self.assertLessEqual(abs(correction), MAX_CORRECTION + 1e-15)
        self.assertLessEqual(abs(correction - previous), RELEASE_STEP + 1e-15)
        previous = correction


if __name__ == '__main__':
  unittest.main()
