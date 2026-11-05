import copy
import unittest

import numpy as np

from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.lane_policy import LanePolicy, is_enabled, plan_lane_correction


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


class TestPlanLaneCorrection(unittest.TestCase):
  def correction(self, speed=25.0, offset=0.3, confidence=0.9, curvature=0.0):
    model = model_output(offset=offset, curvature=curvature)
    center = model['lane_lines'][0, 1:3, :, 0].mean(axis=0)
    return plan_lane_correction(model, center, np.array(ModelConstants.X_IDXS), speed, confidence)

  def test_two_second_lookahead_and_known_acceleration(self):
    # A 30 cm path offset at 90% confidence produces 0.0675 m/s^2 at road speeds.
    for speed in (10.0, 20.0, 100 / 3.6, 120 / 3.6, 40.0):
      with self.subTest(speed=speed):
        correction, error, lookahead = self.correction(speed=speed)
        self.assertAlmostEqual(lookahead, 2 * speed)
        self.assertAlmostEqual(error, 0.3)
        self.assertAlmostEqual(correction * speed ** 2, 0.0675)

  def test_speed_fade(self):
    for speed, lateral_accel in ((0.0, 0.0), (4.0, 0.0), (6.0, 0.0375), (8.0, 0.075)):
      with self.subTest(speed=speed):
        correction, _, lookahead = self.correction(speed=speed, confidence=1.0)
        self.assertAlmostEqual(correction * speed ** 2, lateral_accel)
        self.assertGreaterEqual(lookahead, 10.0)

  def test_mirrored_curves_and_straights(self):
    for curvature in (-0.002, 0.0, 0.002):
      with self.subTest(curvature=curvature):
        right = self.correction(offset=0.3, curvature=curvature)[0]
        left = self.correction(offset=-0.3, curvature=curvature)[0]
        self.assertGreater(right, 0.0)
        self.assertAlmostEqual(right, -left)
        self.assertAlmostEqual(self.correction(offset=0.0, curvature=curvature)[0], 0.0)

  def test_lateral_acceleration_limit(self):
    for speed in (8.0, 20.0, 40.0):
      for offset in (-10.0, 10.0):
        with self.subTest(speed=speed, offset=offset):
          correction, _, _ = self.correction(speed=speed, offset=offset, confidence=1.0)
          self.assertAlmostEqual(abs(correction) * speed ** 2, 0.5)


class TestLanePolicy(unittest.TestCase):
  def setUp(self):
    self.policy = LanePolicy()
    self.model = model_output()

  def update(self, model=None, speed=25.0, base=0.001, **kwargs):
    options = {'lane_policy_enabled': True, 'lateral_active': True}
    options.update(kwargs)
    return self.policy.update(self.model if model is None else model, base, speed, **options)

  def arm(self):
    for _ in range(15):
      self.update()
    self.assertTrue(self.policy.active)

  def test_requires_continuous_strong_two_line_confidence(self):
    for _ in range(14):
      self.assertEqual(self.update(), 0.001)
    self.assertFalse(self.policy.active)
    self.assertTrue(self.policy.blending)
    self.assertEqual(self.update(model_output(left_prob=0.91)), 0.001)
    self.assertFalse(self.policy.blending)
    for _ in range(14):
      self.assertEqual(self.update(), 0.001)
    self.assertGreater(self.update(), 0.001)
    self.assertTrue(self.policy.active)
    self.assertFalse(self.policy.blending)

  def test_exact_arm_threshold(self):
    self.model = model_output(left_prob=0.92, right_prob=0.92)
    self.arm()

  def test_single_line_cannot_arm(self):
    for left, right in ((0.99, 0.0), (0.0, 0.99)):
      with self.subTest(left=left, right=right):
        self.policy.reset()
        for _ in range(30):
          self.assertEqual(self.update(model_output(left_prob=left, right_prob=right)), 0.001)
        self.assertFalse(self.policy.active)

  def test_retains_qualified_lines_at_lower_confidence(self):
    self.arm()
    for _ in range(50):
      self.update(model_output(left_prob=0.70, right_prob=0.70))
    self.assertTrue(self.policy.active)
    self.assertFalse(self.policy.holding_line)
    self.assertEqual(self.policy.lane_confidence, 0.70)
    self.assertEqual(self.update(model_output(left_prob=0.69, right_prob=0.69)), 0.001)
    self.assertFalse(self.policy.active)
    self.assertEqual(self.policy.correction_filter.x, 0.0)

  def test_one_line_hold_uses_retained_line_and_expires(self):
    for missing in (1, 2):
      with self.subTest(missing=missing):
        self.policy.reset()
        self.arm()
        model = model_output(left_prob=0.8, right_prob=0.8)
        model['lane_lines_prob'][0, 2 * missing + 1] = 0.0
        model['lane_lines'][0, missing, :, 0] = np.nan
        for _ in range(19):
          self.assertGreater(self.update(model), 0.001)
          self.assertTrue(self.policy.active)
          self.assertTrue(self.policy.blending)
          self.assertEqual(self.policy.lane_confidence, 0.8)
        self.assertEqual(self.update(model), 0.001)
        self.assertFalse(self.policy.active)
        self.assertEqual(self.policy.correction_filter.x, 0.0)

  def test_two_lines_recover_from_hold(self):
    self.arm()
    for _ in range(10):
      self.update(model_output(right_prob=0.0))
    self.update()
    self.assertFalse(self.policy.holding_line)
    self.assertEqual(self.policy.line_loss_time, 0.0)
    self.assertAlmostEqual(self.policy.width, 3.6)

  def test_disagreeing_lines_keep_original_left_priority(self):
    self.arm()
    model = model_output()
    model['lane_lines'][0, 2, :, 0] += 2.0
    self.update(model)
    self.assertTrue(self.policy.holding_line)
    self.assertAlmostEqual(self.policy.width, 3.6)
    # The bad right line cannot pull the reconstructed centre to the right.
    reference = LanePolicy()
    for _ in range(16):
      expected = reference.update(self.model, 0.001, 25.0, lane_policy_enabled=True)
    self.assertAlmostEqual(self.policy.correction + 0.001, expected)

  def test_invalid_width_cannot_arm(self):
    for width in (2.5, 2.65, 5.15, 5.3):
      with self.subTest(width=width):
        self.policy.reset()
        for _ in range(20):
          self.assertEqual(self.update(model_output(width=width)), 0.001)
        self.assertFalse(self.policy.active)

  def test_varying_width_cannot_arm(self):
    model = model_output()
    model['lane_lines'][0, 2, :, 0] += np.linspace(-2.0, 2.0, ModelConstants.IDX_N)
    for _ in range(20):
      self.assertEqual(self.update(model), 0.001)
    self.assertFalse(self.policy.active)

  def test_longer_lookahead_checks_lane_geometry(self):
    model = model_output()
    x = np.array(ModelConstants.X_IDXS)
    model['lane_lines'][0, 2, x > 55.0, 0] += 3.0
    for _ in range(20):
      self.assertEqual(self.update(model, speed=120 / 3.6), 0.001)
    self.assertFalse(self.policy.active)

  def test_missing_line_at_long_lookahead_uses_one_line_hold(self):
    self.arm()
    model = model_output()
    x = np.array(ModelConstants.X_IDXS)
    model['lane_lines'][0, 2, x > 55.0, 0] = np.nan
    self.assertGreater(self.update(model, speed=120 / 3.6), 0.001)
    self.assertTrue(self.policy.holding_line)

  def test_unused_far_lane_samples_do_not_disable_detection(self):
    model = model_output()
    x = np.array(ModelConstants.X_IDXS)
    model['lane_lines'][0, 1:3, x > 100.0, 0] = np.nan
    for _ in range(15):
      self.update(model)
    self.assertTrue(self.policy.active)

  def test_toggle_and_blinker_clear_filter_immediately(self):
    for override in ({'lane_policy_enabled': False}, {'blinkers_active': True}):
      with self.subTest(override=override):
        self.arm()
        self.assertEqual(self.update(**override), 0.001)
        self.assertFalse(self.policy.active)
        self.assertEqual(self.policy.correction_filter.x, 0.0)
        for _ in range(14):
          self.assertEqual(self.update(), 0.001)
        self.assertGreater(self.update(), 0.001)

  def test_lane_change_intent_clears_filter(self):
    for direction in (3, 4):
      with self.subTest(direction=direction):
        self.arm()
        model = model_output()
        model['desire_state'][0, direction] = 0.11
        self.assertEqual(self.update(model), 0.001)
        self.assertFalse(self.policy.active)
        self.assertEqual(self.policy.correction_filter.x, 0.0)

  def test_filter_ramps_and_does_not_accumulate_bias(self):
    self.arm()
    # First correction is 1/11 of the steady target for the upstream 0.5 s filter at 20 Hz.
    self.assertAlmostEqual(self.policy.correction, 0.3 * 0.99 / 2500 / 11)
    for _ in range(200):
      result = self.update()
    self.assertAlmostEqual(result - 0.001, 0.3 * 0.99 / 2500, places=10)
    previous = self.policy.correction
    self.update(model_output(offset=-0.3))
    self.assertGreater(self.policy.correction, 0.0)
    self.assertLess(self.policy.correction, previous)
    for _ in range(200):
      self.update(model_output(offset=0.0))
    self.assertAlmostEqual(self.policy.correction, 0.0, places=10)

  def test_speed_rise_keeps_filtered_correction_bounded(self):
    self.model = model_output(offset=10.0)
    self.arm()
    for _ in range(100):
      self.update(speed=8.0)
    self.assertGreater(self.policy.correction * 40.0 ** 2, 0.5)
    self.update(speed=40.0)
    self.assertLessEqual(abs(self.policy.correction) * 40.0 ** 2, 0.5 + 1e-12)
    self.assertEqual(self.policy.correction_filter.x, self.policy.correction)

  def test_bad_or_short_model_path_releases_existing_correction(self):
    for kind in ('short', 'reversed', 'nan', 'missing', 'bad_shape'):
      with self.subTest(kind=kind):
        self.arm()
        model = model_output()
        if kind == 'short':
          model['plan'][0, :, 0] *= 0.2  # 38.4 m cannot support the 50 m lookahead.
        elif kind == 'reversed':
          model['plan'][0, 10, 0] = model['plan'][0, 9, 0]
        elif kind == 'nan':
          model['plan'][0, 10, 1] = np.nan
        elif kind == 'missing':
          del model['plan']
        else:
          model['plan'] = np.zeros((1, 33, 1))
        self.assertEqual(self.update(model), 0.001)
        self.assertFalse(self.policy.active)
        self.assertEqual(self.policy.correction_filter.x, 0.0)

  def test_nonfinite_lane_change_and_speed_fall_back(self):
    self.arm()
    model = model_output()
    model['desire_state'][0, 3] = np.nan
    self.assertEqual(self.update(model), 0.001)
    self.arm()
    self.assertEqual(self.update(speed=np.nan), 0.001)
    self.assertFalse(self.policy.active)

  def test_input_model_and_base_action_are_preserved(self):
    before = copy.deepcopy(self.model)
    self.arm()
    for key in self.model:
      np.testing.assert_array_equal(self.model[key], before[key])
    self.assertEqual(self.update(base=-0.002, lane_policy_enabled=False), -0.002)

  def test_existing_toggle_encoding(self):
    for value in (None, True, b'1'):
      self.assertTrue(is_enabled(value))
    for value in (False, b'0', b''):
      self.assertFalse(is_enabled(value))


if __name__ == '__main__':
  unittest.main()
