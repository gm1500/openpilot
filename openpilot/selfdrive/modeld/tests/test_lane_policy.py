import unittest

import numpy as np

from openpilot.selfdrive.modeld import modeld
from openpilot.selfdrive.modeld.constants import ModelConstants


def make_model_output(left_prob: float = 0.99, right_prob: float = 0.99, lane_width: float = 3.6,
                      lane_width_end: float | None = None, lane_center: float = 0.0,
                      plan_y: float = 0.0) -> dict[str, np.ndarray]:
  x = np.asarray(ModelConstants.X_IDXS, dtype=np.float64)
  lane_lines = np.zeros((1, 4, len(x), 2), dtype=np.float64)
  target_width = lane_width if lane_width_end is None else lane_width_end
  width_progress = np.clip((x - 5.0) / 30.0, 0.0, 1.0)
  widths = lane_width + (target_width - lane_width) * width_progress
  # openpilot lateral coordinates are left-negative and right-positive.
  lane_lines[0, 1, :, 0] = lane_center - widths / 2.0
  lane_lines[0, 2, :, 0] = lane_center + widths / 2.0
  lane_line_probs = np.zeros((1, 8), dtype=np.float64)
  lane_line_probs[0, 3] = left_prob
  lane_line_probs[0, 5] = right_prob
  plan = np.zeros((1, len(x), ModelConstants.PLAN_WIDTH), dtype=np.float64)
  plan[0, :, 0] = x
  plan[0, :, 1] = plan_y
  return {'lane_lines': lane_lines, 'lane_lines_prob': lane_line_probs,
          'desire_state': np.zeros((1, ModelConstants.DESIRE_LEN), dtype=np.float64), 'plan': plan}


class TestLanePolicy(unittest.TestCase):
  def setUp(self):
    modeld.reset_lane_lock()

  def test_disabled_mode_returns_exact_e2e_target(self):
    self.assertEqual(modeld.apply_lane_lock(make_model_output(), 0.0123, 20.0, lane_policy_enabled=False), 0.0123)

  def test_raw_probability_indices(self):
    output = make_model_output(0.97, 0.96)
    output['lane_lines_prob'][0, 1] = 0.01
    self.assertEqual(modeld.get_inner_lane_line_probs(output), (0.97, 0.96))

  def test_actual_lane_order_has_positive_width_and_engages(self):
    output = make_model_output()
    self.assertLess(output['lane_lines'][0, 1, 0, 0], output['lane_lines'][0, 2, 0, 0])
    modeld.apply_lane_lock(output, 0.0, 20.0, lane_policy_enabled=True)
    self.assertTrue(modeld._lane_lock_full_active)

  def test_blinker_releases_lane_lock(self):
    output = make_model_output()
    modeld.apply_lane_lock(output, 0.0, 20.0, lane_policy_enabled=True)
    self.assertTrue(modeld._lane_lock_full_active)
    self.assertEqual(modeld.apply_lane_lock(output, 0.0123, 20.0, blinkers_active=True, lane_policy_enabled=True), 0.0123)
    self.assertFalse(modeld._lane_lock_full_active)

  def test_confidence_hysteresis_and_ramp(self):
    modeld.apply_lane_lock(make_model_output(), 0.0, 20.0, lane_policy_enabled=True)
    self.assertTrue(modeld._lane_lock_full_active)
    self.assertGreater(modeld._lane_lock_weight, 0.0)
    self.assertLess(modeld._lane_lock_weight, 1.0)
    modeld.apply_lane_lock(make_model_output(0.86, 0.86), 0.0, 20.0, lane_policy_enabled=True)
    self.assertTrue(modeld._lane_lock_full_active)
    modeld.apply_lane_lock(make_model_output(0.84, 0.84), 0.0, 20.0, lane_policy_enabled=True)
    self.assertFalse(modeld._lane_lock_full_active)

  def test_lane_midpoint_changes_curvature(self):
    curvature = modeld.apply_lane_lock(make_model_output(lane_center=0.5), 0.0, 20.0, lane_policy_enabled=True)
    self.assertTrue(modeld._lane_lock_full_active)
    self.assertGreater(curvature, 0.0)

  def test_extreme_path_disagreement_releases_lane_lock(self):
    curvature = modeld.apply_lane_lock(make_model_output(plan_y=1.0), 0.0123, 20.0, lane_policy_enabled=True)
    self.assertEqual(curvature, 0.0123)
    self.assertFalse(modeld._lane_lock_full_active)

  def test_curvature_disagreement_releases_lane_lock(self):
    curvature = modeld.apply_lane_lock(make_model_output(lane_center=0.5), -0.01, 20.0, lane_policy_enabled=True)
    self.assertEqual(curvature, -0.01)
    self.assertFalse(modeld._lane_lock_full_active)

  def test_robust_geometry_accepts_normal_width_taper(self):
    modeld.apply_lane_lock(make_model_output(lane_width=3.6, lane_width_end=3.9), 0.0, 20.0, lane_policy_enabled=True)
    self.assertTrue(modeld._lane_lock_full_active)

  def test_robust_geometry_rejects_excessive_width_taper(self):
    modeld.apply_lane_lock(make_model_output(lane_width=3.6, lane_width_end=4.4), 0.0, 20.0, lane_policy_enabled=True)
    self.assertFalse(modeld._lane_lock_full_active)
