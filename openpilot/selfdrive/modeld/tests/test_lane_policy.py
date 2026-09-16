import unittest

import numpy as np

from openpilot.selfdrive.modeld import modeld
from openpilot.selfdrive.modeld.constants import ModelConstants


def make_model_output(left_prob: float = 0.99, right_prob: float = 0.99, lane_width: float = 3.6) -> dict[str, np.ndarray]:
  x = np.asarray(ModelConstants.X_IDXS, dtype=np.float64)
  lane_lines = np.zeros((1, 4, len(x), 2), dtype=np.float64)
  lane_lines[0, 1, :, 0] = lane_width / 2.0
  lane_lines[0, 2, :, 0] = -lane_width / 2.0

  lane_line_probs = np.zeros((1, 8), dtype=np.float64)
  lane_line_probs[0, 3] = left_prob
  lane_line_probs[0, 5] = right_prob

  plan = np.zeros((1, len(x), ModelConstants.PLAN_WIDTH), dtype=np.float64)
  plan[0, :, 0] = x
  return {
    'lane_lines': lane_lines,
    'lane_lines_prob': lane_line_probs,
    'desire_state': np.zeros((1, ModelConstants.DESIRE_LEN), dtype=np.float64),
    'plan': plan,
  }


class TestLanePolicy(unittest.TestCase):
  def setUp(self):
    modeld.reset_lane_lock()

  def test_uses_inner_raw_probability_indices(self):
    output = make_model_output(0.97, 0.96)
    output['lane_lines_prob'][0, 1] = 0.01
    output['lane_lines_prob'][0, 2] = 0.02
    self.assertEqual(modeld.get_inner_lane_line_probs(output), (0.97, 0.96))

  def test_disabled_mode_returns_exact_e2e_target(self):
    output = make_model_output()
    self.assertEqual(modeld.apply_lane_lock(output, 0.0123, 20.0, lane_policy_enabled=False), 0.0123)

  def test_bad_probability_layout_releases_lane_lock(self):
    output = make_model_output()
    modeld.apply_lane_lock(output, 0.0, 20.0, lane_policy_enabled=True)
    self.assertTrue(modeld._lane_lock_full_active)

    output['lane_lines_prob'] = np.zeros((1, 4), dtype=np.float64)
    self.assertEqual(modeld.apply_lane_lock(output, 0.0, 20.0, lane_policy_enabled=True), 0.0)
    self.assertFalse(modeld._lane_lock_full_active)

  def test_invalid_lane_geometry_releases_lane_lock(self):
    output = make_model_output()
    modeld.apply_lane_lock(output, 0.0, 20.0, lane_policy_enabled=True)
    self.assertTrue(modeld._lane_lock_full_active)

    invalid = make_model_output(lane_width=2.0)
    modeld.apply_lane_lock(invalid, 0.0, 20.0, lane_policy_enabled=True)
    self.assertFalse(modeld._lane_lock_full_active)
