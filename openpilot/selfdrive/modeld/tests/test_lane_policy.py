import unittest

import numpy as np

from openpilot.cereal import log
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

  def apply_for(self, output: dict[str, np.ndarray], seconds: float, e2e_curvature: float = 0.0,
                blinkers_active: bool = False) -> float:
    result = e2e_curvature
    for _ in range(max(1, int(np.ceil(seconds / modeld.DT_MDL)))):
      result = modeld.apply_lane_lock(output, e2e_curvature, 20.0,
                                      blinkers_active=blinkers_active, lane_policy_enabled=True)
    return result

  def arm_lane_policy(self, output: dict[str, np.ndarray] | None = None) -> dict[str, np.ndarray]:
    output = make_model_output() if output is None else output
    self.apply_for(output, modeld.LANE_LOCK_ARM_TIME + modeld.DT_MDL)
    self.assertTrue(modeld._lane_lock_ready)
    return output

  def test_disabled_mode_returns_exact_e2e_target(self):
    self.assertEqual(modeld.apply_lane_lock(make_model_output(), 0.0123, 20.0, lane_policy_enabled=False), 0.0123)

  def test_unset_lane_policy_selector_defaults_on_but_explicit_off_wins(self):
    class FakeParams:
      def __init__(self, value):
        self.value = value

      def get(self, key):
        assert key == modeld.LANE_POLICY_ENABLED_PARAM
        return self.value

    self.assertTrue(modeld.get_lane_policy_enabled(FakeParams(None)))
    self.assertTrue(modeld.get_lane_policy_enabled(FakeParams(True)))
    self.assertFalse(modeld.get_lane_policy_enabled(FakeParams(False)))

  def test_raw_probability_indices(self):
    output = make_model_output(0.97, 0.96)
    output['lane_lines_prob'][0, 1] = 0.01
    self.assertEqual(modeld.get_inner_lane_line_probs(output), (0.97, 0.96))

  def test_actual_lane_order_has_positive_width_and_arms_before_engaging(self):
    output = make_model_output()
    self.assertLess(output['lane_lines'][0, 1, 0, 0], output['lane_lines'][0, 2, 0, 0])
    self.apply_for(output, modeld.LANE_LOCK_ARM_TIME - modeld.DT_MDL)
    self.assertFalse(modeld._lane_lock_ready)
    self.assertEqual(modeld._lane_lock_weight, 0.0)
    self.apply_for(output, 2.0 * modeld.DT_MDL)
    self.assertTrue(modeld._lane_lock_ready)
    self.assertTrue(modeld._lane_lock_full_active)
    self.assertGreater(modeld._lane_lock_weight, 0.0)

  def test_blinker_releases_lane_lock(self):
    output = self.arm_lane_policy()
    self.assertEqual(modeld.apply_lane_lock(output, 0.0123, 20.0, blinkers_active=True, lane_policy_enabled=True), 0.0123)
    self.assertFalse(modeld._lane_lock_full_active)
    self.assertFalse(modeld._lane_lock_ready)

  def test_lane_change_intent_releases_and_requires_rearm(self):
    self.arm_lane_policy()
    output = make_model_output()
    output['desire_state'][0, log.Desire.laneChangeLeft] = 0.2
    self.assertEqual(modeld.apply_lane_lock(output, 0.0123, 20.0, lane_policy_enabled=True), 0.0123)
    self.assertFalse(modeld._lane_lock_full_active)
    self.assertFalse(modeld._lane_lock_ready)

  def test_confidence_weight_blends_instead_of_binary_release(self):
    self.arm_lane_policy()
    self.apply_for(make_model_output(), 4.0)
    full_weight = modeld._lane_lock_weight
    self.assertGreater(full_weight, 1.0 - 1e-3)

    self.apply_for(make_model_output(0.80, 0.80), 0.5)
    self.assertTrue(modeld._lane_lock_ready)
    self.assertFalse(modeld._lane_lock_full_active)
    self.assertGreater(modeld._lane_lock_weight, 0.70)
    self.assertLess(modeld._lane_lock_weight, full_weight)

  def test_low_confidence_fades_to_e2e_without_rearming(self):
    self.arm_lane_policy()
    self.apply_for(make_model_output(0.65, 0.65), 4.0, e2e_curvature=0.001)
    self.assertTrue(modeld._lane_lock_ready)
    self.assertFalse(modeld._lane_lock_has_lane_curvature)
    self.assertEqual(modeld.apply_lane_lock(make_model_output(0.65, 0.65), 0.001, 20.0, lane_policy_enabled=True), 0.001)

    modeld.apply_lane_lock(make_model_output(0.80, 0.80), 0.0, 20.0, lane_policy_enabled=True)
    self.assertTrue(modeld._lane_lock_has_lane_curvature)
    self.assertGreater(modeld._lane_lock_weight, 0.0)

  def test_confidence_weight_boundaries(self):
    self.assertEqual(modeld.get_lane_confidence_weight(0.70), 0.0)
    self.assertAlmostEqual(modeld.get_lane_confidence_weight(0.80), 2.0 / 3.0)
    self.assertEqual(modeld.get_lane_confidence_weight(0.85), 1.0)

  def test_lane_midpoint_changes_curvature(self):
    output = make_model_output(lane_center=0.5)
    self.arm_lane_policy(output)
    curvature = self.apply_for(output, 0.5)
    self.assertTrue(modeld._lane_lock_full_active)
    self.assertGreater(curvature, 0.0)

  def test_extreme_path_disagreement_releases_lane_lock(self):
    curvature = modeld.apply_lane_lock(make_model_output(plan_y=1.0), 0.0123, 20.0, lane_policy_enabled=True)
    self.assertEqual(curvature, 0.0123)
    self.assertFalse(modeld._lane_lock_full_active)
    self.assertFalse(modeld._lane_lock_ready)

  def test_curvature_disagreement_releases_lane_lock(self):
    curvature = modeld.apply_lane_lock(make_model_output(lane_center=0.5), -0.01, 20.0, lane_policy_enabled=True)
    self.assertEqual(curvature, -0.01)
    self.assertFalse(modeld._lane_lock_full_active)
    self.assertFalse(modeld._lane_lock_ready)

  def test_robust_geometry_accepts_normal_width_taper(self):
    output = make_model_output(lane_width=3.6, lane_width_end=3.9)
    self.arm_lane_policy(output)
    self.assertTrue(modeld._lane_lock_full_active)

  def test_robust_geometry_rejects_excessive_width_taper_immediately(self):
    self.arm_lane_policy()
    curvature = modeld.apply_lane_lock(make_model_output(lane_width=3.6, lane_width_end=4.4), 0.0123, 20.0,
                                       lane_policy_enabled=True)
    self.assertEqual(curvature, 0.0123)
    self.assertFalse(modeld._lane_lock_full_active)
    self.assertFalse(modeld._lane_lock_ready)
