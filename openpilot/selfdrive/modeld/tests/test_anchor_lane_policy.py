import unittest

import numpy as np

from openpilot.cereal import log
from openpilot.selfdrive.modeld import modeld
from openpilot.selfdrive.modeld.constants import ModelConstants


def make_model_output(left_prob: float = 0.99, right_prob: float = 0.99, lane_width: float = 3.6,
                      lane_width_end: float | None = None, lane_center: float = 0.0) -> dict[str, np.ndarray]:
  x = np.asarray(ModelConstants.X_IDXS, dtype=np.float64)
  lane_lines = np.zeros((1, 4, len(x), 2), dtype=np.float64)
  width_end = lane_width if lane_width_end is None else lane_width_end
  progress = np.clip((x - 5.0) / 30.0, 0.0, 1.0)
  widths = lane_width + (width_end - lane_width) * progress
  lane_lines[0, 1, :, 0] = lane_center - widths / 2.0
  lane_lines[0, 2, :, 0] = lane_center + widths / 2.0
  probs = np.zeros((1, 8), dtype=np.float64)
  probs[0, 3] = left_prob
  probs[0, 5] = right_prob
  stds = np.zeros_like(lane_lines)
  return {
    'lane_lines': lane_lines,
    'lane_lines_prob': probs,
    'lane_lines_stds': stds,
    'desire_state': np.zeros((1, ModelConstants.DESIRE_LEN), dtype=np.float64),
  }


class TestAnchoredLaneOffset(unittest.TestCase):
  def setUp(self):
    modeld.reset_anchored_lane_offset()

  def apply_for(self, output: dict[str, np.ndarray], seconds: float, e2e_curvature: float = 0.0,
                blinkers_active: bool = False) -> float:
    result = e2e_curvature
    for _ in range(max(1, int(np.ceil(seconds / modeld.DT_MDL)))):
      result = modeld.apply_anchored_lane_offset(output, e2e_curvature, 20.0,
                                                  blinkers_active=blinkers_active, lane_policy_enabled=True)
    return result

  def arm(self, output: dict[str, np.ndarray] | None = None) -> dict[str, np.ndarray]:
    output = make_model_output() if output is None else output
    self.apply_for(output, modeld.ANCHOR_LANE_ARM_TIME + 1.0)
    self.assertTrue(modeld._anchor_lane_active)
    return output

  def test_disabled_returns_exact_e2e(self):
    self.assertEqual(modeld.apply_anchored_lane_offset(make_model_output(), 0.0123, 20.0,
                                                        lane_policy_enabled=False), 0.0123)

  def test_unset_toggle_defaults_on(self):
    class FakeParams:
      def __init__(self, value):
        self.value = value

      def get(self, key):
        self.assertEqual(key, modeld.ANCHOR_LANE_POLICY_ENABLED_PARAM)
        return self.value

      def assertEqual(self, a, b):
        if a != b:
          raise AssertionError((a, b))

    self.assertTrue(modeld.get_anchor_lane_policy_enabled(FakeParams(None)))
    self.assertFalse(modeld.get_anchor_lane_policy_enabled(FakeParams(False)))

  def test_uses_current_eight_wide_probability_indices(self):
    output = make_model_output(0.97, 0.96)
    output['lane_lines_prob'][0, 1] = 0.01
    fit = (np.asarray(ModelConstants.X_IDXS) >= 5.0) & (np.asarray(ModelConstants.X_IDXS) <= 35.0)
    self.assertEqual(modeld.get_anchor_lane_line_confidences(output, fit), (0.97, 0.96))

  def test_two_lines_anchor_without_any_plan_field(self):
    # The anchor itself never reads model_output['plan']; path availability is
    # not a policy fallback condition.
    output = self.arm(make_model_output(lane_center=0.45))
    curvature = self.apply_for(output, 1.0)
    self.assertTrue(modeld._anchor_lane_active)
    self.assertGreater(curvature, 0.0)
    self.assertLessEqual(curvature, modeld.ANCHOR_LANE_MAX_CORRECTION)

  def test_one_line_cannot_bypass_two_line_arm(self):
    left_only = make_model_output(left_prob=0.99, right_prob=0.10, lane_center=0.20)
    result = modeld.apply_anchored_lane_offset(left_only, 0.001, 20.0, lane_policy_enabled=True)
    self.assertEqual(result, 0.001)
    self.assertFalse(modeld._anchor_lane_holding)

  def test_one_line_uses_learned_width_without_relearning(self):
    self.arm()
    learned_width = modeld._anchor_lane_width
    left_only = make_model_output(left_prob=0.99, right_prob=0.10, lane_center=0.20)
    # A bad/missing opposite line must not change the stored lane-width estimate.
    left_only['lane_lines'][0, 2, :, 0] = 20.0
    curvature = self.apply_for(left_only, 0.25)
    self.assertTrue(modeld._anchor_lane_holding)
    self.assertAlmostEqual(modeld._anchor_lane_width, learned_width)
    self.assertGreater(curvature, 0.0)

  def test_one_line_hold_is_temporary_then_returns_to_e2e(self):
    self.arm()
    left_only = make_model_output(left_prob=0.99, right_prob=0.10, lane_center=0.20)
    e2e = 0.001
    self.apply_for(left_only, modeld.ANCHOR_LANE_FULL_HOLD_TIME * 0.5, e2e)
    self.assertTrue(modeld._anchor_lane_holding)
    result = self.apply_for(left_only, 3.0, e2e)
    self.assertFalse(modeld._anchor_lane_holding)
    self.assertAlmostEqual(result, e2e, places=5)

  def test_blinker_and_lane_change_return_exact_e2e(self):
    self.arm()
    self.assertEqual(modeld.apply_anchored_lane_offset(make_model_output(), 0.0123, 20.0,
                                                        blinkers_active=True, lane_policy_enabled=True), 0.0123)
    self.arm()
    changing = make_model_output()
    changing['desire_state'][0, log.Desire.laneChangeLeft] = 0.2
    self.assertEqual(modeld.apply_anchored_lane_offset(changing, 0.0123, 20.0,
                                                        lane_policy_enabled=True), 0.0123)

  def test_bad_geometry_does_not_activate(self):
    output = make_model_output(lane_width=3.6, lane_width_end=5.5)
    self.apply_for(output, 2.0)
    self.assertFalse(modeld._anchor_lane_active)
