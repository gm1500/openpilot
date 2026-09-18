import unittest

import numpy as np

from openpilot.cereal import log
from openpilot.selfdrive.modeld import modeld
from openpilot.selfdrive.modeld.constants import ModelConstants


def make_model_output(left_prob: float = 0.99, right_prob: float = 0.99, lane_width: float = 3.6,
                      lane_width_end: float | None = None, lane_center: float = 0.0,
                      lane_heading: float = 0.0) -> dict[str, np.ndarray]:
  x = np.asarray(ModelConstants.X_IDXS, dtype=np.float64)
  lane_lines = np.zeros((1, 4, len(x), 2), dtype=np.float64)
  target_width = lane_width if lane_width_end is None else lane_width_end
  width_progress = np.clip((x - modeld.LANE_LOCK_FIT_START) /
                            (modeld.LANE_LOCK_FIT_END - modeld.LANE_LOCK_FIT_START), 0.0, 1.0)
  widths = lane_width + (target_width - lane_width) * width_progress
  # openpilot lateral coordinates are left-negative and right-positive.
  centerline = lane_center + lane_heading * x
  lane_lines[0, 1, :, 0] = centerline - widths / 2.0
  lane_lines[0, 2, :, 0] = centerline + widths / 2.0
  lane_line_probs = np.zeros((1, 8), dtype=np.float64)
  lane_line_probs[0, 3] = left_prob
  lane_line_probs[0, 5] = right_prob
  plan = np.zeros((1, len(x), ModelConstants.PLAN_WIDTH), dtype=np.float64)
  plan[0, :, 0] = x
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
    self.assertTrue(modeld._lane_lock_full_active)
    self.assertTrue(modeld._lane_lock_width_valid)
    return output

  def test_disabled_mode_returns_exact_e2e_target(self):
    self.assertEqual(modeld.apply_lane_lock(make_model_output(), 0.0123, 20.0, lane_policy_enabled=False), 0.0123)

  def test_selector_decodes_params_bytes_and_defaults_on_when_unset(self):
    class FakeParams:
      def __init__(self, value):
        self.value = value

      def get(self, key):
        assert key == modeld.LANE_POLICY_ENABLED_PARAM
        return self.value

    self.assertTrue(modeld.get_lane_policy_enabled(FakeParams(None)))
    self.assertTrue(modeld.get_lane_policy_enabled(FakeParams(b"1")))
    self.assertFalse(modeld.get_lane_policy_enabled(FakeParams(b"0")))
    self.assertFalse(modeld.get_lane_policy_enabled(FakeParams(False)))

  def test_raw_probability_indices(self):
    output = make_model_output(0.97, 0.96)
    output['lane_lines_prob'][0, 1] = 0.01
    self.assertEqual(modeld.get_inner_lane_line_probs(output), (0.97, 0.96))

  def test_arms_only_after_clean_two_line_timer(self):
    output = make_model_output()
    self.apply_for(output, modeld.LANE_LOCK_ARM_TIME - modeld.DT_MDL)
    self.assertFalse(modeld._lane_lock_ready)
    self.assertEqual(modeld._lane_lock_weight, 0.0)
    self.apply_for(output, 2.0 * modeld.DT_MDL)
    self.assertTrue(modeld._lane_lock_full_active)
    self.assertEqual(modeld._lane_lock_weight, 1.0)

  def test_stable_centering_keeps_e2e_curve_feedforward(self):
    self.arm_lane_policy()
    output = make_model_output(lane_center=0.45)
    e2e = 0.0010
    curvature = modeld.apply_lane_lock(output, e2e, 20.0, lane_policy_enabled=True)
    self.assertTrue(modeld._lane_lock_full_active)
    self.assertGreater(curvature, e2e)
    self.assertLessEqual(curvature - e2e, modeld.LANE_LOCK_MAX_CENTER_CORRECTION)
    self.assertLessEqual(abs(modeld._lane_lock_center_correction),
                         modeld.LANE_LOCK_CENTER_CORRECTION_STEP + 1e-12)

  def test_straight_road_reaches_full_center_authority(self):
    self.arm_lane_policy()
    offset_lane = make_model_output(lane_center=0.45)
    curvature = self.apply_for(offset_lane, 2.0)
    self.assertGreater(curvature, 0.00035)
    self.assertLessEqual(curvature, modeld.LANE_LOCK_MAX_CENTER_CORRECTION)

  def test_opposite_target_cannot_immediately_reverse_e2e_turn(self):
    self.arm_lane_policy()
    # A negative lane target is allowed, but it must first persist while E2E
    # remains in a positive turn. The initial limit preserves E2E direction.
    output = make_model_output(lane_center=-0.45)
    e2e = 0.00018
    guarded_frames = int(np.ceil(modeld.LANE_LOCK_OPPOSING_CONFIRM_TIME / modeld.DT_MDL)) - 1
    for _ in range(guarded_frames):
      curvature = modeld.apply_lane_lock(output, e2e, 20.0, lane_policy_enabled=True)
      self.assertGreater(curvature, 0.0)
    self.assertLessEqual(abs(modeld._lane_lock_center_correction),
                         modeld.LANE_LOCK_OPPOSING_E2E_FRACTION * abs(e2e) + 1e-12)

    # After temporal confirmation, full centering is still available.
    curvature = self.apply_for(output, 1.0, e2e)
    self.assertLess(curvature, e2e)

  def test_strong_e2e_turn_keeps_road_shape_direction(self):
    self.arm_lane_policy()
    output = make_model_output(lane_center=-0.45)
    e2e = 1.2 * modeld.LANE_LOCK_STRONG_E2E_CURVATURE
    for _ in range(30):
      curvature = modeld.apply_lane_lock(output, e2e, 20.0, lane_policy_enabled=True)
      self.assertGreater(curvature, 0.0)
    self.assertLessEqual(abs(modeld._lane_lock_center_correction),
                         modeld.LANE_LOCK_OPPOSING_E2E_FRACTION * abs(e2e) + 1e-12)

  def test_correction_crosses_zero_before_reversing(self):
    self.arm_lane_policy()
    positive_lane = make_model_output(lane_center=0.45)
    self.apply_for(positive_lane, 1.0)
    self.assertGreater(modeld._lane_lock_center_correction, 0.0)

    negative_lane = make_model_output(lane_center=-0.45)
    previous = modeld._lane_lock_center_correction
    for _ in range(20):
      modeld.apply_lane_lock(negative_lane, 0.0, 20.0, lane_policy_enabled=True)
      current = modeld._lane_lock_center_correction
      self.assertLessEqual(abs(current - previous), modeld.LANE_LOCK_CENTER_CORRECTION_STEP + 1e-12)
      if current < 0.0:
        self.assertLessEqual(previous, modeld.LANE_LOCK_CORRECTION_DEADBAND)
        break
      previous = current
    else:
      self.fail("correction did not settle through zero")

  def test_one_line_hold_filters_a_changed_measurement(self):
    self.arm_lane_policy()
    stable = make_model_output(lane_center=0.35)
    self.apply_for(stable, 1.0)
    before = modeld._lane_lock_center_correction

    # A right-line dropout with a sharply different left-line fit must not
    # snap the command or reverse it in one frame.
    one_line = make_model_output(left_prob=0.99, right_prob=0.10, lane_center=-0.45)
    modeld.apply_lane_lock(one_line, 0.0, 20.0, lane_policy_enabled=True)
    after = modeld._lane_lock_center_correction
    self.assertTrue(modeld._lane_lock_one_line_hold)
    self.assertGreater(after, 0.0)
    self.assertLessEqual(abs(after - before), modeld.LANE_LOCK_CENTER_CORRECTION_STEP + 1e-12)

  def test_hysteresis_retains_full_center_above_exit_threshold(self):
    self.arm_lane_policy()
    reduced_confidence = make_model_output(left_prob=0.80, right_prob=0.80, lane_center=0.25)
    curvature = modeld.apply_lane_lock(reduced_confidence, 0.001, 20.0, lane_policy_enabled=True)
    self.assertTrue(modeld._lane_lock_full_active)
    self.assertFalse(modeld._lane_lock_one_line_hold)
    self.assertGreater(curvature, 0.001)

  def test_below_exit_confidence_releases_to_exact_e2e(self):
    self.arm_lane_policy()
    low_confidence = make_model_output(left_prob=0.65, right_prob=0.65)
    self.assertEqual(modeld.apply_lane_lock(low_confidence, -0.0012, 20.0, lane_policy_enabled=True), -0.0012)
    self.assertFalse(modeld._lane_lock_full_active)

  def test_one_line_hold_expires_to_exact_e2e(self):
    self.arm_lane_policy()
    one_line = make_model_output(left_prob=0.99, right_prob=0.10, lane_center=0.35)
    e2e = -0.0012
    result = self.apply_for(one_line, modeld.LANE_LOCK_ONE_LINE_HOLD_TIME + 2.0 * modeld.DT_MDL, e2e)
    self.assertEqual(result, e2e)
    self.assertFalse(modeld._lane_lock_full_active)

  def test_blinker_and_lane_change_intent_release_lane_lock(self):
    output = self.arm_lane_policy()
    self.assertEqual(modeld.apply_lane_lock(output, 0.0123, 20.0, blinkers_active=True, lane_policy_enabled=True), 0.0123)
    self.assertFalse(modeld._lane_lock_full_active)

    self.arm_lane_policy()
    output = make_model_output()
    output['desire_state'][0, log.Desire.laneChangeLeft] = 0.2
    self.assertEqual(modeld.apply_lane_lock(output, 0.0123, 20.0, lane_policy_enabled=True), 0.0123)
    self.assertFalse(modeld._lane_lock_full_active)

  def test_robust_geometry_accepts_normal_taper_and_rejects_extreme_taper(self):
    self.arm_lane_policy(make_model_output(lane_width=3.6, lane_width_end=4.1))
    self.assertTrue(modeld._lane_lock_full_active)

    modeld.reset_lane_lock()
    output = make_model_output(lane_width=3.6, lane_width_end=5.0)
    self.apply_for(output, modeld.LANE_LOCK_ARM_TIME + modeld.DT_MDL)
    self.assertFalse(modeld._lane_lock_full_active)


if __name__ == "__main__":
  unittest.main()
