import unittest

import numpy as np

from openpilot.cereal import log
from openpilot.selfdrive.modeld import modeld
from openpilot.selfdrive.modeld.constants import ModelConstants


def make_model_output(left_prob: float = 0.99, right_prob: float = 0.99, lane_width: float = 3.6,
                      lane_width_end: float | None = None, lane_center: float = 0.0,
                      lane_heading: float = 0.0, lane_quadratic: float = 0.0,
                      e2e_path_center: float = 0.0, e2e_path_heading: float = 0.0,
                      e2e_path_quadratic: float = 0.0) -> dict[str, np.ndarray]:
  x = np.asarray(ModelConstants.X_IDXS, dtype=np.float64)
  lane_lines = np.zeros((1, 4, len(x), 2), dtype=np.float64)
  target_width = lane_width if lane_width_end is None else lane_width_end
  width_progress = np.clip((x - modeld.LANE_LOCK_FIT_START) /
                            (modeld.LANE_LOCK_FIT_END - modeld.LANE_LOCK_FIT_START), 0.0, 1.0)
  widths = lane_width + (target_width - lane_width) * width_progress
  # openpilot lateral coordinates are left-negative and right-positive.
  centerline = lane_center + lane_heading * x + lane_quadratic * x * x
  lane_lines[0, 1, :, 0] = centerline - widths / 2.0
  lane_lines[0, 2, :, 0] = centerline + widths / 2.0
  lane_line_probs = np.zeros((1, 8), dtype=np.float64)
  lane_line_probs[0, 3] = left_prob
  lane_line_probs[0, 5] = right_prob
  plan = np.zeros((1, len(x), ModelConstants.PLAN_WIDTH), dtype=np.float64)
  plan[0, :, 0] = x
  plan[0, :, 1] = e2e_path_center + e2e_path_heading * x + e2e_path_quadratic * x * x
  return {'lane_lines': lane_lines, 'lane_lines_prob': lane_line_probs,
          'desire_state': np.zeros((1, ModelConstants.DESIRE_LEN), dtype=np.float64), 'plan': plan}


class TestLanePolicy(unittest.TestCase):
  def setUp(self):
    modeld.reset_lane_lock()

  def apply_for(self, output: dict[str, np.ndarray], seconds: float, e2e_curvature: float = 0.0,
                blinkers_active: bool = False, lateral_active: bool = False) -> float:
    result = e2e_curvature
    for _ in range(max(1, int(np.ceil(seconds / modeld.DT_MDL)))):
      result = modeld.apply_lane_lock(output, e2e_curvature, 20.0,
                                      blinkers_active=blinkers_active, lane_policy_enabled=True,
                                      lateral_active=lateral_active)
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

  def test_full_center_correction_keeps_e2e_curve_feedforward(self):
    self.arm_lane_policy()
    output = make_model_output(lane_center=0.45)
    e2e = 0.0010
    curvature = modeld.apply_lane_lock(output, e2e, 20.0, lane_policy_enabled=True)
    self.assertTrue(modeld._lane_lock_full_active)
    self.assertGreater(curvature, e2e)
    self.assertLessEqual(curvature - e2e, modeld.LANE_LOCK_MAX_CENTER_CORRECTION)
    # Entry ramps in rather than causing a one-frame steering step.
    self.assertLessEqual(abs(modeld._lane_lock_center_correction),
                         modeld.LANE_LOCK_CORRECTION_ENGAGE_STEP + 1e-12)

    # A smaller target releases faster than it engages, so a curve/lane-change
    # correction does not persist through the midpoint.
    centered = make_model_output(lane_center=0.0)
    previous = modeld._lane_lock_center_correction
    modeld.apply_lane_lock(centered, e2e, 20.0, lane_policy_enabled=True)
    self.assertLessEqual(abs(modeld._lane_lock_center_correction - previous),
                         modeld.LANE_LOCK_CORRECTION_RELEASE_STEP + 1e-12)

  def test_turn_direction_change_releases_opposed_correction(self):
    centered = make_model_output()
    negative_turn = -2.0 * modeld.LANE_LOCK_TURN_CURVATURE
    self.apply_for(centered, modeld.LANE_LOCK_ARM_TIME + modeld.DT_MDL, negative_turn)

    # A steady curve may legitimately need a lane correction opposite E2E;
    # full centering must remain available until E2E changes turn direction.
    output = make_model_output(lane_center=-0.45)
    for _ in range(6):
      modeld.apply_lane_lock(output, negative_turn, 20.0, lane_policy_enabled=True)
    before = modeld._lane_lock_center_correction
    self.assertLess(before, 0.0)

    # When the meaningful E2E turn direction flips, release the stale,
    # opposing correction at the faster release rate rather than carrying it
    # into the new curve.
    positive_turn = 2.0 * modeld.LANE_LOCK_TURN_CURVATURE
    modeld.apply_lane_lock(output, positive_turn, 20.0, lane_policy_enabled=True)
    after = modeld._lane_lock_center_correction
    self.assertGreater(after, before)
    self.assertLessEqual(abs(after - before), modeld.LANE_LOCK_CORRECTION_RELEASE_STEP + 1e-12)

  def test_sharp_curve_lane_target_reversal_settles_before_crossing(self):
    # E2E remains in the same sharp turn while the lane-fit heading flips.
    # Releasing the old correction before accepting the new direction prevents
    # the controller from commanding an immediate steering reversal.
    e2e_curve = 1.5 * modeld.LANE_LOCK_HEADING_CURVATURE_FADE
    self.arm_lane_policy()
    negative_heading = make_model_output(lane_heading=-0.03)
    for _ in range(6):
      modeld.apply_lane_lock(negative_heading, e2e_curve, 20.0, lane_policy_enabled=True)
    before = modeld._lane_lock_center_correction
    self.assertLess(before, 0.0)

    positive_heading = make_model_output(lane_heading=0.03)
    modeld.apply_lane_lock(positive_heading, e2e_curve, 20.0, lane_policy_enabled=True)
    self.assertGreater(modeld._lane_lock_curve_reversal_hold_time, 0.0)
    self.assertLessEqual(modeld._lane_lock_center_correction, 0.0)

    for _ in range(int(np.ceil(modeld.LANE_LOCK_CURVE_REVERSAL_HOLD_TIME / modeld.DT_MDL)) + 2):
      if modeld._lane_lock_curve_reversal_hold_time <= 0.0:
        break
      modeld.apply_lane_lock(positive_heading, e2e_curve, 20.0, lane_policy_enabled=True)
      self.assertLessEqual(modeld._lane_lock_center_correction, 0.0)
    else:
      self.fail("curve-target reversal hold did not expire")

    modeld.apply_lane_lock(positive_heading, e2e_curve, 20.0, lane_policy_enabled=True)
    self.assertGreater(modeld._lane_lock_center_correction, 0.0)


  def test_anchor_gain_is_zero_near_and_full_at_far_lookahead(self):
    self.assertEqual(modeld.get_lane_anchor_blend(0.0), 0.0)
    self.assertEqual(modeld.get_lane_anchor_blend(modeld.LANE_ANCHOR_BLEND_START), 0.0)
    self.assertEqual(modeld.get_lane_anchor_blend(modeld.LANE_ANCHOR_BLEND_END), 1.0)

  def test_anchor_leaves_matching_e2e_curve_unchanged(self):
    curve = 0.0008
    output = make_model_output(lane_quadratic=curve, e2e_path_quadratic=curve)
    self.arm_lane_policy(output)
    e2e = 0.0010
    self.assertAlmostEqual(modeld.apply_lane_lock(output, e2e, 28.0, lane_policy_enabled=True), e2e, places=6)

  def test_matching_paths_have_no_base_correction_without_active_feedback(self):
    output = make_model_output(lane_center=0.30, e2e_path_center=0.30)
    self.arm_lane_policy(output)
    e2e = 0.0010
    self.assertAlmostEqual(modeld.apply_lane_lock(output, e2e, 28.0, lane_policy_enabled=True), e2e, places=6)

  def test_persistent_vehicle_offset_is_corrected_even_when_paths_match(self):
    for sign in (-1, 1):
      with self.subTest(sign=sign):
        modeld.reset_lane_lock()
        output = make_model_output(lane_center=sign * 0.08, e2e_path_center=sign * 0.08)
        self.arm_lane_policy(output)
        early = self.apply_for(output, 1.0, lateral_active=True)
        self.assertAlmostEqual(early, 0.0)
        later = self.apply_for(output, 12.0, lateral_active=True)
        self.assertGreater(sign * later, 0.0)
        self.assertLessEqual(abs(later), modeld.LANE_ANCHOR_BIAS_MAX)

  def test_near_geometry_separates_position_and_heading_from_road_curve(self):
    x = np.asarray(ModelConstants.X_IDXS)
    offset, heading = modeld.get_lane_anchor_near_geometry(0.08 - 0.002 * x + 0.001 * x ** 2, x)
    self.assertAlmostEqual(offset, 0.08)
    self.assertAlmostEqual(heading, -0.002)

  def test_approach_easing_is_symmetric_and_keeps_parallel_centering(self):
    for sign in (-1, 1):
      with self.subTest(sign=sign):
        self.assertEqual(modeld.get_lane_anchor_approach_scale(sign * 0.08, 0.0, 25.0), 1.0)
        self.assertEqual(modeld.get_lane_anchor_approach_scale(sign * 0.08, sign * 0.003, 25.0), 1.0)
        easing = modeld.get_lane_anchor_approach_scale(sign * 0.08, -sign * 0.003, 25.0)
        self.assertLess(easing, 1.0)
        self.assertGreaterEqual(easing, 1.0 - modeld.LANE_ANCHOR_APPROACH_MAX_FRACTION)

  def test_bias_does_not_build_during_fast_convergence_or_intermittent_error(self):
    for _ in range(200):
      modeld.update_lane_anchor_bias(0.10, -0.01, 25.0, 0.0, True)
    self.assertEqual(modeld._lane_lock_center_bias, 0.0)
    for _ in range(10):
      for _ in range(20):
        modeld.update_lane_anchor_bias(0.10, 0.0, 25.0, 0.0, True)
      modeld.update_lane_anchor_bias(0.0, 0.0, 25.0, 0.0, True)
    self.assertEqual(modeld._lane_lock_center_bias, 0.0)

  def test_bias_holds_at_center_and_unwinds_before_reversing(self):
    output = make_model_output(lane_center=0.08, e2e_path_center=0.08)
    self.apply_for(output, 15.0, lateral_active=True)
    learned = modeld._lane_lock_center_bias
    self.assertGreater(learned, 0.0)
    for _ in range(100):
      modeld.update_lane_anchor_bias(0.0, 0.0, 20.0, 0.0, True)
    self.assertEqual(modeld._lane_lock_center_bias, learned)
    modeld.update_lane_anchor_bias(-0.08, 0.0, 20.0, 0.0, True)
    self.assertGreaterEqual(modeld._lane_lock_center_bias, 0.0)
    self.assertLess(modeld._lane_lock_center_bias, learned)
    for _ in range(100):
      if modeld._lane_lock_center_bias == 0.0:
        break
      modeld.update_lane_anchor_bias(-0.08, 0.0, 20.0, 0.0, True)
    self.assertEqual(modeld._lane_lock_center_bias, 0.0)
    self.assertEqual(modeld._lane_lock_bias_arm_time, 0.0)
    for _ in range(20):
      modeld.update_lane_anchor_bias(-0.08, 0.0, 20.0, 0.0, True)
    self.assertEqual(modeld._lane_lock_center_bias, 0.0)

  def test_bias_releases_before_predicted_crossing_and_within_build_deadband(self):
    for sign in (-1, 1):
      with self.subTest(sign=sign):
        modeld.reset_lane_lock()
        output = make_model_output(lane_center=sign * 0.08, e2e_path_center=sign * 0.08)
        self.apply_for(output, 12.0, lateral_active=True)
        learned = abs(modeld._lane_lock_center_bias)
        modeld.update_lane_anchor_bias(sign * 0.01, -sign * 0.004, 20.0, 0.0, True)
        anticipating = abs(modeld._lane_lock_center_bias)
        self.assertLess(anticipating, learned)
        modeld.update_lane_anchor_bias(-sign * 0.005, 0.0, 20.0, 0.0, True)
        self.assertLess(abs(modeld._lane_lock_center_bias), anticipating)

  def test_bias_and_total_correction_are_bounded_without_windup(self):
    for sign in (-1, 1):
      with self.subTest(sign=sign):
        modeld.reset_lane_lock()
        for _ in range(1000):
          bias = modeld.update_lane_anchor_bias(sign * 0.20, 0.0, 20.0, 0.0, True)
        self.assertAlmostEqual(bias, sign * modeld.LANE_ANCHOR_BIAS_MAX)
        modeld.reset_lane_anchor_bias()
        for _ in range(200):
          bias = modeld.update_lane_anchor_bias(sign * 0.20, 0.0, 20.0,
                                                sign * modeld.LANE_LOCK_MAX_CENTER_CORRECTION, True)
        self.assertEqual(bias, 0.0)
        output = make_model_output(lane_center=sign * 0.20, e2e_path_center=-sign * 0.80)
        result = self.apply_for(output, 10.0, lateral_active=True)
        self.assertLessEqual(abs(result), modeld.LANE_LOCK_MAX_CENTER_CORRECTION)
        self.assertEqual(modeld._lane_lock_center_bias, 0.0)

  def test_bias_clears_on_override_disengagement_and_unreliable_geometry(self):
    cases = [dict(lateral_active=False), dict(steering_pressed=True),
             dict(left_prob=0.80), dict(right_prob=0.10), dict(v_ego=5.0),
             dict(e2e_curvature=0.005), dict(lane_center=0.50)]
    for case in cases:
      with self.subTest(case=case):
        modeld.reset_lane_lock()
        output = make_model_output(lane_center=0.08, e2e_path_center=0.08)
        self.apply_for(output, 12.0, lateral_active=True)
        self.assertGreater(modeld._lane_lock_center_bias, 0.0)
        geometry = {k: v for k, v in case.items() if k in ('left_prob', 'right_prob', 'lane_center')}
        options = dict(e2e_curvature=0.0, v_ego=20.0, lateral_active=True, lane_policy_enabled=True)
        options.update({k: v for k, v in case.items() if k not in geometry})
        modeld.apply_lane_lock(make_model_output(**geometry), **options)
        self.assertEqual(modeld._lane_lock_center_bias, 0.0)
        self.assertEqual(modeld._lane_lock_bias_arm_time, 0.0)

  def test_fallback_is_exact_e2e_and_discards_learned_bias(self):
    for reason in ('disabled', 'blinker', 'lane_change', 'low_confidence', 'bad_near_geometry'):
      with self.subTest(reason=reason):
        modeld.reset_lane_lock()
        output = make_model_output(lane_center=0.08, e2e_path_center=0.08)
        self.apply_for(output, 12.0, lateral_active=True)
        self.assertGreater(modeld._lane_lock_center_bias, 0.0)
        if reason == 'lane_change':
          output['desire_state'][0, log.Desire.laneChangeRight] = 0.2
        elif reason == 'low_confidence':
          output['lane_lines_prob'][:] = 0.1
        elif reason == 'bad_near_geometry':
          output['lane_lines'][0, 1, 0, 0] = np.nan
        result = modeld.apply_lane_lock(output, -0.001, 20.0, lane_policy_enabled=reason != 'disabled',
                                        blinkers_active=reason == 'blinker', lateral_active=True)
        self.assertEqual(result, -0.001)
        self.assertEqual(modeld._lane_lock_center_bias, 0.0)
        self.assertFalse(modeld._lane_lock_full_active)

  def test_turn_guard_clears_bias_instead_of_bypassing_release(self):
    output = make_model_output(lane_center=0.08, e2e_path_center=0.08)
    self.apply_for(output, 12.0, e2e_curvature=-0.0003, lateral_active=True)
    self.assertGreater(modeld._lane_lock_center_bias, 0.0)
    modeld.apply_lane_lock(output, 0.0003, 20.0, lane_policy_enabled=True, lateral_active=True)
    self.assertGreater(modeld._lane_lock_turn_release_time, 0.0)
    self.assertEqual(modeld._lane_lock_center_bias, 0.0)

  def test_short_plan_keeps_original_fit_without_extrapolation(self):
    output = make_model_output(lane_center=0.20)
    x = np.asarray(ModelConstants.X_IDXS)
    output['plan'] = output['plan'][:, x <= 30.0, :]
    self.arm_lane_policy(output)
    result = self.apply_for(output, 1.0)
    self.assertGreater(result, 0.0)
    self.assertTrue(modeld._lane_lock_full_active)
    output['plan'] = output['plan'][:, :4, :]
    self.assertEqual(modeld.apply_lane_lock(output, 0.001, 20.0, lane_policy_enabled=True), 0.001)

  def test_invalid_e2e_path_returns_exact_e2e(self):
    self.arm_lane_policy()
    output = make_model_output(lane_center=0.35)
    output['plan'][:] = np.nan
    e2e = -0.0012
    self.assertEqual(modeld.apply_lane_lock(output, e2e, 20.0, lane_policy_enabled=True), e2e)
    self.assertFalse(modeld._lane_lock_full_active)

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

  def test_one_line_hold_uses_learned_width(self):
    self.arm_lane_policy()
    one_line = make_model_output(left_prob=0.99, right_prob=0.10, lane_center=0.35)
    curvature = self.apply_for(one_line, 0.50)
    self.assertTrue(modeld._lane_lock_full_active)
    self.assertTrue(modeld._lane_lock_one_line_hold)
    self.assertGreater(curvature, 0.0)

  def test_one_line_hold_expires_to_exact_e2e(self):
    self.arm_lane_policy()
    one_line = make_model_output(left_prob=0.99, right_prob=0.10, lane_center=0.35)
    e2e = -0.0012
    result = self.apply_for(one_line, modeld.LANE_LOCK_ONE_LINE_HOLD_TIME + 2.0 * modeld.DT_MDL, e2e)
    self.assertEqual(result, e2e)
    self.assertFalse(modeld._lane_lock_full_active)
    self.assertFalse(modeld._lane_lock_ready)

  def test_blinker_releases_lane_lock(self):
    output = self.arm_lane_policy()
    self.assertEqual(modeld.apply_lane_lock(output, 0.0123, 20.0, blinkers_active=True, lane_policy_enabled=True), 0.0123)
    self.assertFalse(modeld._lane_lock_full_active)

  def test_lane_change_intent_releases_lane_lock(self):
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
