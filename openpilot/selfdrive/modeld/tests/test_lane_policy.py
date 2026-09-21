import unittest

import numpy as np

from openpilot.selfdrive.modeld import lane_policy
from openpilot.selfdrive.modeld.constants import ModelConstants


def make_model_output(left_prob: float = 0.99, right_prob: float = 0.99, lane_width: float = 3.6,
                      lane_width_end: float | None = None, lane_center: float = 0.0,
                      lane_heading: float = 0.0, lane_quadratic: float = 0.0,
                      e2e_path_center: float = 0.0, e2e_path_heading: float = 0.0,
                      e2e_path_quadratic: float = 0.0) -> dict[str, np.ndarray]:
  x = np.asarray(ModelConstants.X_IDXS, dtype=np.float64)
  lane_lines = np.zeros((1, 4, len(x), 2), dtype=np.float64)
  target_width = lane_width if lane_width_end is None else lane_width_end
  width_progress = np.clip((x - lane_policy.WIDTH_FIT_START) /
                            (lane_policy.WIDTH_FIT_END - lane_policy.WIDTH_FIT_START), 0.0, 1.0)
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
    self.policy = lane_policy.LanePolicy()

  def test_policy_instances_do_not_share_steering_history(self):
    other = lane_policy.LanePolicy()
    output = make_model_output(lane_center=0.08, e2e_path_center=0.08)
    self.apply_for(output, 12.0, lateral_active=True)
    self.assertGreater(self.policy.bias, 0.0)
    self.assertFalse(other.active)
    self.assertEqual(other.bias, 0.0)
    self.assertIsNone(other.motion_time)
    self.assertEqual(other.update(output, -0.001, 20.0), -0.001)
    self.assertGreater(self.policy.bias, 0.0)

  def test_hud_status_tracks_arming_active_hold_and_fallback(self):
    output = make_model_output()
    self.apply_for(output, 0.5)
    self.assertFalse(self.policy.active)
    self.assertTrue(self.policy.blending)
    self.apply_for(output, 0.5)
    self.assertTrue(self.policy.active)
    self.assertFalse(self.policy.blending)
    self.apply_for(make_model_output(right_prob=0.10), 0.5)
    self.assertTrue(self.policy.active)
    self.assertTrue(self.policy.blending)
    self.policy.update(output, 0.001, 20.0, lane_policy_enabled=False)
    self.assertFalse(self.policy.active)
    self.assertFalse(self.policy.blending)

  def apply_for(self, output: dict[str, np.ndarray], seconds: float, e2e_curvature: float = 0.0,
                blinkers_active: bool = False, lateral_active: bool = False) -> float:
    result = e2e_curvature
    for _ in range(max(1, int(np.ceil(seconds / lane_policy.DT)))):
      result = self.policy.update(output, e2e_curvature, 20.0,
                                      blinkers_active=blinkers_active, lane_policy_enabled=True,
                                      lateral_active=lateral_active)
    return result

  def arm_lane_policy(self, output: dict[str, np.ndarray] | None = None) -> dict[str, np.ndarray]:
    output = make_model_output() if output is None else output
    self.apply_for(output, lane_policy.ARM_TIME + lane_policy.DT)
    self.assertTrue(self.policy.active)
    self.assertIsNotNone(self.policy.width)
    return output

  def test_disabled_mode_returns_exact_e2e_target(self):
    self.assertEqual(self.policy.update(make_model_output(), 0.0123, 20.0, lane_policy_enabled=False), 0.0123)

  def test_selector_decodes_params_bytes_and_defaults_on_when_unset(self):
    self.assertTrue(lane_policy.is_enabled(None))
    self.assertTrue(lane_policy.is_enabled(b"1"))
    self.assertFalse(lane_policy.is_enabled(b"0"))
    self.assertFalse(lane_policy.is_enabled(False))

  def test_raw_probability_indices(self):
    output = make_model_output(0.97, 0.96)
    output['lane_lines_prob'][0, 1] = 0.01
    self.assertEqual(lane_policy.inner_probs(output), (0.97, 0.96))

  def test_arms_only_after_clean_two_line_timer(self):
    output = make_model_output()
    self.apply_for(output, lane_policy.ARM_TIME - lane_policy.DT)
    self.assertFalse(self.policy.active)
    self.apply_for(output, 2.0 * lane_policy.DT)
    self.assertTrue(self.policy.active)

  def test_full_center_correction_keeps_e2e_curve_feedforward(self):
    self.arm_lane_policy()
    output = make_model_output(lane_center=0.45)
    e2e = 0.0010
    curvature = self.policy.update(output, e2e, 20.0, lane_policy_enabled=True)
    self.assertTrue(self.policy.active)
    self.assertGreater(curvature, e2e)
    self.assertLessEqual(curvature - e2e, lane_policy.MAX_CORRECTION)
    # Entry ramps in rather than causing a one-frame steering step.
    self.assertLessEqual(abs(self.policy.correction),
                         lane_policy.ENGAGE_STEP + 1e-12)

    # A smaller target releases faster than it engages, so a curve/lane-change
    # correction does not persist through the midpoint.
    centered = make_model_output(lane_center=0.0)
    previous = self.policy.correction
    self.policy.update(centered, e2e, 20.0, lane_policy_enabled=True)
    self.assertLessEqual(abs(self.policy.correction - previous),
                         lane_policy.RELEASE_STEP + 1e-12)

  def test_turn_direction_change_releases_opposed_correction(self):
    centered = make_model_output()
    negative_turn = -2.0 * lane_policy.TURN_THRESHOLD
    self.apply_for(centered, lane_policy.ARM_TIME + lane_policy.DT, negative_turn)

    # A steady curve may legitimately need a lane correction opposite E2E;
    # full centering must remain available until E2E changes turn direction.
    output = make_model_output(lane_center=-0.45)
    for _ in range(6):
      self.policy.update(output, negative_turn, 20.0, lane_policy_enabled=True)
    before = self.policy.correction
    self.assertLess(before, 0.0)

    # When the meaningful E2E turn direction flips, release the stale,
    # opposing correction at the faster release rate rather than carrying it
    # into the new curve.
    positive_turn = 2.0 * lane_policy.TURN_THRESHOLD
    self.policy.update(output, positive_turn, 20.0, lane_policy_enabled=True)
    after = self.policy.correction
    self.assertGreater(after, before)
    self.assertLessEqual(abs(after - before), lane_policy.RELEASE_STEP + 1e-12)

  def test_sharp_curve_lane_target_reversal_settles_before_crossing(self):
    # E2E remains in the same sharp turn while the lane-fit heading flips.
    # Releasing the old correction before accepting the new direction prevents
    # the controller from commanding an immediate steering reversal.
    e2e_curve = 1.5 * lane_policy.CURVE_THRESHOLD
    self.arm_lane_policy()
    negative_heading = make_model_output(lane_heading=-0.03)
    for _ in range(6):
      self.policy.update(negative_heading, e2e_curve, 20.0, lane_policy_enabled=True)
    before = self.policy.correction
    self.assertLess(before, 0.0)

    positive_heading = make_model_output(lane_heading=0.03)
    self.policy.update(positive_heading, e2e_curve, 20.0, lane_policy_enabled=True)
    self.assertGreater(self.policy.curve_release, 0.0)
    self.assertLessEqual(self.policy.correction, 0.0)

    for _ in range(int(np.ceil(lane_policy.CURVE_REVERSAL_TIME / lane_policy.DT)) + 2):
      if self.policy.curve_release <= 0.0:
        break
      self.policy.update(positive_heading, e2e_curve, 20.0, lane_policy_enabled=True)
      self.assertLessEqual(self.policy.correction, 0.0)
    else:
      self.fail("curve-target reversal hold did not expire")

    self.policy.update(positive_heading, e2e_curve, 20.0, lane_policy_enabled=True)
    self.assertGreater(self.policy.correction, 0.0)


  def test_anchor_gain_is_zero_near_and_full_at_far_lookahead(self):
    self.assertEqual(lane_policy.anchor_gain(0.0), 0.0)
    self.assertEqual(lane_policy.anchor_gain(lane_policy.ANCHOR_BLEND_START), 0.0)
    self.assertEqual(lane_policy.anchor_gain(lane_policy.ANCHOR_BLEND_END), 1.0)

  def test_anchor_leaves_matching_e2e_curve_unchanged(self):
    curve = 0.0008
    output = make_model_output(lane_quadratic=curve, e2e_path_quadratic=curve)
    self.arm_lane_policy(output)
    e2e = 0.0010
    self.assertAlmostEqual(self.policy.update(output, e2e, 28.0, lane_policy_enabled=True), e2e, places=6)

  def test_whole_path_alignment_removes_affine_disagreement(self):
    x = np.asarray(ModelConstants.X_IDXS)
    x = x[x <= lane_policy.ANCHOR_FIT_END]
    e2e = 0.0003 * x ** 2 + 0.02 * np.sin(x / 12.0)
    for side in (-1, 1):
      center = e2e + side * (0.12 + 0.003 * x)
      target, _ = lane_policy.blend_path(center, e2e, x)
      np.testing.assert_allclose(target, center, atol=1e-12)

  def test_virtual_path_keeps_near_e2e_shape_and_reaches_far_lane(self):
    x = np.asarray(ModelConstants.X_IDXS)
    x = x[x <= lane_policy.ANCHOR_FIT_END]
    e2e = 0.0004 * x ** 2
    center = 0.08 + 0.002 * x + 0.0001 * x ** 2
    target, _ = lane_policy.blend_path(center, e2e, x)
    near = x <= lane_policy.ANCHOR_BLEND_START
    # The near transformation is affine, so it adds no second derivative.
    near_change = np.polyfit(x[near], target[near] - e2e[near], 2)
    self.assertAlmostEqual(near_change[0], 0.0, places=12)
    far = x >= lane_policy.ANCHOR_BLEND_END
    np.testing.assert_allclose(target[far], center[far], atol=1e-12)
    np.testing.assert_array_equal(e2e, 0.0004 * x ** 2)

  def test_distributed_fit_preserves_parallel_centering_gain(self):
    x = np.asarray(ModelConstants.X_IDXS)
    for speed in (5.0, 16.0, 20.0, 25.0, 35.0):
      for horizon in (30.0, 45.0, 60.0):
        for displacement in (-0.20, 0.20):
          with self.subTest(speed=speed, horizon=horizon, displacement=displacement):
            output = make_model_output(lane_center=displacement)
            output['plan'] = output['plan'][:, x <= horizon, :]
            center = np.full_like(x, displacement)
            correction, _, lookahead = lane_policy.anchor_correction(output, center, x, speed)
            expected = 2.0 * lane_policy.anchor_gain(lookahead) * displacement / lookahead ** 2
            self.assertAlmostEqual(correction, expected, places=12)

  def test_path_hugging_still_produces_opposing_correction(self):
    x = np.asarray(ModelConstants.X_IDXS)
    for side in (-1, 1):
      output = make_model_output(e2e_path_center=side * 0.80)
      correction, _, _ = lane_policy.anchor_correction(output, np.zeros_like(x), x, 30.0)
      self.assertAlmostEqual(correction, -side * lane_policy.MAX_CORRECTION)

  def test_far_shape_inside_fit_influences_correction(self):
    x = np.asarray(ModelConstants.X_IDXS)
    output = make_model_output()
    center = 0.20 * np.clip((x - 40.0) / 15.0, 0.0, 1.0)
    correction, _, _ = lane_policy.anchor_correction(output, center, x, 30.0)
    self.assertGreater(correction, lane_policy.CORRECTION_DEADBAND)
    # This shape was invisible to v3's 8-40 m alignment fit.
    legacy, _, _ = lane_policy.anchor_correction(output, center, x, 30.0, path_blend=False)
    self.assertEqual(legacy, 0.0)

  def test_fit_ignores_geometry_beyond_shared_horizon(self):
    x = np.asarray(ModelConstants.X_IDXS)
    for horizon in (30.0, 45.0, 192.0):
      with self.subTest(horizon=horizon):
        output = make_model_output(lane_center=0.10)
        output['plan'] = output['plan'][:, x <= horizon, :]
        center = np.full_like(x, 0.10)
        expected = lane_policy.anchor_correction(output, center, x, 25.0)
        shared_end = min(lane_policy.ANCHOR_FIT_END, output['plan'][0, -1, 0])
        center[x > shared_end] = np.nan
        actual = lane_policy.anchor_correction(output, center, x, 25.0)
        np.testing.assert_allclose(actual, expected, atol=1e-12)

  def test_one_line_fit_keeps_v3_heading_and_offset_response(self):
    x = np.asarray(ModelConstants.X_IDXS)
    output = make_model_output(e2e_path_center=-0.04, e2e_path_heading=0.001)
    center = 0.08 + 0.003 * x
    correction, _, lookahead = lane_policy.anchor_correction(output, center, x, 20.0, path_blend=False)
    expected = 2.0 * lane_policy.anchor_gain(lookahead) * (0.12 + lane_policy.HEADING_GAIN * 0.002 * lookahead) / lookahead ** 2
    self.assertAlmostEqual(correction, expected, places=12)

  def test_one_line_hold_uses_legacy_fit_for_non_affine_geometry(self):
    output = make_model_output(lane_center=0.08, lane_quadratic=0.0001)
    self.arm_lane_policy(output)
    output['lane_lines_prob'][0, 5] = 0.10
    result = self.apply_for(output, 0.50)
    self.assertTrue(self.policy.holding_line)
    x = np.asarray(ModelConstants.X_IDXS)
    center = output['lane_lines'][0, 1, :, 0] + self.policy.width / 2.0
    legacy, _, _ = lane_policy.anchor_correction(output, center, x, 20.0, path_blend=False)
    self.assertAlmostEqual(result, lane_policy.soften_correction(legacy), places=12)

  def test_distributed_fit_has_symmetric_steering_response(self):
    x = np.asarray(ModelConstants.X_IDXS)
    center = 0.07 - 0.002 * x + 0.0001 * x ** 2
    positive = make_model_output(e2e_path_center=-0.02, e2e_path_quadratic=0.00004)
    negative = make_model_output(e2e_path_center=0.02, e2e_path_quadratic=-0.00004)
    for speed in (10.0, 20.0, 30.0):
      a = lane_policy.anchor_correction(positive, center, x, speed)
      b = lane_policy.anchor_correction(negative, -center, x, speed)
      self.assertAlmostEqual(a[0], -b[0], places=12)

  def test_matching_paths_have_no_base_correction_without_active_feedback(self):
    output = make_model_output(lane_center=0.30, e2e_path_center=0.30)
    self.arm_lane_policy(output)
    e2e = 0.0010
    self.assertAlmostEqual(self.policy.update(output, e2e, 28.0, lane_policy_enabled=True), e2e, places=6)

  def test_persistent_vehicle_offset_is_corrected_even_when_paths_match(self):
    for sign in (-1, 1):
      with self.subTest(sign=sign):
        self.policy.reset()
        output = make_model_output(lane_center=sign * 0.08, e2e_path_center=sign * 0.08)
        self.arm_lane_policy(output)
        early = self.apply_for(output, 1.0, lateral_active=True)
        self.assertAlmostEqual(early, 0.0)
        later = self.apply_for(output, 12.0, lateral_active=True)
        self.assertGreater(sign * later, 0.0)
        self.assertLessEqual(abs(later), lane_policy.BIAS_MAX)

  def test_near_geometry_separates_position_and_heading_from_road_curve(self):
    x = np.asarray(ModelConstants.X_IDXS)
    offset, heading = lane_policy.near_geometry(0.08 - 0.002 * x + 0.001 * x ** 2, x)
    self.assertAlmostEqual(offset, 0.08)
    self.assertAlmostEqual(heading, -0.002)

  def test_approach_easing_is_symmetric_and_keeps_parallel_centering(self):
    for sign in (-1, 1):
      with self.subTest(sign=sign):
        self.assertEqual(lane_policy.approach_scale(sign * 0.08, 0.0), 1.0)
        self.assertEqual(lane_policy.approach_scale(sign * 0.08, sign * 0.075), 1.0)
        easing = lane_policy.approach_scale(sign * 0.08, -sign * 0.075)
        self.assertLess(easing, 1.0)
        self.assertGreaterEqual(easing, 1.0 - lane_policy.APPROACH_MAX_FRACTION)

  def test_motion_estimate_rejects_false_convergence_at_fixed_offset(self):
    for sign in (-1, 1):
      with self.subTest(sign=sign):
        self.policy.reset()
        for frame in range(300):
          rate, ready = self.policy.update_motion(sign * 0.08, -sign * 0.04, True, frame * lane_policy.DT)
        self.assertTrue(ready)
        self.assertAlmostEqual(rate, 0.0, delta=0.0001)
        self.assertAlmostEqual(self.policy.motion_bias, -sign * 0.04, delta=0.0001)
        self.assertGreater(lane_policy.approach_scale(sign * 0.08, rate), 0.999)

  def test_motion_estimate_preserves_real_drift_with_irregular_frame_intervals(self):
    for sign in (-1, 1):
      with self.subTest(sign=sign):
        self.policy.reset()
        timestamp = 0.0
        for frame in range(100):
          timestamp += 0.03 if frame % 2 else 0.07
          expected_rate = sign * 0.04
          rate, _ = self.policy.update_motion(expected_rate * timestamp, expected_rate, True, timestamp)
          self.assertAlmostEqual(rate, expected_rate)

  def test_motion_estimate_preserves_sway_while_removing_heading_bias(self):
    rates, expected = [], []
    for frame in range(400):
      timestamp = frame * lane_policy.DT
      omega = 2.0 * np.pi / 5.0
      offset = 0.08 + 0.04 * np.sin(omega * timestamp)
      true_rate = 0.04 * omega * np.cos(omega * timestamp)
      rate, _ = self.policy.update_motion(offset, true_rate - 0.04, True, timestamp)
      if timestamp >= 10.0:
        rates.append(rate)
        expected.append(true_rate)
    self.assertLess(np.sqrt(np.mean((np.asarray(rates) - expected) ** 2)), 0.002)
    self.assertGreater(np.corrcoef(rates, expected)[0, 1], 0.995)

  def test_motion_history_resets_on_gap_duplicate_backward_time_and_lane_jump(self):
    for timestamp, offset in ((6.0, 0.08), (4.95, 0.08), (4.0, 0.08), (5.0, 0.20)):
      with self.subTest(timestamp=timestamp, offset=offset):
        self.policy.reset()
        for frame in range(100):
          self.policy.update_motion(0.08, -0.04, True, frame * lane_policy.DT)
        self.assertLess(self.policy.motion_bias, -0.03)
        rate, ready = self.policy.update_motion(offset, -0.04, True, timestamp)
        self.assertFalse(ready)
        self.assertEqual(rate, -0.04)
        self.assertEqual(self.policy.motion_bias, 0.0)

  def test_motion_bias_is_bounded_and_invalid_samples_discard_history(self):
    for frame in range(300):
      self.policy.update_motion(0.08, 0.5, True, frame * lane_policy.DT)
    self.assertEqual(self.policy.motion_bias, lane_policy.MOTION_MAX_BIAS)
    for values in ((np.nan, 0.0, 15.0), (0.08, np.inf, 15.0), (0.08, 0.0, np.nan)):
      with self.subTest(values=values), self.assertRaises(ValueError):
        self.policy.update_motion(values[0], values[1], True, values[2])
      self.assertIsNone(self.policy.motion_time)
      self.assertEqual(self.policy.motion_bias, 0.0)

  def test_fixed_offset_can_center_despite_heading_that_falsely_predicts_motion(self):
    for sign in (-1, 1):
      with self.subTest(sign=sign):
        self.policy.reset()
        output = make_model_output(lane_center=sign * 0.08, e2e_path_center=sign * 0.08,
                                   lane_heading=-sign * 0.004, e2e_path_heading=-sign * 0.004)
        result = self.apply_for(output, 15.0, lateral_active=True)
        self.assertGreater(sign * result, 0.0)
        self.assertAlmostEqual(self.policy.motion_bias, -sign * 0.08, delta=0.0002)

  def test_frame_discontinuity_clears_learned_centering(self):
    for timestamp, offset in ((20.0, 0.08), (15.0, 0.20)):
      with self.subTest(timestamp=timestamp, offset=offset):
        self.policy.reset()
        output = make_model_output(lane_center=0.08, e2e_path_center=0.08)
        for frame in range(300):
          self.policy.update(output, 0.0, 20.0, lane_policy_enabled=True,
                                 lateral_active=True, frame_time=frame * lane_policy.DT)
        self.assertGreater(self.policy.bias, 0.0)
        moved = make_model_output(lane_center=offset, e2e_path_center=offset)
        self.policy.update(moved, 0.0, 20.0, lane_policy_enabled=True, lateral_active=True, frame_time=timestamp)
        self.assertEqual(self.policy.bias, 0.0)
        self.assertEqual(self.policy.bias_arm_time, 0.0)

  def test_small_correction_has_continuous_symmetric_transition(self):
    threshold = lane_policy.CORRECTION_DEADBAND
    values = np.linspace(-2.0 * threshold, 2.0 * threshold, 501)
    softened = np.array([lane_policy.soften_correction(value) for value in values])
    np.testing.assert_allclose(softened, -softened[::-1], atol=1e-15)
    self.assertTrue(np.all(np.diff(softened) >= -1e-15))
    self.assertTrue(np.all(abs(softened) <= abs(values) + 1e-15))
    np.testing.assert_allclose(softened[abs(values) >= threshold], values[abs(values) >= threshold])
    self.assertEqual(lane_policy.soften_correction(0.5 * threshold), 0.0)
    for boundary in (0.5 * threshold, threshold):
      below = lane_policy.soften_correction(boundary - 1e-10)
      above = lane_policy.soften_correction(boundary + 1e-10)
      self.assertLess(above - below, 3e-10)

  def test_tiny_soft_corrections_do_not_trigger_curve_reversal_hold(self):
    self.arm_lane_policy()
    for offset in (0.008, -0.008):
      output = make_model_output(lane_center=offset)
      self.apply_for(output, 0.5, e2e_curvature=0.001)
      self.assertEqual(self.policy.curve_release, 0.0)
      self.assertGreater(self.policy.correction * offset, 0.0)
      self.assertLess(abs(self.policy.correction), lane_policy.CORRECTION_DEADBAND)

  def test_policy_keeps_input_e2e_path_unchanged(self):
    output = make_model_output(lane_center=0.08, e2e_path_center=0.12)
    before = {key: value.copy() for key, value in output.items()}
    self.apply_for(output, 12.0, lateral_active=True)
    for key, value in before.items():
      np.testing.assert_array_equal(output[key], value)

  def test_bias_qualification_pauses_through_brief_motion_without_integrating(self):
    for sign in (-1, 1):
      with self.subTest(sign=sign):
        self.policy.reset()
        for _ in range(20):
          self.policy.update_bias(sign * 0.10, 0.0, 0.0, True)
        qualified = self.policy.bias_arm_time
        for _ in range(10):
          self.policy.update_bias(sign * 0.10, -sign * 0.06, 0.0, True)
        self.assertEqual(self.policy.bias_arm_time, qualified)
        self.assertEqual(self.policy.bias, 0.0)
        for _ in range(21):
          self.policy.update_bias(sign * 0.10, 0.0, 0.0, True)
        learned = self.policy.bias
        self.assertGreater(sign * learned, 0.0)
        for _ in range(10):
          self.policy.update_bias(sign * 0.10, -sign * 0.06, 0.0, True)
        self.assertEqual(self.policy.bias, learned)

  def test_prolonged_motion_or_opposite_error_discards_bias_qualification(self):
    for reason in ('prolonged_motion', 'opposite_error'):
      with self.subTest(reason=reason):
        self.policy.reset()
        for _ in range(20):
          self.policy.update_bias(0.10, 0.0, 0.0, True)
        if reason == 'prolonged_motion':
          for _ in range(int(np.ceil(lane_policy.BIAS_MAX_PAUSE / lane_policy.DT)) + 1):
            self.policy.update_bias(0.10, 0.06, 0.0, True)
          offset = 0.10
        else:
          offset = -0.10
          self.policy.update_bias(offset, 0.06, 0.0, True)
        self.assertEqual(self.policy.bias_arm_time, 0.0)
        for _ in range(20):
          self.policy.update_bias(offset, 0.0, 0.0, True)
        self.assertEqual(self.policy.bias, 0.0)

  def test_bias_does_not_build_during_fast_convergence_or_intermittent_error(self):
    for _ in range(200):
      self.policy.update_bias(0.10, -0.25, 0.0, True)
    self.assertEqual(self.policy.bias, 0.0)
    for _ in range(10):
      for _ in range(20):
        self.policy.update_bias(0.10, 0.0, 0.0, True)
      self.policy.update_bias(0.0, 0.0, 0.0, True)
    self.assertEqual(self.policy.bias, 0.0)

  def test_bias_holds_at_center_and_unwinds_before_reversing(self):
    output = make_model_output(lane_center=0.08, e2e_path_center=0.08)
    self.apply_for(output, 15.0, lateral_active=True)
    learned = self.policy.bias
    self.assertGreater(learned, 0.0)
    for _ in range(100):
      self.policy.update_bias(0.0, 0.0, 0.0, True)
    self.assertEqual(self.policy.bias, learned)
    self.policy.update_bias(-0.08, 0.0, 0.0, True)
    self.assertGreaterEqual(self.policy.bias, 0.0)
    self.assertLess(self.policy.bias, learned)
    for _ in range(100):
      if self.policy.bias == 0.0:
        break
      self.policy.update_bias(-0.08, 0.0, 0.0, True)
    self.assertEqual(self.policy.bias, 0.0)
    self.assertEqual(self.policy.bias_arm_time, 0.0)
    for _ in range(20):
      self.policy.update_bias(-0.08, 0.0, 0.0, True)
    self.assertEqual(self.policy.bias, 0.0)

  def test_bias_releases_before_predicted_crossing_and_within_build_deadband(self):
    for sign in (-1, 1):
      with self.subTest(sign=sign):
        self.policy.reset()
        output = make_model_output(lane_center=sign * 0.08, e2e_path_center=sign * 0.08)
        self.apply_for(output, 12.0, lateral_active=True)
        learned = abs(self.policy.bias)
        self.policy.update_bias(sign * 0.01, -sign * 0.08, 0.0, True)
        anticipating = abs(self.policy.bias)
        self.assertLess(anticipating, learned)
        self.policy.update_bias(-sign * 0.005, 0.0, 0.0, True)
        self.assertLess(abs(self.policy.bias), anticipating)

  def test_bias_and_total_correction_are_bounded_without_windup(self):
    for sign in (-1, 1):
      with self.subTest(sign=sign):
        self.policy.reset()
        for _ in range(1000):
          bias = self.policy.update_bias(sign * 0.20, 0.0, 0.0, True)
        self.assertAlmostEqual(bias, sign * lane_policy.BIAS_MAX)
        self.policy.reset_bias()
        for _ in range(200):
          bias = self.policy.update_bias(sign * 0.20, 0.0,
                                                sign * lane_policy.MAX_CORRECTION, True)
        self.assertEqual(bias, 0.0)
        output = make_model_output(lane_center=sign * 0.20, e2e_path_center=-sign * 0.80)
        result = self.apply_for(output, 10.0, lateral_active=True)
        self.assertLessEqual(abs(result), lane_policy.MAX_CORRECTION)
        self.assertEqual(self.policy.bias, 0.0)

  def test_bias_clears_on_override_disengagement_and_unreliable_geometry(self):
    cases = [{'lateral_active': False}, {'steering_pressed': True},
             {'left_prob': 0.80}, {'right_prob': 0.10}, {'v_ego': 5.0},
             {'e2e_curvature': 0.005}, {'lane_center': 0.50}]
    for case in cases:
      with self.subTest(case=case):
        self.policy.reset()
        output = make_model_output(lane_center=0.08, e2e_path_center=0.08)
        self.apply_for(output, 12.0, lateral_active=True)
        self.assertGreater(self.policy.bias, 0.0)
        geometry = {k: v for k, v in case.items() if k in ('left_prob', 'right_prob', 'lane_center')}
        options = {'e2e_curvature': 0.0, 'v_ego': 20.0, 'lateral_active': True, 'lane_policy_enabled': True}
        options.update({k: v for k, v in case.items() if k not in geometry})
        self.policy.update(make_model_output(**geometry), **options)
        self.assertEqual(self.policy.bias, 0.0)
        self.assertEqual(self.policy.bias_arm_time, 0.0)
        self.assertEqual(self.policy.bias_pause_time, 0.0)
        self.assertIsNone(self.policy.motion_time)
        self.assertEqual(self.policy.motion_bias, 0.0)

  def test_fallback_is_exact_e2e_and_discards_learned_bias(self):
    for reason in ('disabled', 'blinker', 'lane_change', 'low_confidence', 'bad_near_geometry'):
      with self.subTest(reason=reason):
        self.policy.reset()
        output = make_model_output(lane_center=0.08, e2e_path_center=0.08)
        self.apply_for(output, 12.0, lateral_active=True)
        self.assertGreater(self.policy.bias, 0.0)
        if reason == 'lane_change':
          output['desire_state'][0, 4] = 0.2
        elif reason == 'low_confidence':
          output['lane_lines_prob'][:] = 0.1
        elif reason == 'bad_near_geometry':
          output['lane_lines'][0, 1, 0, 0] = np.nan
        result = self.policy.update(output, -0.001, 20.0, lane_policy_enabled=reason != 'disabled',
                                        blinkers_active=reason == 'blinker', lateral_active=True)
        self.assertEqual(result, -0.001)
        self.assertEqual(self.policy.bias, 0.0)
        self.assertFalse(self.policy.active)
        self.assertIsNone(self.policy.motion_time)

  def test_turn_guard_clears_bias_instead_of_bypassing_release(self):
    output = make_model_output(lane_center=0.08, e2e_path_center=0.08)
    self.apply_for(output, 12.0, e2e_curvature=-0.0003, lateral_active=True)
    self.assertGreater(self.policy.bias, 0.0)
    self.policy.update(output, 0.0003, 20.0, lane_policy_enabled=True, lateral_active=True)
    self.assertGreater(self.policy.turn_release, 0.0)
    self.assertEqual(self.policy.bias, 0.0)
    self.assertIsNone(self.policy.motion_time)

  def test_short_plan_keeps_original_fit_without_extrapolation(self):
    output = make_model_output(lane_center=0.20)
    x = np.asarray(ModelConstants.X_IDXS)
    output['plan'] = output['plan'][:, x <= 30.0, :]
    self.arm_lane_policy(output)
    result = self.apply_for(output, 1.0)
    self.assertGreater(result, 0.0)
    self.assertTrue(self.policy.active)
    output['plan'] = output['plan'][:, :4, :]
    self.assertEqual(self.policy.update(output, 0.001, 20.0, lane_policy_enabled=True), 0.001)

  def test_invalid_e2e_path_returns_exact_e2e(self):
    self.arm_lane_policy()
    output = make_model_output(lane_center=0.35)
    output['plan'][:] = np.nan
    e2e = -0.0012
    self.assertEqual(self.policy.update(output, e2e, 20.0, lane_policy_enabled=True), e2e)
    self.assertFalse(self.policy.active)

  def test_hysteresis_retains_full_center_above_exit_threshold(self):
    self.arm_lane_policy()
    reduced_confidence = make_model_output(left_prob=0.80, right_prob=0.80, lane_center=0.25)
    curvature = self.policy.update(reduced_confidence, 0.001, 20.0, lane_policy_enabled=True)
    self.assertTrue(self.policy.active)
    self.assertFalse(self.policy.holding_line)
    self.assertGreater(curvature, 0.001)

  def test_below_exit_confidence_releases_to_exact_e2e(self):
    self.arm_lane_policy()
    low_confidence = make_model_output(left_prob=0.65, right_prob=0.65)
    self.assertEqual(self.policy.update(low_confidence, -0.0012, 20.0, lane_policy_enabled=True), -0.0012)
    self.assertFalse(self.policy.active)

  def test_one_line_hold_uses_learned_width(self):
    self.arm_lane_policy()
    one_line = make_model_output(left_prob=0.99, right_prob=0.10, lane_center=0.35)
    curvature = self.apply_for(one_line, 0.50)
    self.assertTrue(self.policy.active)
    self.assertTrue(self.policy.holding_line)
    self.assertGreater(curvature, 0.0)

  def test_one_line_hold_expires_to_exact_e2e(self):
    self.arm_lane_policy()
    one_line = make_model_output(left_prob=0.99, right_prob=0.10, lane_center=0.35)
    e2e = -0.0012
    result = self.apply_for(one_line, lane_policy.ONE_LINE_HOLD_TIME + 2.0 * lane_policy.DT, e2e)
    self.assertEqual(result, e2e)
    self.assertFalse(self.policy.active)

  def test_blinker_releases_lane_lock(self):
    output = self.arm_lane_policy()
    self.assertEqual(self.policy.update(output, 0.0123, 20.0, blinkers_active=True, lane_policy_enabled=True), 0.0123)
    self.assertFalse(self.policy.active)

  def test_lane_change_intent_releases_lane_lock(self):
    self.arm_lane_policy()
    output = make_model_output()
    output['desire_state'][0, 3] = 0.2
    self.assertEqual(self.policy.update(output, 0.0123, 20.0, lane_policy_enabled=True), 0.0123)
    self.assertFalse(self.policy.active)

  def test_robust_geometry_accepts_normal_taper_and_rejects_extreme_taper(self):
    self.arm_lane_policy(make_model_output(lane_width=3.6, lane_width_end=4.1))
    self.assertTrue(self.policy.active)

    self.policy.reset()
    output = make_model_output(lane_width=3.6, lane_width_end=5.0)
    self.apply_for(output, lane_policy.ARM_TIME + lane_policy.DT)
    self.assertFalse(self.policy.active)


if __name__ == "__main__":
  unittest.main()
