import unittest
from unittest.mock import patch

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

  def test_no_plan_gate_for_clean_lanes(self):
    output = make_model_output(lane_center=0.35)
    output['plan'][:] = np.nan
    self.arm_lane_policy(output)
    curvature = modeld.apply_lane_lock(output, 0.0, 20.0, lane_policy_enabled=True)
    self.assertGreater(curvature, 0.0)

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

  def test_approach_preserves_parallel_diverging_and_large_offsets(self):
    for sign in (-1.0, 1.0):
      for offset, heading in ((0.2, 0.0), (0.2, 0.01), (0.5, -0.01)):
        self.assertEqual(modeld.get_lane_lock_approach_offset(sign * offset, sign * heading, 28.0, 1.0, 0.99),
                         sign * offset)

  def test_approach_is_symmetric_bounded_and_immediate(self):
    for sign in (-1.0, 1.0):
      damped = modeld.get_lane_lock_approach_offset(sign * 0.2, -sign * 0.01, 28.0, 1.0, 0.99)
      self.assertLess(abs(damped), 0.2)
      self.assertGreaterEqual(sign * damped, 0.1)
      # No stored damping remains when convergence stops.
      self.assertEqual(modeld.get_lane_lock_approach_offset(sign * 0.2, 0.0, 28.0, 1.0, 0.99), sign * 0.2)

  def test_approach_fades_with_confidence_speed_and_curve_authority(self):
    approach = modeld.get_lane_lock_approach_offset
    full = approach(0.2, -0.005, 20.0, 1.0, 0.99)
    self.assertEqual(approach(0.2, -0.005, 20.0, 1.0, 0.70), 0.2)
    self.assertEqual(approach(0.2, -0.005, 8.0, 1.0, 0.99), 0.2)
    self.assertGreater(approach(0.2, -0.005, 20.0, 0.5, 0.99), full)
    self.assertGreater(approach(0.2, -0.005, 20.0, 1.0, 0.80), full)

  def test_approach_eases_converging_correction_without_weakening_steady_centering(self):
    self.arm_lane_policy()
    output = make_model_output(lane_center=0.2, lane_heading=-0.004)
    result = self.apply_for(output, 0.5)
    original = 2.0 * (0.2 + modeld.LANE_LOCK_HEADING_GAIN * -0.004 * 40.0) / 40.0 ** 2
    self.assertGreater(result, 0.0)
    self.assertLess(result, original)
    steady = self.apply_for(make_model_output(lane_center=0.2), 0.5)
    self.assertAlmostEqual(steady, 2.0 * 0.2 / 40.0 ** 2)

  def test_one_line_hold_keeps_original_offset_and_heading_target(self):
    self.arm_lane_policy()
    output = make_model_output(left_prob=0.99, right_prob=0.1, lane_center=0.2, lane_heading=-0.004)
    result = self.apply_for(output, 0.5)
    original = 2.0 * (0.2 + modeld.LANE_LOCK_HEADING_GAIN * -0.004 * 40.0) / 40.0 ** 2
    self.assertTrue(modeld._lane_lock_one_line_hold)
    self.assertAlmostEqual(result, original)

  def test_delayed_straight_lane_response_reduces_overshoot(self):
    # A deliberately simple closed-loop regression, not a vehicle validation:
    # lane offset/heading kinematics plus steering delay and first-order lag.
    # Disable only the new helper to recover the lp-final baseline behavior.
    def simulate(speed, delay, lag):
      modeld.reset_lane_lock()
      self.arm_lane_policy()
      offset, heading, actual = 0.25, 0.0, 0.0
      pending = [0.0] * round(delay / modeld.DT_MDL)
      offsets = []
      for _ in range(600):
        output = make_model_output(lane_center=offset, lane_heading=heading)
        command = modeld.apply_lane_lock(output, 0.0, speed, lane_policy_enabled=True)
        pending.append(command)
        delayed = pending.pop(0)
        actual += modeld.DT_MDL / (lag + modeld.DT_MDL) * (delayed - actual)
        heading -= speed * actual * modeld.DT_MDL
        offset += speed * heading * modeld.DT_MDL
        offsets.append(offset)
      return max(0.0, -min(offsets)), float(np.sqrt(np.mean(np.square(offsets[200:]))))

    for speed in (20.0, 28.0):
      for delay, lag in ((0.2, 0.25), (0.3, 0.35)):
        with self.subTest(speed=speed, delay=delay, lag=lag):
          with patch.object(modeld, 'get_lane_lock_approach_offset', side_effect=lambda offset, *args: offset):
            baseline = simulate(speed, delay, lag)
          candidate = simulate(speed, delay, lag)
          self.assertLess(candidate[0], baseline[0])
          self.assertLess(candidate[1], baseline[1])

  def test_action_passes_adaptive_time_to_policy(self):
    output = make_model_output(lane_center=0.2, lane_heading=-0.004)
    output['action'] = np.zeros((1, 2))
    with patch.object(modeld, 'apply_lane_lock', wraps=modeld.apply_lane_lock) as apply:
      modeld.get_action_from_model(output, log.ModelDataV2.Action(), 0.375, 0.3, 20.0,
                                   lane_policy_enabled=True, approach_time=0.23)
    self.assertEqual(apply.call_args.args[-1], 0.23)

  def test_adaptive_time_keeps_exact_fallbacks_and_offset_cap(self):
    for anticipation in (0.17, 0.20, 0.25):
      self.arm_lane_policy()
      output = make_model_output(lane_center=0.2, lane_heading=-0.02)
      damped = modeld.get_lane_lock_approach_offset(0.2, -0.02, 28.0, 1.0, 0.99, anticipation)
      self.assertGreaterEqual(damped, 0.2 * (1.0 - modeld.LANE_LOCK_APPROACH_MAX_FRACTION))
      self.assertEqual(modeld.apply_lane_lock(output, 0.00123, 28.0, True, True, anticipation), 0.00123)
      self.assertEqual(modeld.apply_lane_lock(output, 0.00123, 28.0, False, False, anticipation), 0.00123)


class TestLanePolicyApproachTiming(unittest.TestCase):
  def settle(self, delay):
    timing = modeld.LanePolicyApproachTiming()
    for _ in range(1200):
      timing.update(delay, True, True, 0.1)
    return timing

  def test_reference_retains_working_tune(self):
    self.assertEqual(self.settle(0.30).approach_time, 0.20)
    self.assertAlmostEqual(self.settle(0.302).approach_time, 0.2005)

  def test_slower_response_adds_bounded_anticipation(self):
    for delay, expected in ((0.15, 0.17), (0.20, 0.175), (0.40, 0.225), (0.50, 0.25), (0.65, 0.25)):
      with self.subTest(delay=delay):
        self.assertAlmostEqual(self.settle(delay).approach_time, expected)

  def test_missing_invalid_and_stale_estimates_use_baseline(self):
    for delay, estimated, valid, age in ((0.4, False, True, 0.1), (0.4, True, False, 0.1),
                                       (0.4, True, True, 1.01), (0.4, True, True, -0.1),
                                       (0.4, True, True, float('nan')), (float('nan'), True, True, 0.1),
                                       (float('inf'), True, True, 0.1), (0.0, True, True, 0.1),
                                       (0.66, True, True, 0.1)):
      timing = modeld.LanePolicyApproachTiming()
      self.assertEqual(timing.update(delay, estimated, valid, age), 0.20)
      self.assertFalse(timing.using_estimate)

  def test_loss_and_recovery_never_step_the_tune(self):
    timing = self.settle(0.65)
    for valid in (False, True, False):
      for _ in range(1200):
        previous = timing.approach_time
        actual = timing.update(0.15, True, valid, 0.1)
        self.assertLessEqual(abs(actual - previous), modeld.LANE_LOCK_TIMING_MAX_RATE * modeld.DT_MDL + 1e-12)
        self.assertGreaterEqual(actual, modeld.LANE_LOCK_APPROACH_MIN_TIME)
        self.assertLessEqual(actual, modeld.LANE_LOCK_APPROACH_MAX_TIME)
      self.assertAlmostEqual(actual, 0.17 if valid else 0.20)

  def test_lane_reset_does_not_reset_delay_schedule(self):
    timing = self.settle(0.40)
    before = timing.approach_time
    modeld.reset_lane_lock()
    self.assertEqual(timing.approach_time, before)

  def test_jittering_delay_remains_bounded(self):
    timing = modeld.LanePolicyApproachTiming()
    for i in range(400):
      timing.update(0.15 if i % 2 else 0.65, True, True, 0.0)
      self.assertLess(abs(timing.approach_time - 0.20), 0.01)


if __name__ == "__main__":
  unittest.main()
