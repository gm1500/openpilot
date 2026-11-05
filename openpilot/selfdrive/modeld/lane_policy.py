"""plan-long centering with batmobile's lane qualification and fallback.

The correction follows commaai/openpilot commit 1c2df3b: two-second lookahead,
half gain, confidence/speed weighting and a 0.5 s filter. The raw model path
and steering action stay intact; only the qualified correction is added.
"""
import math
import time

import numpy as np

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.modeld.constants import ModelConstants, Plan

DT = 1.0 / ModelConstants.MODEL_RUN_FREQ
ARM_LINE_PROB = 0.92
RETAIN_LINE_PROB = 0.70
MIN_LANE_WIDTH = 2.6                         # m
MAX_LANE_WIDTH = 5.2                         # m
MIN_WIDTH_EDGE = 0.10                       # m inside accepted width range
MAX_WIDTH_CHANGE = 0.75                     # m across the fitted horizon
ARM_TIME = 0.75                             # s of strong two-line confidence
WIDTH_TIME = 9.95                           # s, learned width for one-line hold
ONE_LINE_HOLD_TIME = 1.00                    # s
WIDTH_FIT_START, WIDTH_FIT_END = 8.0, 55.0    # m
MAX_LANE_CHANGE_PROB = 0.10
LOG_INTERVAL = 1.0                         # s
LOOKAHEAD_TIME = 2.0                        # s
MIN_LOOKAHEAD = 10.0                        # m
CENTER_GAIN = 0.5
MAX_CORRECTION_ACCEL = 0.5                  # m/s^2, added lateral acceleration
CORRECTION_FILTER_TIME = 0.5                # s


def is_enabled(value: bytes | bool | None) -> bool:
  if value is None:
    return True
  return value == b"1" if isinstance(value, (bytes, bytearray)) else bool(value)


def inner_probs(model_output: dict[str, np.ndarray]) -> tuple[float, float]:
  probs = np.asarray(model_output['lane_lines_prob'])
  if probs.shape != (1, 8):
    raise ValueError(f"expected lane_lines_prob shape (1, 8), got {probs.shape}")
  return float(probs[0, 3]), float(probs[0, 5])


def lane_width(left: np.ndarray, right: np.ndarray, fit: np.ndarray) -> tuple[float, bool]:
  low, median, high = np.percentile((right - left)[fit], (10.0, 50.0, 90.0))
  edge = min(low - MIN_LANE_WIDTH, MAX_LANE_WIDTH - high)
  valid = MIN_LANE_WIDTH <= median <= MAX_LANE_WIDTH and edge >= MIN_WIDTH_EDGE and high - low <= MAX_WIDTH_CHANGE
  return float(median), bool(valid)


def plan_lane_correction(output: dict[str, np.ndarray], center: np.ndarray, x: np.ndarray,
                         speed: float, confidence: float) -> tuple[float, float, float]:
  """Apply upstream's curvature rule to the centre accepted by our detector."""
  position = np.asarray(output['plan'][0, :, Plan.POSITION], dtype=np.float64)
  if position.ndim != 2 or position.shape[1] < 2:
    raise ValueError("E2E position path has an invalid shape")
  plan_x, plan_y = position[:, 0], position[:, 1]
  if (len(plan_x) < 4 or not np.all(np.isfinite(plan_x)) or not np.all(np.isfinite(plan_y)) or
      np.any(np.diff(plan_x) <= 0.0)):
    raise ValueError("E2E position path is not a finite increasing path")
  if x.shape != center.shape:
    raise ValueError("lane centre has an invalid shape")
  lookahead = max(LOOKAHEAD_TIME * speed, MIN_LOOKAHEAD)
  # Keep batmobile's no-extrapolation rule at the new, longer lookahead.
  if not (plan_x[0] <= lookahead <= plan_x[-1] and x[0] <= lookahead <= x[-1]):
    raise ValueError("lane/model path does not cover the lookahead")
  upper = int(np.searchsorted(x, lookahead))
  if not np.all(np.isfinite(center[max(0, upper - 1):upper + 1])):
    raise ValueError("lane centre is not finite at the lookahead")
  error = float(np.interp(lookahead, x, center) - np.interp(lookahead, plan_x, plan_y))
  weight = float(np.clip(confidence, 0.0, 1.0)) * np.interp(speed, [4.0, 8.0], [0.0, 1.0])
  max_curvature = MAX_CORRECTION_ACCEL / max(speed, 1.0) ** 2
  correction = float(np.clip(CENTER_GAIN * weight * 2.0 * error / lookahead ** 2, -max_curvature, max_curvature))
  return correction, error, lookahead


class LanePolicy:
  def __init__(self, logger=None):
    self.logger = logger
    self.last_mode = None
    self.last_mode_time = self.last_anchor_time = 0.0
    self.error_logged = False
    self.correction_filter = FirstOrderFilter(0.0, CORRECTION_FILTER_TIME, DT)
    self.reset()

  @property
  def blending(self) -> bool:
    """Legacy HUD status: confidence arming or temporary one-line hold."""
    return self.holding_line or (not self.active and self.arm_time > 0.0)

  def reset(self) -> None:
    self.active = self.holding_line = False
    self.arm_time = self.line_loss_time = self.correction = 0.0
    self.width: float | None = None
    self.lane_confidence = 0.0
    self.correction_filter.x = 0.0

  def _mode(self, mode: str) -> None:
    now = time.monotonic()
    if self.logger is not None and mode != self.last_mode and now - self.last_mode_time >= LOG_INTERVAL:
      self.logger.info(f"lp-plan-long: {mode}")
      self.last_mode, self.last_mode_time = mode, now

  def _fallback(self, e2e: float, reason: str) -> float:
    self.reset()
    self._mode(reason)
    return float(e2e)

  def _record(self, lookahead: float, error: float, e2e: float) -> None:
    now = time.monotonic()
    if self.logger is not None and now - self.last_anchor_time >= LOG_INTERVAL:
      self.logger.info(" ".join((
        f"lp-plan-long: x={lookahead:.1f}m error={error:+.3f}m corr={self.correction:+.6f} e2e={e2e:+.6f}",
        f"confidence={self.lane_confidence:.3f} one_line={int(self.holding_line)}",
      )))
      self.last_anchor_time = now

  def _centerline(self, output: dict[str, np.ndarray], x: np.ndarray,
                  lookahead: float = WIDTH_FIT_END) -> tuple[np.ndarray | None, bool]:
    left, right = output['lane_lines'][0, 1:3, :, 0].astype(np.float64)
    left_prob, right_prob = inner_probs(output)
    fit = (x >= WIDTH_FIT_START) & (x <= WIDTH_FIT_END)
    if lookahead > WIDTH_FIT_END:
      # Apply the same detector checks through the samples used by the longer lookahead.
      upper = min(int(np.searchsorted(x, lookahead)), len(x) - 1)
      fit = (x >= WIDTH_FIT_START) & (x <= x[upper])
    if x.shape != left.shape or np.count_nonzero(fit) < 3:
      raise ValueError("lane-line horizon does not match ModelConstants.X_IDXS")
    valid_left = np.isfinite(left_prob) and left_prob >= RETAIN_LINE_PROB and np.all(np.isfinite(left[fit]))
    valid_right = np.isfinite(right_prob) and right_prob >= RETAIN_LINE_PROB and np.all(np.isfinite(right[fit]))
    if valid_left and valid_right:
      width, valid_width = lane_width(left, right, fit)
      if valid_width:
        strong = min(left_prob, right_prob) >= ARM_LINE_PROB
        self.line_loss_time, self.holding_line = 0.0, False
        if strong:
          self.width = width if self.width is None else self.width + min(DT / WIDTH_TIME, 1.0) * (width - self.width)
        self.lane_confidence = min(left_prob, right_prob)
        return 0.5 * (left + right), strong
    # Preserve the existing left-line priority when two visible lines disagree.
    if self.active and self.width is not None and (valid_left or valid_right):
      self.line_loss_time += DT
      if self.line_loss_time <= ONE_LINE_HOLD_TIME:
        self.holding_line = True
        # Weight the reconstructed centre by the line actually used, not the missing line.
        self.lane_confidence = left_prob if valid_left else right_prob
        return (left + self.width / 2.0 if valid_left else right - self.width / 2.0), False
    return None, False

  def update(self, model_output: dict[str, np.ndarray], e2e_curvature: float, v_ego: float,
             blinkers_active: bool = False, lane_policy_enabled: bool = False,
             lateral_active: bool = False, steering_pressed: bool = False, frame_time: float | None = None) -> float:
    if not lane_policy_enabled:
      return self._fallback(e2e_curvature, "stock-e2e (lane toggle off)")
    if blinkers_active:
      return self._fallback(e2e_curvature, "stock-e2e fallback: blinker")
    try:
      if not all(math.isfinite(value) for value in (v_ego, e2e_curvature)):
        raise ValueError("non-finite speed or E2E curvature")
      # Desire slots 3/4 are lane-change left/right in the model output layout.
      desire = model_output['desire_state'][0]
      lane_change_prob = float(desire[3] + desire[4])
      if not math.isfinite(lane_change_prob):
        raise ValueError("non-finite lane-change probability")
      if lane_change_prob > MAX_LANE_CHANGE_PROB:
        return self._fallback(e2e_curvature, "stock-e2e fallback: lane-change intent")
      x = np.asarray(ModelConstants.X_IDXS, dtype=np.float64)
      center, strong = self._centerline(model_output, x, max(LOOKAHEAD_TIME * v_ego, MIN_LOOKAHEAD))
      if center is None:
        return self._fallback(e2e_curvature, "stock-e2e fallback: lane geometry or confidence")
      if not self.active:
        self.arm_time = min(self.arm_time + DT, ARM_TIME) if strong else 0.0
        if self.arm_time < ARM_TIME:
          self._mode("stock-e2e fallback: arming lane confidence")
          return float(e2e_curvature)
        self.active, self.line_loss_time = True, 0.0

      correction, error, lookahead = plan_lane_correction(model_output, center, x, v_ego, self.lane_confidence)
      max_curvature = MAX_CORRECTION_ACCEL / max(v_ego, 1.0) ** 2
      # Keep the limit valid after a speed rise, including the filter's retained state.
      self.correction = float(np.clip(self.correction_filter.update(correction), -max_curvature, max_curvature))
      self.correction_filter.x = self.correction
      self._record(lookahead, error, e2e_curvature)
      self.error_logged = False
      self._mode("plan-long one-line hold" if self.holding_line else "plan-long lane centering")
      return float(e2e_curvature + self.correction)
    except (KeyError, IndexError, TypeError, ValueError, FloatingPointError, np.linalg.LinAlgError) as err:
      if not self.error_logged and self.logger is not None:
        self.logger.warning(f"lp-plan-long input error: {type(err).__name__}: {err}")
      self.error_logged = True
      return self._fallback(e2e_curvature, "stock-e2e fallback: lane-policy input error")
