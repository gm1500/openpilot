"""E2E lane anchoring, independent of model inference and messaging.

Only the scalar curvature command is adjusted; the E2E position path is never
rewritten. Positive lane offset means the lane centre is right of the vehicle.
"""
import math
import time

import numpy as np

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
ANCHOR_BLEND_START, ANCHOR_BLEND_END = 10.0, 45.0  # m, scalar correction gain
ANCHOR_FIT_START, ANCHOR_FIT_END = 8.0, 40.0  # m, compare nearby lane/E2E geometry
ANCHOR_FALLBACK_END = 30.0                   # m, short-plan fit without extrapolation
MIN_LOOKAHEAD, MAX_LOOKAHEAD = 32.0, 45.0     # m
HEADING_GAIN = 0.55
NEAR_FIT_END = 20.0                         # m, lane-relative offset and tangent
MOTION_BIAS_TIME = 2.0                       # s, reconcile heading with offset changes
MOTION_MAX_BIAS = 0.10                      # m/s
MOTION_MAX_GAP = 0.15                       # s, reject stale frame history
MOTION_MAX_STEP = 0.10                      # m/frame, reject lane estimate jumps
APPROACH_TIME = 0.20                        # s
APPROACH_MAX_FRACTION = 0.25
BIAS_ARM_TIME = 2.0                         # s of steady, same-side error
BIAS_MAX_PAUSE = 2.0                        # s, discard qualification after long motion
BIAS_DEADBAND = 0.015                       # m
BIAS_MAX_OFFSET = 0.35                      # m, leave large transients to base anchoring
BIAS_MAX_RATE = 0.05                        # m/s, integrate only during steady motion
BIAS_KI = 0.00008                           # 1/(m^2 s)
BIAS_MAX = 0.00015                          # 1/m, inside the shared correction limit
BIAS_BUILD_RATE = 0.00001                   # 1/(m s)
BIAS_RELEASE_PREVIEW = 0.35                 # s
BIAS_RELEASE_GAIN = 0.003                   # 1/(m^2 s)
BIAS_RELEASE_RATE = 0.00008                 # 1/(m s)
CURVE_THRESHOLD = 0.00060                   # 1/m, guard lane-target reversals in curves
CURVE_REVERSAL_TIME = 0.30                  # s
TURN_THRESHOLD = 0.00015                    # 1/m, detect meaningful E2E direction changes
TURN_RELEASE_TIME = 0.35                    # s
MAX_CORRECTION = 0.00045                    # 1/m
ENGAGE_STEP, RELEASE_STEP = 0.00008, 0.00020 # 1/m per model frame
CORRECTION_DEADBAND = 0.000012               # 1/m
MAX_LANE_CHANGE_PROB = 0.10
LOG_INTERVAL = 1.0                         # s


def is_enabled(value: bytes | bool | None) -> bool:
  if value is None:
    return True
  return value == b"1" if isinstance(value, (bytes, bytearray)) else bool(value)


def sign(value: float) -> int:
  return 1 if value > 0.0 else -1 if value < 0.0 else 0


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


def anchor_gain(x: np.ndarray | float) -> np.ndarray | float:
  progress = np.clip((np.asarray(x) - ANCHOR_BLEND_START) / (ANCHOR_BLEND_END - ANCHOR_BLEND_START), 0.0, 1.0)
  return progress ** 3 * (10.0 + progress * (-15.0 + 6.0 * progress))


def anchor_correction(output: dict[str, np.ndarray], center: np.ndarray, x: np.ndarray, speed: float) -> tuple[float, float, float]:
  position = np.asarray(output['plan'][0, :, Plan.POSITION], dtype=np.float64)
  if position.ndim != 2 or position.shape[1] < 2:
    raise ValueError("E2E position path has an invalid shape")
  plan_x, plan_y = position[:, 0], position[:, 1]
  valid = np.isfinite(plan_x) & np.isfinite(plan_y)
  plan_x, plan_y = plan_x[valid], plan_y[valid]
  if plan_x.shape[0] < 4 or np.any(np.diff(plan_x) <= 0.0):
    raise ValueError("E2E position path is not a finite increasing path")
  fit = (x >= ANCHOR_FIT_START) & (x <= ANCHOR_FALLBACK_END)
  if np.count_nonzero(fit) < 3:
    raise ValueError("lane horizon has too few anchor samples")
  if plan_x[0] > x[fit][0] or plan_x[-1] < x[fit][-1]:
    raise ValueError("E2E position path does not cover the anchor horizon")
  extended = (x >= ANCHOR_FIT_START) & (x <= ANCHOR_FIT_END)
  if plan_x[-1] >= x[extended][-1]:
    fit = extended
  relative = center[fit] - np.interp(x[fit], plan_x, plan_y)
  if not np.all(np.isfinite(relative)):
    raise ValueError("lane/E2E anchor path is not finite")
  # Align affine disagreement, retaining E2E's road curvature.
  heading, offset = np.polyfit(x[fit], relative, 1)
  lookahead = float(np.clip(2.0 * speed, MIN_LOOKAHEAD, MAX_LOOKAHEAD))
  error = float(anchor_gain(lookahead) * (HEADING_GAIN * heading * lookahead + offset))
  correction = float(np.clip(2.0 * error / (lookahead * lookahead), -MAX_CORRECTION, MAX_CORRECTION))
  return correction, error, lookahead


def near_geometry(center: np.ndarray, x: np.ndarray) -> tuple[float, float]:
  fit = (x >= 0.0) & (x <= NEAR_FIT_END)
  if np.count_nonzero(fit) < 3 or not np.all(np.isfinite(center[fit])):
    raise ValueError("invalid near-lane geometry")
  _, heading, offset = np.polyfit(x[fit], center[fit], 2)
  return float(offset), float(heading)


def soften_correction(correction: float) -> float:
  if abs(correction) >= CORRECTION_DEADBAND:
    return correction
  ramp = float(np.clip((abs(correction) / CORRECTION_DEADBAND - 0.5) * 2.0, 0.0, 1.0))
  return correction * ramp * ramp * (3.0 - 2.0 * ramp)


def approach_scale(offset: float, rate: float) -> float:
  if offset * rate >= 0.0:
    return 1.0
  near_weight = float(np.clip((0.30 - abs(offset)) / 0.20, 0.0, 1.0))
  fraction = min(APPROACH_TIME * abs(rate) / max(abs(offset), 1e-6), APPROACH_MAX_FRACTION)
  return 1.0 - near_weight * fraction


class LanePolicy:
  def __init__(self, logger=None):
    self.logger = logger
    self.last_mode = None
    self.last_mode_time = self.last_anchor_time = 0.0
    self.error_logged = False
    self.reset()

  @property
  def blending(self) -> bool:
    """Legacy HUD status: confidence arming or temporary one-line hold."""
    return self.holding_line or (not self.active and self.arm_time > 0.0)

  def reset(self) -> None:
    self.active = self.holding_line = False
    self.arm_time = self.line_loss_time = self.correction = 0.0
    self.width: float | None = None
    self.last_turn_sign = self.last_curve_sign = 0
    self.turn_release = self.curve_release = 0.0
    self.reset_motion()
    self.reset_bias()

  def reset_motion(self) -> None:
    self.motion_time: float | None = None
    self.motion_offset = self.motion_bias = 0.0

  def reset_bias_arm(self) -> None:
    self.bias_arm_time = self.bias_pause_time = 0.0
    self.bias_error_sign = 0

  def reset_bias(self) -> None:
    self.bias = 0.0
    self.reset_bias_arm()

  def _mode(self, mode: str) -> None:
    now = time.monotonic()
    if self.logger is not None and mode != self.last_mode and now - self.last_mode_time >= LOG_INTERVAL:
      self.logger.info(f"ui-lp-full-center: {mode}")
      self.last_mode, self.last_mode_time = mode, now

  def _fallback(self, e2e: float, reason: str) -> float:
    self.reset()
    self._mode(reason)
    return float(e2e)

  def _record(self, lookahead: float, error: float, e2e: float, offset: float, rate: float, geometric_rate: float, ready: bool) -> None:
    now = time.monotonic()
    if self.logger is not None and now - self.last_anchor_time >= LOG_INTERVAL:
      self.logger.info(" ".join((
        f"lp-anchor-v3: x={lookahead:.1f}m error={error:+.3f}m corr={self.correction:+.6f} e2e={e2e:+.6f}",
        f"offset={offset:+.3f}m rate={rate:+.3f}m/s bias={self.bias:+.6f}",
        f"geom_rate={geometric_rate:+.3f}m/s rate_bias={self.motion_bias:+.3f}m/s motion_ready={int(ready)}",
        f"bias_arm={self.bias_arm_time:.2f}s bias_pause={self.bias_pause_time:.2f}s",
      )))
      self.last_anchor_time = now

  def _centerline(self, output: dict[str, np.ndarray], x: np.ndarray) -> tuple[np.ndarray | None, bool]:
    left, right = output['lane_lines'][0, 1:3, :, 0].astype(np.float64)
    left_prob, right_prob = inner_probs(output)
    fit = (x >= WIDTH_FIT_START) & (x <= WIDTH_FIT_END)
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
        return 0.5 * (left + right), strong
    # Preserve the existing left-line priority when two visible lines disagree.
    if self.active and self.width is not None and (valid_left or valid_right):
      self.line_loss_time += DT
      if self.line_loss_time <= ONE_LINE_HOLD_TIME:
        self.holding_line = True
        return (left + self.width / 2.0 if valid_left else right - self.width / 2.0), False
    return None, False

  def update_motion(self, offset: float, geometric_rate: float, allowed: bool,
                    frame_time: float | None = None) -> tuple[float, bool]:
    """Remove persistent heading-rate error without filtering the steering command."""
    if not allowed:
      self.reset_motion()
      return geometric_rate, False
    if frame_time is None:
      frame_time = 0.0 if self.motion_time is None else self.motion_time + DT
    if not all(math.isfinite(value) for value in (offset, geometric_rate, frame_time)):
      self.reset_motion()
      raise ValueError("invalid lane-motion sample")
    ready = False
    if self.motion_time is not None:
      dt, step = frame_time - self.motion_time, offset - self.motion_offset
      ready = 0.0 < dt <= MOTION_MAX_GAP and abs(step) < MOTION_MAX_STEP
      if ready:
        innovation = dt * (geometric_rate - self.motion_bias) - step
        self.motion_bias = float(np.clip(self.motion_bias + innovation / (MOTION_BIAS_TIME + dt), -MOTION_MAX_BIAS, MOTION_MAX_BIAS))
      else:
        self.motion_bias = 0.0
    self.motion_time, self.motion_offset = frame_time, offset
    return geometric_rate - self.motion_bias, ready

  def update_bias(self, offset: float, rate: float, base: float, allowed: bool) -> float:
    """Learn steady centering error; hold at centre and unwind before crossing."""
    if not allowed or abs(offset) > BIAS_MAX_OFFSET:
      self.reset_bias()
      return 0.0
    error = math.copysign(max(abs(offset) - BIAS_DEADBAND, 0.0), offset)
    bias_sign = sign(self.bias)
    projected = offset + BIAS_RELEASE_PREVIEW * rate
    release = bias_sign * min(bias_sign * offset, bias_sign * projected, 0.0)
    if release * self.bias < 0.0:
      change = float(np.clip(BIAS_RELEASE_GAIN * release, -BIAS_RELEASE_RATE, BIAS_RELEASE_RATE)) * DT
      self.bias = math.copysign(max(abs(self.bias) - abs(change), 0.0), self.bias)
      self.reset_bias_arm()
    elif error == 0.0:
      self.reset_bias_arm()
    else:
      if sign(error) != self.bias_error_sign:
        self.reset_bias_arm()
      self.bias_error_sign = sign(error)
      steady = abs(rate) <= BIAS_MAX_RATE
      if steady:
        self.bias_pause_time = 0.0
        self.bias_arm_time = min(self.bias_arm_time + DT, BIAS_ARM_TIME)
      else:
        self.bias_pause_time = min(self.bias_pause_time + DT, BIAS_MAX_PAUSE)
        if self.bias_pause_time >= BIAS_MAX_PAUSE:
          self.bias_arm_time = 0.0
      if steady and self.bias_arm_time >= BIAS_ARM_TIME:
        change = float(np.clip(BIAS_KI * error, -BIAS_BUILD_RATE, BIAS_BUILD_RATE)) * DT
        candidate = float(np.clip(self.bias + change, -BIAS_MAX, BIAS_MAX))
        total = base + candidate
        if abs(total) <= MAX_CORRECTION or change * total < 0.0:
          self.bias = candidate
    return self.bias

  def _track_turn(self, e2e: float) -> None:
    if abs(e2e) >= TURN_THRESHOLD:
      direction = sign(e2e)
      if self.last_turn_sign and direction != self.last_turn_sign:
        self.turn_release = TURN_RELEASE_TIME
      self.last_turn_sign = direction

  def _guard_correction(self, correction: float, e2e: float) -> tuple[float, bool]:
    """Release stale alignment during lane-target or E2E turn reversals."""
    target_sign = sign(correction) if abs(correction) >= CORRECTION_DEADBAND else 0
    if abs(e2e) >= CURVE_THRESHOLD:
      if target_sign and self.last_curve_sign and target_sign != self.last_curve_sign:
        self.curve_release = CURVE_REVERSAL_TIME
      if target_sign:
        self.last_curve_sign = target_sign
    else:
      self.last_curve_sign = 0
    guarded = self.turn_release > 0.0 or self.curve_release > 0.0
    if guarded:
      self.reset_motion()
    if self.turn_release > 0.0:
      self.turn_release = max(0.0, self.turn_release - DT)
      if correction * e2e < 0.0:
        correction = 0.0
    if self.curve_release > 0.0:
      self.curve_release = max(0.0, self.curve_release - DT)
      correction = 0.0
    return correction, guarded

  def update(self, model_output: dict[str, np.ndarray], e2e_curvature: float, v_ego: float,
             blinkers_active: bool = False, lane_policy_enabled: bool = False,
             lateral_active: bool = False, steering_pressed: bool = False, frame_time: float | None = None) -> float:
    if not lane_policy_enabled:
      return self._fallback(e2e_curvature, "stock-e2e (lane toggle off)")
    if blinkers_active:
      return self._fallback(e2e_curvature, "stock-e2e fallback: blinker")
    try:
      # Desire slots 3/4 are lane-change left/right in the model output layout.
      desire = model_output['desire_state'][0]
      if float(desire[3] + desire[4]) > MAX_LANE_CHANGE_PROB:
        return self._fallback(e2e_curvature, "stock-e2e fallback: lane-change intent")
      x = np.asarray(ModelConstants.X_IDXS, dtype=np.float64)
      center, strong = self._centerline(model_output, x)
      if center is None:
        return self._fallback(e2e_curvature, "stock-e2e fallback: lane geometry or confidence")
      if not self.active:
        self.arm_time = min(self.arm_time + DT, ARM_TIME) if strong else 0.0
        if self.arm_time < ARM_TIME:
          self._mode("stock-e2e fallback: arming lane confidence")
          return float(e2e_curvature)
        self.active, self.line_loss_time = True, 0.0

      correction, error, lookahead = anchor_correction(model_output, center, x, v_ego)
      offset, heading = near_geometry(center, x)
      self._track_turn(e2e_curvature)
      feedback_allowed = lateral_active and not steering_pressed and strong and v_ego >= 10.0 and abs(e2e_curvature) * v_ego ** 2 < 1.5
      motion_allowed = feedback_allowed and abs(offset) <= BIAS_MAX_OFFSET and self.turn_release <= 0.0 and self.curve_release <= 0.0
      geometric_rate = v_ego * heading
      rate, ready = self.update_motion(offset, geometric_rate, motion_allowed, frame_time)
      if strong and correction * offset > 0.0:
        correction *= approach_scale(offset, rate)
      correction, guarded = self._guard_correction(soften_correction(correction), e2e_curvature)
      if guarded:
        rate, ready = geometric_rate, False
      correction += self.update_bias(offset, rate, correction, feedback_allowed and ready)
      correction = float(np.clip(correction, -MAX_CORRECTION, MAX_CORRECTION))
      # Smooth entry, with faster release when an old correction becomes stale.
      releasing = abs(correction) < abs(self.correction) or correction * self.correction < 0.0
      step = RELEASE_STEP if releasing else ENGAGE_STEP
      self.correction += float(np.clip(correction - self.correction, -step, step))
      self._record(lookahead, error, e2e_curvature, offset, rate, geometric_rate, ready)
      self.error_logged = False
      self._mode("anchor lane center hold" if self.holding_line else "E2E lane anchor")
      return float(e2e_curvature + self.correction)
    except (KeyError, IndexError, TypeError, ValueError, FloatingPointError, np.linalg.LinAlgError) as err:
      if not self.error_logged and self.logger is not None:
        self.logger.warning(f"ui-lp-full-center input error: {type(err).__name__}: {err}")
      self.error_logged = True
      return self._fallback(e2e_curvature, "stock-e2e fallback: lane-policy input error")
