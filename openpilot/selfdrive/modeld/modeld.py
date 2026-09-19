#!/usr/bin/env python3
from collections.abc import Callable
import ctypes
from functools import cached_property
import os
os.environ['GMMU'] = '0' # for chestnut fast loading, noop for qcom
from tinygrad.device import Buffer, Device
from tinygrad.dtype import DType, dtypes
from tinygrad.tensor import Tensor
from tinygrad.helpers import round_up
from tinygrad.uop.ops import UOp
import math
import pickle
import threading
import time
import numpy as np
import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from opendbc.car.structs import car
from openpilot.cereal.messaging import PubMaster, SubMaster
from openpilot.cereal.services import SERVICE_LIST
from openpilot.cereal.visionipc import VisionStreamType
from msgq.visionipc import VisionIpcClient, VisionBuf
from opendbc.car.car_helpers import get_demo_car_params
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import config_realtime_process, DT_MDL
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
from openpilot.common.transformations.model import get_warp_matrix
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.controls.lib.drive_helpers import get_accel_from_plan, should_stop, smooth_value, get_curvature_from_plan
from openpilot.selfdrive.modeld.parse_model_outputs import Parser
from openpilot.selfdrive.modeld.fill_model_msg import fill_model_msg, fill_driving_model_data, fill_pose_msg, PublishState
from openpilot.common.file_chunker import open_file_chunked
from openpilot.selfdrive.modeld.constants import ModelConstants, Plan
from openpilot.selfdrive.modeld.helpers import MODELS_DIR, chestnut_present, chestnut_compiled, modeld_pkl_path, load_oob

SEND_RAW_PRED = os.getenv('SEND_RAW_PRED')

LAT_SMOOTH_SECONDS = 0.0
LONG_SMOOTH_SECONDS = 0.3
MIN_LAT_CONTROL_SPEED = 0.3
BIG_MODEL_TIMEOUT = 60


# On-road UI lane-policy toggle. The lane policy is selected by default each
# drive; the HUD selector selects exact upstream E2E. Once both inner lines have been
# clean long enough, it applies a full lane-center correction while retaining
# E2E curvature as the road-shape feed-forward target.
LANE_POLICY_ENABLED_PARAM = "LanePolicyEnabled"

# Two-line confidence uses entry/exit hysteresis. It prevents a clean lane from
# dropping to E2E merely because one model frame is slightly less certain.
LANE_LOCK_ARM_LINE_PROB = 0.92
LANE_LOCK_RETAIN_LINE_PROB = 0.70
LANE_LOCK_MIN_LANE_WIDTH = 2.6                 # m
LANE_LOCK_MAX_LANE_WIDTH = 5.2                 # m
LANE_LOCK_MIN_WIDTH_EDGE = 0.10                # m inside accepted range
LANE_LOCK_MAX_WIDTH_CHANGE = 0.75              # m across the fitted horizon
LANE_LOCK_ARM_TIME = 0.75                      # s of clean two-line confidence
LANE_LOCK_WIDTH_TIME = 9.95                    # s, matches the old lane planner
LANE_LOCK_ONE_LINE_HOLD_TIME = 1.00            # s using the learned lane width
LANE_LOCK_FIT_START = 8.0                      # m
LANE_LOCK_FIT_END = 55.0                       # m
LANE_LOCK_MIN_LOOKAHEAD = 25.0                 # m
LANE_LOCK_MAX_LOOKAHEAD = 45.0                 # m
LANE_LOCK_HEADING_GAIN = 0.55
# A short geometric prediction eases the offset term while approaching center.
# It adds no filter state or delay and cannot remove more than 35% of that term.
LANE_LOCK_APPROACH_TIME = 0.20                 # s
LANE_LOCK_APPROACH_MIN_TIME = 0.17             # s
LANE_LOCK_APPROACH_MAX_TIME = 0.25             # s
LANE_LOCK_DELAY_REFERENCE = 0.30              # s; ~0.302 s in the reference drive
LANE_LOCK_DELAY_GAIN = 0.25                   # schedule a fraction, not the full compensated delay
LANE_LOCK_DELAY_MIN = 0.15                    # lagd's accepted range
LANE_LOCK_DELAY_MAX = 0.65
LANE_LOCK_DELAY_MAX_AGE = 1.0                 # s; lateralDelay publishes at 4 Hz
LANE_LOCK_TIMING_TIME_CONSTANT = 2.0          # s; filters the tuning parameter only
LANE_LOCK_TIMING_MAX_RATE = 0.01              # s of anticipation / s
LANE_LOCK_APPROACH_MAX_FRACTION = 0.35
LANE_LOCK_APPROACH_FULL_OFFSET = 0.25          # m
LANE_LOCK_APPROACH_END_OFFSET = 0.50           # m
# E2E supplies the road-shape feed-forward. Fade only the added lane-heading
# contribution in a substantial E2E curve; lane-center offset authority stays
# intact so the policy can still center the car without fighting the turn.
LANE_LOCK_HEADING_CURVATURE_FADE = 0.00060      # 1/m
LANE_LOCK_CURVE_REVERSAL_HOLD_TIME = 0.30       # s
LANE_LOCK_MAX_CENTER_CORRECTION = 0.00045      # 1/m
LANE_LOCK_TURN_CURVATURE = 0.00015             # 1/m
LANE_LOCK_TURN_RELEASE_TIME = 0.35             # s
# Build lane authority deliberately, but release it faster when the lane
# midpoint says the previous correction is no longer needed. This prevents a
# curve-exit or lane-change correction from lingering past the centerline.
LANE_LOCK_CORRECTION_ENGAGE_STEP = 0.00008      # 1/m per model frame
LANE_LOCK_CORRECTION_RELEASE_STEP = 0.00020     # 1/m per model frame
LANE_LOCK_CORRECTION_DEADBAND = 0.000012        # 1/m
LANE_LOCK_MAX_LANE_CHANGE_PROB = 0.10
LANE_LOCK_LOG_INTERVAL = 1.0                   # seconds

# The legacy names are retained because the HUD publisher already reads them.
# _lane_lock_weight is binary (0/1), not an E2E/lane output blend.
_lane_lock_weight = 0.0
_lane_lock_lane_curvature = 0.0
_lane_lock_has_lane_curvature = False
_lane_lock_full_active = False
_lane_lock_ready = False
_lane_lock_arm_time = 0.0
_lane_lock_width = 3.7
_lane_lock_width_valid = False
_lane_lock_line_loss_time = 0.0
_lane_lock_center_correction = 0.0
_lane_lock_has_center_correction = False
_lane_lock_one_line_hold = False
_lane_lock_error_logged = False
_lane_lock_last_turn_sign = 0
_lane_lock_turn_release_time = 0.0
_lane_lock_last_curve_target_sign = 0
_lane_lock_curve_reversal_hold_time = 0.0
_lane_lock_last_logged_mode = None
_lane_lock_last_log_time = 0.0


def reset_lane_lock() -> None:
  """Discard all lane-policy state so the caller immediately receives raw E2E."""
  global _lane_lock_weight, _lane_lock_lane_curvature
  global _lane_lock_has_lane_curvature, _lane_lock_full_active
  global _lane_lock_ready, _lane_lock_arm_time
  global _lane_lock_width, _lane_lock_width_valid, _lane_lock_line_loss_time
  global _lane_lock_center_correction, _lane_lock_has_center_correction
  global _lane_lock_one_line_hold, _lane_lock_last_turn_sign, _lane_lock_turn_release_time
  global _lane_lock_last_curve_target_sign, _lane_lock_curve_reversal_hold_time
  _lane_lock_weight = 0.0
  _lane_lock_lane_curvature = 0.0
  _lane_lock_has_lane_curvature = False
  _lane_lock_full_active = False
  _lane_lock_ready = False
  _lane_lock_arm_time = 0.0
  _lane_lock_width = 3.7
  _lane_lock_width_valid = False
  _lane_lock_line_loss_time = 0.0
  _lane_lock_center_correction = 0.0
  _lane_lock_has_center_correction = False
  _lane_lock_one_line_hold = False
  _lane_lock_last_turn_sign = 0
  _lane_lock_turn_release_time = 0.0
  _lane_lock_last_curve_target_sign = 0
  _lane_lock_curve_reversal_hold_time = 0.0


def log_lane_lock_mode(mode: str) -> None:
  """Log state transitions at a bounded rate for rlog validation."""
  global _lane_lock_last_logged_mode, _lane_lock_last_log_time
  now = time.monotonic()
  if mode != _lane_lock_last_logged_mode and now - _lane_lock_last_log_time >= LANE_LOCK_LOG_INTERVAL:
    cloudlog.info(f"lp-final-v3: {mode}")
    _lane_lock_last_logged_mode = mode
    _lane_lock_last_log_time = now


def get_inner_lane_line_probs(model_output: dict[str, np.ndarray]) -> tuple[float, float]:
  """Return inner left/right raw-model probabilities from the 8-wide layout."""
  lane_line_probs = np.asarray(model_output['lane_lines_prob'])
  if lane_line_probs.shape != (1, 8):
    raise ValueError(f"expected lane_lines_prob shape (1, 8), got {lane_line_probs.shape}")
  return float(lane_line_probs[0, 3]), float(lane_line_probs[0, 5])


def get_lane_policy_enabled(params: Params) -> bool:
  """Read the per-drive selector, correctly decoding Params' b"0"/b"1" value."""
  value = params.get(LANE_POLICY_ENABLED_PARAM)
  if value is None:
    return True
  if isinstance(value, (bytes, bytearray)):
    return value == b"1"
  return bool(value)


def get_lane_width_measurement(left_y: np.ndarray, right_y: np.ndarray,
                               fit: np.ndarray) -> tuple[float, bool]:
  """Return a robust width measurement and whether the pair is geometrically usable."""
  lane_width = right_y - left_y
  width_p10, width_median, width_p90 = np.percentile(lane_width[fit], (10.0, 50.0, 90.0))
  width_change = float(width_p90 - width_p10)
  width_edge_distance = min(width_p10 - LANE_LOCK_MIN_LANE_WIDTH,
                            LANE_LOCK_MAX_LANE_WIDTH - width_p90)
  valid = (LANE_LOCK_MIN_LANE_WIDTH <= width_median <= LANE_LOCK_MAX_LANE_WIDTH and
           width_edge_distance >= LANE_LOCK_MIN_WIDTH_EDGE and
           width_change <= LANE_LOCK_MAX_WIDTH_CHANGE)
  return float(width_median), bool(valid)


class LanePolicyApproachTiming:
  """Slowly schedule the extra lane correction's anticipation, not E2E delay.

  Model action_t and the torque controller already compensate lateralDelay.
  This empirical, bounded schedule only adjusts the added lane-offset damping
  around the working 0.20 s tune. It is not a second actuator-delay correction.
  Keep this state across lane-policy transitions; never filter lane geometry.
  """
  def __init__(self):
    self.approach_time = LANE_LOCK_APPROACH_TIME
    self.using_estimate = False

  def update(self, delay: float, estimated: bool, valid: bool, age: float) -> float:
    self.using_estimate = (estimated and valid and math.isfinite(delay) and math.isfinite(age) and
                           LANE_LOCK_DELAY_MIN <= delay <= LANE_LOCK_DELAY_MAX and
                           0.0 <= age <= LANE_LOCK_DELAY_MAX_AGE)
    target = LANE_LOCK_APPROACH_TIME
    if self.using_estimate:
      target += LANE_LOCK_DELAY_GAIN * (delay - LANE_LOCK_DELAY_REFERENCE)
    target = float(np.clip(target, LANE_LOCK_APPROACH_MIN_TIME, LANE_LOCK_APPROACH_MAX_TIME))
    delta = DT_MDL / (LANE_LOCK_TIMING_TIME_CONSTANT + DT_MDL) * (target - self.approach_time)
    max_step = LANE_LOCK_TIMING_MAX_RATE * DT_MDL
    self.approach_time += float(np.clip(delta, -max_step, max_step))
    return self.approach_time


def get_lane_lock_approach_offset(offset: float, heading: float, v_ego: float,
                                  heading_scale: float, confidence: float,
                                  approach_time: float = LANE_LOCK_APPROACH_TIME) -> float:
  """Ease only the offset term when lane-relative heading indicates convergence.

  For small lane-relative angles, offset rate is approximately speed * heading.
  Keep the current measured geometry: no delayed offset/heading filters and no
  frame-to-frame differentiation of noisy lane fits. A parallel offset or motion
  away from center gets exactly the original centering authority. The prediction
  is bounded, fades with confidence/curve heading authority, and cannot reverse
  the offset term. The existing heading term still provides countersteering.
  """
  if offset * heading >= 0.0:
    return offset
  offset_weight = float(np.clip((LANE_LOCK_APPROACH_END_OFFSET - abs(offset)) /
                               (LANE_LOCK_APPROACH_END_OFFSET - LANE_LOCK_APPROACH_FULL_OFFSET), 0.0, 1.0))
  confidence_weight = float(np.clip((confidence - LANE_LOCK_RETAIN_LINE_PROB) /
                                   (LANE_LOCK_ARM_LINE_PROB - LANE_LOCK_RETAIN_LINE_PROB), 0.0, 1.0))
  speed_weight = float(np.clip((v_ego - 8.0) / 4.0, 0.0, 1.0))
  if not math.isfinite(approach_time):
    approach_time = LANE_LOCK_APPROACH_TIME
  approach_time = float(np.clip(approach_time, LANE_LOCK_APPROACH_MIN_TIME, LANE_LOCK_APPROACH_MAX_TIME))
  approach = min(approach_time * max(v_ego, 0.0) * abs(heading) * heading_scale,
                 LANE_LOCK_APPROACH_MAX_FRACTION * abs(offset))
  return offset - math.copysign(approach * offset_weight * confidence_weight * speed_weight, offset)


def apply_lane_lock(model_output: dict[str, np.ndarray], e2e_curvature: float, v_ego: float,
                    blinkers_active: bool = False, lane_policy_enabled: bool = False,
                    approach_time: float = LANE_LOCK_APPROACH_TIME) -> float:
  """Anchor E2E progressively to a stable lane midpoint without output blending.

  E2E remains the road-shape feed-forward command. Once a clean two-line lane
  has armed, the policy adds a bounded correction from the lane-center offset
  and heading at a longer spatial anchor. That avoids the lagging filtered lane
  curvature target that caused the previous policy to overshoot on a straight
  road. A temporarily missing side uses the learned, slowly filtered lane width
  for one second; blinkers and lane-change intent always return exact E2E.
  """
  global _lane_lock_weight, _lane_lock_lane_curvature
  global _lane_lock_has_lane_curvature, _lane_lock_full_active
  global _lane_lock_ready, _lane_lock_arm_time
  global _lane_lock_width, _lane_lock_width_valid, _lane_lock_line_loss_time
  global _lane_lock_center_correction, _lane_lock_has_center_correction
  global _lane_lock_one_line_hold, _lane_lock_error_logged
  global _lane_lock_last_turn_sign, _lane_lock_turn_release_time
  global _lane_lock_last_curve_target_sign, _lane_lock_curve_reversal_hold_time

  if not lane_policy_enabled:
    reset_lane_lock()
    log_lane_lock_mode("stock-e2e (lane toggle off)")
    return float(e2e_curvature)

  if blinkers_active:
    reset_lane_lock()
    log_lane_lock_mode("stock-e2e fallback: blinker")
    return float(e2e_curvature)

  try:
    # Lane axes are [batch, lane, distance, coordinate]. The inner lines are
    # lane 1 (left) and 2 (right); their raw confidences are entries 3 and 5.
    left_y = model_output['lane_lines'][0, 1, :, 0].astype(np.float64)
    right_y = model_output['lane_lines'][0, 2, :, 0].astype(np.float64)
    left_prob, right_prob = get_inner_lane_line_probs(model_output)
    desire_state = model_output['desire_state'][0]
    lane_change_prob = float(desire_state[log.Desire.laneChangeLeft] +
                             desire_state[log.Desire.laneChangeRight])
    if lane_change_prob > LANE_LOCK_MAX_LANE_CHANGE_PROB:
      reset_lane_lock()
      log_lane_lock_mode("stock-e2e fallback: lane-change intent")
      return float(e2e_curvature)

    x = np.asarray(ModelConstants.X_IDXS, dtype=np.float64)
    fit = (x >= LANE_LOCK_FIT_START) & (x <= LANE_LOCK_FIT_END)
    if x.shape != left_y.shape or np.count_nonzero(fit) < 3:
      raise ValueError("lane-line horizon does not match ModelConstants.X_IDXS")

    valid_left = (np.isfinite(left_prob) and left_prob >= LANE_LOCK_RETAIN_LINE_PROB and
                  np.all(np.isfinite(left_y[fit])))
    valid_right = (np.isfinite(right_prob) and right_prob >= LANE_LOCK_RETAIN_LINE_PROB and
                   np.all(np.isfinite(right_y[fit])))
    two_line_geometry = False
    two_line_confidence = min(left_prob, right_prob)
    center_y: np.ndarray | None = None

    if valid_left and valid_right:
      measured_width, two_line_geometry = get_lane_width_measurement(left_y, right_y, fit)
      if two_line_geometry:
        center_y = 0.5 * (left_y + right_y)
        _lane_lock_line_loss_time = 0.0
        _lane_lock_one_line_hold = False

        # Preserve the old lane planner's long (~10 s) width memory. It is
        # only updated from strong, two-line geometry, never from a single line.
        if two_line_confidence >= LANE_LOCK_ARM_LINE_PROB:
          if not _lane_lock_width_valid:
            _lane_lock_width = measured_width
            _lane_lock_width_valid = True
          else:
            alpha = min(DT_MDL / LANE_LOCK_WIDTH_TIME, 1.0)
            _lane_lock_width += alpha * (measured_width - _lane_lock_width)

    if center_y is None and _lane_lock_full_active and _lane_lock_width_valid:
      # If a side is briefly clipped (intersections, dashed paint, shadows),
      # reconstruct its midpoint from the remaining reliable line and learned
      # width. If both visible lines disagree about width, choose the more
      # confident side rather than injecting their bad midpoint.
      selected_left = valid_left and (not valid_right or not two_line_geometry or left_prob >= right_prob)
      selected_right = valid_right and not selected_left
      if selected_left or selected_right:
        _lane_lock_line_loss_time += DT_MDL
        if _lane_lock_line_loss_time <= LANE_LOCK_ONE_LINE_HOLD_TIME:
          center_y = (left_y + _lane_lock_width / 2.0 if selected_left else
                      right_y - _lane_lock_width / 2.0)
          _lane_lock_one_line_hold = True

    if center_y is None:
      reset_lane_lock()
      log_lane_lock_mode("stock-e2e fallback: lane geometry or confidence")
      return float(e2e_curvature)

    if not _lane_lock_full_active:
      if two_line_geometry and two_line_confidence >= LANE_LOCK_ARM_LINE_PROB:
        _lane_lock_arm_time = min(_lane_lock_arm_time + DT_MDL, LANE_LOCK_ARM_TIME)
        _lane_lock_ready = _lane_lock_arm_time >= LANE_LOCK_ARM_TIME
      else:
        _lane_lock_arm_time = 0.0

      if not _lane_lock_ready:
        _lane_lock_has_lane_curvature = False
        _lane_lock_one_line_hold = False
        log_lane_lock_mode("stock-e2e fallback: arming lane confidence")
        return float(e2e_curvature)

      _lane_lock_full_active = True
      _lane_lock_weight = 1.0
      _lane_lock_line_loss_time = 0.0

    # The center fit uses a farther, measured horizon. E2E keeps the curve
    # feed-forward. As E2E curvature grows, only the added lane-heading term
    # fades; lane-center offset remains available for full centering. This is
    # deliberately not an E2E/lane-curvature blend.
    _, heading, offset = np.polyfit(x[fit], center_y[fit], 2)
    lookahead = float(np.clip(2.0 * v_ego, LANE_LOCK_MIN_LOOKAHEAD, LANE_LOCK_MAX_LOOKAHEAD))
    heading_scale = min(1.0, LANE_LOCK_HEADING_CURVATURE_FADE / max(abs(e2e_curvature), 1e-6))
    # Only a valid two-line midpoint supplies approach damping. Preserve the
    # original learned-width behavior during a one-line hold.
    approach_offset = (get_lane_lock_approach_offset(float(offset), float(heading), v_ego,
                                                    heading_scale, two_line_confidence, approach_time)
                       if two_line_geometry else float(offset))
    center_correction = (2.0 * (LANE_LOCK_HEADING_GAIN * heading_scale * heading * lookahead + approach_offset) /
                         (lookahead * lookahead))
    center_correction = float(np.clip(center_correction,
                                      -LANE_LOCK_MAX_CENTER_CORRECTION,
                                      LANE_LOCK_MAX_CENTER_CORRECTION))
    if abs(center_correction) < LANE_LOCK_CORRECTION_DEADBAND:
      center_correction = 0.0

    # A lane-line fit can move its heading through zero at a curve entry while
    # E2E correctly remains in the same turn. Do not ask the controller to
    # reverse a meaningful lane correction immediately: first release toward
    # E2E, then require the new lane target to stay stable briefly. This avoids
    # the torque-controller lag/wag seen in the curve-entry rlogs.
    target_sign = 1 if center_correction > 0.0 else -1 if center_correction < 0.0 else 0
    if abs(e2e_curvature) >= LANE_LOCK_HEADING_CURVATURE_FADE:
      if (target_sign and _lane_lock_last_curve_target_sign and
          target_sign != _lane_lock_last_curve_target_sign):
        _lane_lock_curve_reversal_hold_time = LANE_LOCK_CURVE_REVERSAL_HOLD_TIME
      if target_sign:
        _lane_lock_last_curve_target_sign = target_sign
    else:
      _lane_lock_last_curve_target_sign = 0

    # On a meaningful E2E turn-direction change, drop only a correction
    # that still asks for the previous turn. This preserves full lane
    # centering during steady curves while avoiding a direction-change tug.
    if abs(e2e_curvature) >= LANE_LOCK_TURN_CURVATURE:
      e2e_turn_sign = 1 if e2e_curvature > 0.0 else -1
      if _lane_lock_last_turn_sign and e2e_turn_sign != _lane_lock_last_turn_sign:
        _lane_lock_turn_release_time = LANE_LOCK_TURN_RELEASE_TIME
      _lane_lock_last_turn_sign = e2e_turn_sign
    if _lane_lock_turn_release_time > 0.0:
      _lane_lock_turn_release_time = max(0.0, _lane_lock_turn_release_time - DT_MDL)
      if center_correction * e2e_curvature < 0.0:
        center_correction = 0.0

    if _lane_lock_curve_reversal_hold_time > 0.0:
      _lane_lock_curve_reversal_hold_time = max(0.0, _lane_lock_curve_reversal_hold_time - DT_MDL)
      center_correction = 0.0

    # Enter smoothly after arming. Build correction deliberately, but release
    # it faster when the target shrinks or reverses after a curve/lane change.
    if not _lane_lock_has_center_correction:
      _lane_lock_center_correction = 0.0
      _lane_lock_has_center_correction = True

    correction_step = (LANE_LOCK_CORRECTION_RELEASE_STEP
                       if (abs(center_correction) < abs(_lane_lock_center_correction) or
                           center_correction * _lane_lock_center_correction < 0.0)
                       else LANE_LOCK_CORRECTION_ENGAGE_STEP)
    delta = float(np.clip(center_correction - _lane_lock_center_correction,
                          -correction_step, correction_step))
    _lane_lock_center_correction += delta

    _lane_lock_lane_curvature = float(e2e_curvature + _lane_lock_center_correction)
    _lane_lock_has_lane_curvature = True
    _lane_lock_weight = 1.0
    _lane_lock_full_active = True
    _lane_lock_ready = True
    _lane_lock_error_logged = False
    log_lane_lock_mode("lane center hold" if _lane_lock_one_line_hold else "full lane center")
    return _lane_lock_lane_curvature

  except (KeyError, IndexError, TypeError, ValueError, FloatingPointError, np.linalg.LinAlgError) as err:
    reset_lane_lock()
    if not _lane_lock_error_logged:
      cloudlog.warning(f"ui-lp-full-center input error: {type(err).__name__}: {err}")
      _lane_lock_error_logged = True
    log_lane_lock_mode("stock-e2e fallback: lane-policy input error")
    return float(e2e_curvature)


def get_action_from_model(model_output: dict[str, np.ndarray], prev_action: log.ModelDataV2.Action,
                          lat_action_t: float, long_action_t: float, v_ego: float,
                          blinkers_active: bool = False, lane_policy_enabled: bool = False,
                          approach_time: float = LANE_LOCK_APPROACH_TIME) -> log.ModelDataV2.Action:
  if 'action' not in model_output:
    plan = model_output['plan'][0]
    desired_accel = get_accel_from_plan(plan[:,Plan.VELOCITY][:,0],
                                        plan[:,Plan.ACCELERATION][:,0],
                                        ModelConstants.T_IDXS,
                                        action_t=long_action_t)
    desired_curvature = get_curvature_from_plan(plan[:,Plan.T_FROM_CURRENT_EULER][:,2],
                                                plan[:,Plan.ORIENTATION_RATE][:,2],
                                                ModelConstants.T_IDXS,
                                                v_ego,
                                                lat_action_t)
  else:
    desired_accel = model_output['action'][0,1]
    desired_curvature = model_output['action'][0,0] / (max(1.0, v_ego))**2
  desired_curvature = apply_lane_lock(model_output, desired_curvature, v_ego,
                                      blinkers_active, lane_policy_enabled, approach_time)
  stop = should_stop(v_ego, desired_accel)
  desired_accel = smooth_value(desired_accel, prev_action.desiredAcceleration, LONG_SMOOTH_SECONDS)
  if v_ego > MIN_LAT_CONTROL_SPEED:
    desired_curvature = smooth_value(desired_curvature, prev_action.desiredCurvature, LAT_SMOOTH_SECONDS)
  else:
    desired_curvature = prev_action.desiredCurvature

  return log.ModelDataV2.Action(desiredCurvature=float(desired_curvature),
                                desiredAcceleration=float(desired_accel),
                                shouldStop=bool(stop))


class ChestnutGpuState:
  # GPU metrics require modeld's GPU context
  def __init__(self, pm: PubMaster, big: bool):
    self.pm = pm
    self.big = big
    self.valid = True
    self.sends = 0
    self.metrics = {}

  @cached_property
  def power_limit(self) -> int:
    smu = Device["AMD"].iface.dev_impl.smu
    return smu._send_msg(smu.smu_mod.PPSMC_MSG_GetPptLimit, 0, read_back_arg=True, timeout=100)

  def send(self) -> None:
    msg = messaging.new_message('chestnutGpuState')
    state = msg.chestnutGpuState
    self.sends += 1
    if self.big and "AMD" in Device._opened_devices and self.sends % 100 == 1:
      try:
        smu = Device["AMD"].iface.dev_impl.smu
        metrics_t = smu.smu_mod.SmuMetricsExternal_t
        smu._send_msg(smu.smu_mod.PPSMC_MSG_TransferTableSmu2Dram, smu.smu_mod.TABLE_SMU_METRICS, timeout=100)
        metrics_buf = bytearray(smu.adev.vram.view(smu.driver_table_paddr, ctypes.sizeof(metrics_t))[:])
        metrics = metrics_t.from_buffer(metrics_buf).SmuMetrics
        self.metrics = {'tempC': metrics.AvgTemperature[smu.smu_mod.TEMP_HOTSPOT],
                        'memoryTempC': metrics.AvgTemperature[smu.smu_mod.TEMP_MEM],
                        'powerDrawW': metrics.AverageSocketPower,
                        'powerLimitW': self.power_limit,
                        'gpuUsagePercent': metrics.AverageGfxActivity,
                        'gpuClockMhz': metrics.AverageGfxclkFrequencyPostDs,
                        'fanSpeedRpm': metrics.AvgFanRpm}
        self.valid = True
      except Exception:
        if self.valid:
          cloudlog.exception("chestnut state read failed")
        self.valid = False
        self.metrics.clear()
    if self.big:
      for k, v in self.metrics.items():
        setattr(state, k, v)

    msg.valid = not self.big or (self.valid and bool(self.metrics))
    self.pm.send('chestnutGpuState', msg)


class FrameMeta:
  frame_id: int = 0
  timestamp_sof: int = 0
  timestamp_eof: int = 0

  def __init__(self, vipc=None):
    if vipc is not None:
      self.frame_id, self.timestamp_sof, self.timestamp_eof = vipc.frame_id, vipc.timestamp_sof, vipc.timestamp_eof


def input_view(buffer: Buffer, shape: tuple[int, ...], dtype: DType, offset: int) -> Tensor:
  view = buffer.view(math.prod(shape), dtype, offset).ensure_allocated()
  return Tensor(UOp.from_buffer(view)).reshape(shape)


class ModelState:
  prev_desire: np.ndarray  # for tracking the rising edge of the pulse

  def __init__(self, cam_w: int, cam_h: int, chestnut: bool):
    jits = load_oob(open_file_chunked(modeld_pkl_path(chestnut)))
    self.model_device = jits['input_devices']['model']
    self.input_shapes = jits['input_shapes']
    self.state_pairs = jits['state_pairs']
    self.vision_input_names = ('img', 'big_img')
    self.output_slices = jits['metadata']['output_slices']

    self.prev_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
    self.chestnut = chestnut

    stride, y_height, uv_height, _ = get_nv12_info(cam_w, cam_h)
    self.frame_copy_size = stride * (y_height + uv_height)
    self.pack_inputs()
    with open(MODELS_DIR / f'{"big_" if chestnut else ""}driving_warp_{cam_w}x{cam_h}_tinygrad.pkl', 'rb') as f:
      self.run_warp = pickle.load(f)
    self.run_model = jits['run_model']
    self.parser = Parser()

  def pack_inputs(self) -> None:
    # Pack host inputs into one upload to reduce USB transfer overhead for the eGPU.
    self.input_queues = {name: Tensor(np.zeros(shape, dtype=dtype.fmt), device=self.model_device).realize()
                         for name, (shape, dtype) in self.input_shapes.items() if name in self.state_pairs}
    shapes = {'tfm': (2, 3, 3)} | {name: shape for name, (shape, _) in self.input_shapes.items()
                                   if name not in self.state_pairs and name != 'new_img'}
    npy_size = sum(round_up(math.prod(shape) * 4, 128) for shape in shapes.values())
    self.packed_input = np.zeros(npy_size + 2 * self.frame_copy_size, dtype=np.uint8)
    self.input_host = Tensor(self.packed_input, device='NPY')._buffer()
    self.input_device = Tensor(self.packed_input, device=self.model_device)._buffer()
    self.npy = {}
    offset = 0
    for name, shape in shapes.items():
      self.npy[name] = np.ndarray(shape, dtype=np.float32, buffer=self.packed_input, offset=offset)
      self.input_queues[name] = input_view(self.input_device, shape, dtypes.float32, offset)
      offset += round_up(self.npy[name].nbytes, 128)
    self.frames = self.packed_input[npy_size:].reshape(2, self.frame_copy_size)
    self.warp_inputs = (input_view(self.input_device, self.frames.shape, dtypes.uint8, npy_size), self.input_queues.pop('tfm'))

  def slice_outputs(self, model_outputs: np.ndarray, output_slices: dict[str, slice]) -> dict[str, np.ndarray]:
    return {k: model_outputs[np.newaxis, v] for k,v in output_slices.items()}

  def run(self, bufs: dict[str, VisionBuf], transforms: dict[str, np.ndarray],
          inputs: dict[str, np.ndarray], after_enqueue: Callable[[], None] | None = None) -> dict[str, np.ndarray]:
    for i, key in enumerate(self.vision_input_names):
      np.copyto(self.frames[i], np.frombuffer(bufs[key].data, dtype=np.uint8, count=self.frame_copy_size))
      self.npy['tfm'][i] = transforms[key]

    # Model decides when action is completed, so desire input is just a pulse triggered on rising edge
    inputs['desire_pulse'][0] = 0
    self.npy['desire'][:] = np.where(inputs['desire_pulse'] - self.prev_desire > .99, inputs['desire_pulse'], 0)
    self.prev_desire[:] = inputs['desire_pulse']
    self.npy['traffic_convention'][:] = inputs['traffic_convention']
    self.npy['action_t'][:] = inputs['action_t']

    self.input_device.copy_from(self.input_host)
    self.input_queues['new_img'] = self.run_warp(*self.warp_inputs)
    outs, = self.run_model(**self.input_queues)
    if after_enqueue is not None:
      after_enqueue()
    model_output = outs.numpy()[0]
    if self.chestnut and not np.all(np.isfinite(model_output)):
      raise RuntimeError("model output not finite")
    outputs_dict = self.parser.parse_outputs(self.slice_outputs(model_output, self.output_slices))

    if SEND_RAW_PRED:
      outputs_dict['raw_pred'] = model_output.copy()
    return outputs_dict

  def warmup(self) -> None:
    dummy_frames = {k: np.zeros(self.frame_copy_size, dtype=np.uint8) for k in self.vision_input_names}
    eye = np.eye(3, dtype=np.float32)
    dims = {'desire_pulse': ModelConstants.DESIRE_LEN, 'traffic_convention': 2, 'action_t': 2}
    self.run(dummy_frames, dict.fromkeys(self.vision_input_names, eye), {k: np.zeros(v, dtype=np.float32) for k, v in dims.items()})
    self.packed_input[:] = 0
    for key in self.state_pairs:
      self.input_queues[key].assign(0).realize()
    self.prev_desire[:] = 0


def main(demo=False):
  cloudlog.warning("modeld init")

  CHESTNUT = chestnut_present() and chestnut_compiled()
  if CHESTNUT:
    os.environ['HCQDEV_WAIT_TIMEOUT_MS'] = '3000'
  params = Params()
  params.put_bool("ChestnutLoading", CHESTNUT)
  params.remove("ChestnutActive")

  config_realtime_process(7, 54)

  # visionipc clients
  while True:
    available_streams = VisionIpcClient.available_streams("camerad", block=False)
    if available_streams:
      use_extra_client = VisionStreamType.VISION_STREAM_WIDE_ROAD in available_streams and VisionStreamType.VISION_STREAM_NARROW_ROAD in available_streams
      main_wide_camera = VisionStreamType.VISION_STREAM_NARROW_ROAD not in available_streams
      break
    time.sleep(.1)

  vipc_client_main_stream = VisionStreamType.VISION_STREAM_WIDE_ROAD if main_wide_camera else VisionStreamType.VISION_STREAM_NARROW_ROAD
  vipc_client_main = VisionIpcClient("camerad", vipc_client_main_stream, True)
  vipc_client_extra = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_WIDE_ROAD, False)
  cloudlog.warning(f"vision stream set up, main_wide_camera: {main_wide_camera}, use_extra_client: {use_extra_client}")

  while not vipc_client_main.connect(False):
    time.sleep(0.1)
  while use_extra_client and not vipc_client_extra.connect(False):
    time.sleep(0.1)

  cloudlog.warning(f"connected main cam with buffer size: {vipc_client_main.buffer_len} ({vipc_client_main.width} x {vipc_client_main.height})")
  if use_extra_client:
    cloudlog.warning(f"connected extra cam with buffer size: {vipc_client_extra.buffer_len} ({vipc_client_extra.width} x {vipc_client_extra.height})")

  st = time.monotonic()
  cloudlog.warning("loading model")
  model = None
  if CHESTNUT:
    big_model = None
    def load_big():
      nonlocal big_model
      try:
        m = ModelState(vipc_client_main.width, vipc_client_main.height, True)
        m.warmup()
        big_model = m
      except Exception:
        cloudlog.exception("big model load failed")
    loader = threading.Thread(target=load_big, daemon=True)
    loader.start()
    loader.join(BIG_MODEL_TIMEOUT)
    model = big_model
    params.put_bool("ChestnutActive", model is not None)

  small_model = ModelState(vipc_client_main.width, vipc_client_main.height, False) if model is None or CHESTNUT else None
  if model is None:
    model = small_model
  params.put_bool("ChestnutLoading", False)
  cloudlog.warning(f"models loaded in {time.monotonic() - st:.1f}s, modeld starting")

  # messaging
  pub_socks = ["modelV2", "drivingModelData", "cameraOdometry"] + (["chestnutGpuState"] if CHESTNUT else [])
  pm = PubMaster(pub_socks)
  sm = SubMaster(["deviceState", "carState", "narrowRoadCameraState", "extrinsicsCalibration", "driverMonitoringState", "carControl", "lateralDelay"])

  publish_state = PublishState()
  params = Params()
  chestnut_state = ChestnutGpuState(pm, model.chestnut) if CHESTNUT else None

  # setup filter to track dropped frames
  frame_dropped_filter = FirstOrderFilter(0., 10., 1. / ModelConstants.MODEL_RUN_FREQ)
  frame_id = 0
  last_vipc_frame_id = 0
  run_count = 0

  model_transform_main = np.zeros((3, 3), dtype=np.float32)
  model_transform_extra = np.zeros((3, 3), dtype=np.float32)
  extrinsics_calibration_seen = False
  buf_main, buf_extra = None, None
  meta_main = FrameMeta()
  meta_extra = FrameMeta()

  if demo:
    CP = get_demo_car_params()
  else:
    CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  cloudlog.info("modeld got CarParams: %s", CP.brand)

  # TODO this needs more thought, use .2s extra for now to estimate other delays
  # TODO Move smooth seconds to action function
  long_delay = CP.longitudinalActuatorDelay + LONG_SMOOTH_SECONDS
  prev_action = log.ModelDataV2.Action()
  lane_approach_timing = LanePolicyApproachTiming()
  last_lane_timing_log = 0.0

  DH = DesireHelper()
  lane_policy_enabled = get_lane_policy_enabled(params)
  last_published_lane_policy_active: bool | None = None
  last_published_lane_policy_blending: bool | None = None
  params.put_bool("LanePolicyActive", False)
  params.put_bool("LanePolicyBlending", False)

  while True:
    # Keep receiving frames until we are at least 1 frame ahead of previous extra frame
    while meta_main.timestamp_sof < meta_extra.timestamp_sof + 25000000:
      buf_main = vipc_client_main.recv()
      meta_main = FrameMeta(vipc_client_main)
      if buf_main is None:
        break

    if buf_main is None:
      cloudlog.debug("vipc_client_main no frame")
      continue

    if use_extra_client:
      # Keep receiving extra frames until frame id matches main camera
      while True:
        buf_extra = vipc_client_extra.recv()
        meta_extra = FrameMeta(vipc_client_extra)
        if buf_extra is None or meta_main.timestamp_sof < meta_extra.timestamp_sof + 25000000:
          break

      if buf_extra is None:
        cloudlog.debug("vipc_client_extra no frame")
        continue

      if abs(meta_main.timestamp_sof - meta_extra.timestamp_sof) > 10000000:
        cloudlog.error(f"frames out of sync! main: {meta_main.frame_id} ({meta_main.timestamp_sof / 1e9:.5f}),\
                         extra: {meta_extra.frame_id} ({meta_extra.timestamp_sof / 1e9:.5f})")

    else:
      # Use single camera
      buf_extra = buf_main
      meta_extra = meta_main

    sm.update(0)
    desire = DH.desire
    is_rhd = sm["driverMonitoringState"].isRHD
    frame_id = sm["narrowRoadCameraState"].frameId
    v_ego = max(sm["carState"].vEgo, 0.)
    lat_delay = sm["lateralDelay"].lateralDelay + LAT_SMOOTH_SECONDS
    if sm.updated["extrinsicsCalibration"] and sm.seen['narrowRoadCameraState'] and sm.seen['deviceState']:
      device_from_calib_euler = np.array(sm["extrinsicsCalibration"].rpyCalib, dtype=np.float32)
      dc = DEVICE_CAMERAS[(str(sm['deviceState'].deviceType), str(sm['narrowRoadCameraState'].sensor))]
      main_intrinsics = dc.wide_road.intrinsics if main_wide_camera else dc.narrow_road.intrinsics
      model_transform_main = get_warp_matrix(device_from_calib_euler, main_intrinsics, False).astype(np.float32)
      has_wide_camera = use_extra_client or main_wide_camera
      extra_intrinsics = dc.wide_road.intrinsics if has_wide_camera else dc.narrow_road.intrinsics
      model_transform_extra = get_warp_matrix(device_from_calib_euler, extra_intrinsics, True).astype(np.float32)
      extrinsics_calibration_seen = True

    traffic_convention = np.zeros(2)
    traffic_convention[int(is_rhd)] = 1

    vec_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
    if desire >= 0 and desire < ModelConstants.DESIRE_LEN:
      vec_desire[desire] = 1

    # tracked dropped frames
    vipc_dropped_frames = max(0, meta_main.frame_id - last_vipc_frame_id - 1)
    frames_dropped = frame_dropped_filter.update(min(vipc_dropped_frames, 10))
    if run_count < 10: # let frame drops warm up
      frame_dropped_filter.x = 0.
      frames_dropped = 0.
    run_count = run_count + 1

    frame_drop_ratio = frames_dropped / (1 + frames_dropped)

    bufs = {name: buf_extra if 'big' in name else buf_main for name in model.vision_input_names}
    transforms = {name: model_transform_extra if 'big' in name else model_transform_main for name in model.vision_input_names}
    frame_delay = DT_MDL # compensate for time passed since the frame was captured: current_time - timestamp_eof is 50ms on average
    action_delay = DT_MDL / 2 # middle of the interval between model output (current state) and next frame (expected state)
    lat_action_t = lat_delay + frame_delay + action_delay
    long_action_t = long_delay + frame_delay + action_delay
    inputs: dict[str, np.ndarray] = {
      'desire_pulse': vec_desire,
      'traffic_convention': traffic_convention,
      'action_t': np.array([lat_action_t, long_action_t], dtype=np.float32),
    }

    mt1 = time.perf_counter()
    try:
      send_chestnut = (chestnut_state is not None and
                       run_count % round(ModelConstants.MODEL_RUN_FREQ / SERVICE_LIST['chestnutGpuState'].frequency) == 0)
      model_output = model.run(bufs, transforms, inputs, chestnut_state.send if send_chestnut else None)
    except Exception:
      if not params.get_bool("ChestnutActive"):
        raise
      # fallback to small model
      cloudlog.exception("big model failed, fall back to small")
      params.put_bool("ChestnutActive", False)
      model = small_model
      if chestnut_state is not None:
        chestnut_state.big = False
      run_count = 0
      model_output = None
    mt2 = time.perf_counter()
    model_execution_time = mt2 - mt1

    if model_output is not None:
      modelv2_send = messaging.new_message('modelV2')
      drivingdata_send = messaging.new_message('drivingModelData')
      posenet_send = messaging.new_message('cameraOdometry')

      blinkers_active = sm['carState'].leftBlinker or sm['carState'].rightBlinker
      # Read the on-road HUD selector for every model frame so OFF -> ON and
      # ON -> OFF take effect immediately, not on the next one-second poll.
      lane_policy_enabled = get_lane_policy_enabled(params)
      delay_msg = sm['lateralDelay']
      now = time.monotonic()
      approach_time = lane_approach_timing.update(
        delay_msg.lateralDelay, delay_msg.status == log.LateralDelay.Status.estimated,
        sm.seen['lateralDelay'] and sm.valid['lateralDelay'] and sm.alive['lateralDelay'],
        now - sm.logMonoTime['lateralDelay'] / 1e9)
      action = get_action_from_model(model_output, prev_action, lat_action_t, long_action_t, v_ego,
                                     blinkers_active, lane_policy_enabled, approach_time)
      if now - last_lane_timing_log >= 5.0:
        source = 'estimated' if lane_approach_timing.using_estimate else 'baseline'
        cloudlog.info(f"lp-final-v3 timing: source={source} delay={delay_msg.lateralDelay:.4f} approach={approach_time:.4f}")
        last_lane_timing_log = now
      lane_policy_active = lane_policy_enabled and _lane_lock_full_active
      # Retain the existing UI parameter as a status bit: READY while the
      # two-line timer arms, HOLD while a single line is being reconstructed.
      lane_policy_blending = (lane_policy_enabled and
                              (_lane_lock_one_line_hold or
                               (not lane_policy_active and _lane_lock_arm_time > 0.0)))
      if (lane_policy_active != last_published_lane_policy_active or
          lane_policy_blending != last_published_lane_policy_blending):
        params.put_bool("LanePolicyActive", lane_policy_active)
        params.put_bool("LanePolicyBlending", lane_policy_blending)
        last_published_lane_policy_active = lane_policy_active
        last_published_lane_policy_blending = lane_policy_blending
      prev_action = action
      fill_model_msg(modelv2_send, model_output, action,
                     publish_state, meta_main.frame_id, meta_extra.frame_id, frame_id,
                     frame_drop_ratio, meta_main.timestamp_eof, model_execution_time, extrinsics_calibration_seen)
      modelv2_send.modelV2.big = model.chestnut

      desire_state = modelv2_send.modelV2.meta.desireState
      l_lane_change_prob = desire_state[log.Desire.laneChangeLeft]
      r_lane_change_prob = desire_state[log.Desire.laneChangeRight]
      lane_change_prob = l_lane_change_prob + r_lane_change_prob
      DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob)
      modelv2_send.modelV2.meta.laneChangeState = DH.lane_change_state
      modelv2_send.modelV2.meta.laneChangeDirection = DH.lane_change_direction

      fill_driving_model_data(drivingdata_send, modelv2_send)
      fill_pose_msg(posenet_send, model_output, meta_main.frame_id, vipc_dropped_frames, meta_main.timestamp_eof, extrinsics_calibration_seen)
      pm.send('modelV2', modelv2_send)
      pm.send('drivingModelData', drivingdata_send)
      pm.send('cameraOdometry', posenet_send)
    last_vipc_frame_id = meta_main.frame_id

if __name__ == "__main__":
  try:
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--demo', action='store_true', help='A boolean for demo mode.')
    args = parser.parse_args()
    main(demo=args.demo)
  except KeyboardInterrupt:
    cloudlog.warning("got SIGINT")
