import math
import numpy as np
from collections import deque

from openpilot.cereal import log
from opendbc.car.lateral import FRICTION_THRESHOLD, get_friction
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects the
# proportional gain is increased at low speeds by the PID controller.
# Additionally, there is friction in the steering wheel that needs
# to be overcome to move it at all, this is compensated for too.

KP = 0.8
KI = 0.15

INTERP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
KP_INTERP = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, KP]

LP_FILTER_CUTOFF_HZ = 1.2
JERK_LOOKAHEAD_SECONDS = 0.19
JERK_GAIN = 0.3
LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0

# Speed-dependent actuator-side lateral jerk limiting. The limit is applied in
# lateral-acceleration space after PID and before the vehicle torque mapping.
JERK_LIMIT_SPEEDS = [0.0, 10.0, 20.0, 30.0]
JERK_LIMITS = [10.0, 10.0, 2.5, 1.0]
MEASURED_JERK_FILTER_CUTOFF_HZ = 5.0
MEASURED_JERK_LIMIT = 10.0
MEASURED_JERK_DAMP_GAIN = 0.05
CURVATURE_RATE_JERK_GAIN = 6.0
MIN_CURVATURE_RATE_JERK_SCALE = 0.5

# Lane-center feedback from previewed curvature error. This is restricted to
# gentle turns so it shapes the planner request without influencing tight turns.
CENTER_PREVIEW_A = 4.5
CENTER_PREVIEW_B = 0.25
K_Y = 0.06
K_PSI = 0.025

# Outward corrections when measured curvature is tighter than requested.
K_APEX = 0.40
K_CUT = 0.60

VERSION = 3

class LatControlTorque(LatControl):
  def __init__(self, CP, CI, dt):
    super().__init__(CP, CI, dt)
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()
    self.pid = PIDController([INTERP_SPEEDS, KP_INTERP], KI, rate=1/self.dt)
    self.update_limits()
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.lat_accel_request_buffer_len = int(LAT_ACCEL_REQUEST_BUFFER_SECONDS / self.dt)
    self.lat_accel_request_buffer = deque([0.] * self.lat_accel_request_buffer_len , maxlen=self.lat_accel_request_buffer_len)
    self.lookahead_frames = int(JERK_LOOKAHEAD_SECONDS / self.dt)
    self.jerk_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt)
    self.measured_jerk_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * MEASURED_JERK_FILTER_CUTOFF_HZ), self.dt)
    self._last_output_lataccel = 0.0
    self._last_actual_lateral_accel = None
    self._last_feedback_error = 0.0
    self._last_desired_curvature = None

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction
    self.update_limits()

  def update_limits(self):
    self.pid.set_limits(self.lateral_accel_from_torque(self.steer_max, self.torque_params),
                        self.lateral_accel_from_torque(-self.steer_max, self.torque_params))
    if hasattr(self, '_last_output_lataccel'):
      self._last_output_lataccel = float(np.clip(self._last_output_lataccel, self.pid.neg_limit, self.pid.pos_limit))

  @staticmethod
  def _deadzone(value, deadzone):
    if abs(value) <= deadzone:
      return 0.0
    return value - math.copysign(deadzone, value)

  def _apply_jerk_limit(self, target_lataccel, v_ego, curvature_rate_scale):
    max_jerk = float(np.interp(v_ego, JERK_LIMIT_SPEEDS, JERK_LIMITS))
    max_delta = max_jerk * curvature_rate_scale * self.dt
    self._last_output_lataccel = float(np.clip(self._last_output_lataccel, self.pid.neg_limit, self.pid.pos_limit))
    output_lataccel = float(np.clip(target_lataccel,
                                    max(self._last_output_lataccel - max_delta, self.pid.neg_limit),
                                    min(self._last_output_lataccel + max_delta, self.pid.pos_limit)))
    self._last_output_lataccel = output_lataccel
    return output_lataccel

  @staticmethod
  def _shape_desired_lateral_accel(desired_curvature, measured_curvature, v_ego):
    base_desired_lateral_accel = desired_curvature * v_ego ** 2
    desired_lateral_accel = base_desired_lateral_accel
    desired_lateral_accel_magnitude = abs(base_desired_lateral_accel)

    # Preview curvature error as heading and cross-track error. Fade this out
    # for tight turns, where the model request should remain authoritative.
    preview_distance = float(np.clip(CENTER_PREVIEW_A + CENTER_PREVIEW_B * v_ego, 4.0, 20.0))
    curvature_error = desired_curvature - measured_curvature
    heading_error = curvature_error * preview_distance
    cross_track_error = 0.5 * curvature_error * preview_distance ** 2

    tight_curve = abs(desired_curvature) * v_ego > 0.25
    if desired_lateral_accel_magnitude < 2.5 and not tight_curve:
      speed_scale = np.clip((v_ego - 3.0) / 4.0, 0.0, 1.0)
      center_term = (K_Y * cross_track_error + K_PSI * v_ego * heading_error) * speed_scale
      max_center = 0.40 * (desired_lateral_accel_magnitude + 0.5)
      desired_lateral_accel += float(np.clip(center_term, -max_center, max_center))

    # CUT applies symmetrically whenever measured curvature magnitude exceeds
    # the request. Preserve the source behavior for opposite-sign curvature.
    inside_excess = max(0.0, abs(measured_curvature) - abs(desired_curvature))
    if inside_excess > 0.0:
      cut_guard = -K_CUT * v_ego ** 2 * inside_excess * np.sign(desired_curvature)
      max_cut_guard = 0.50 * (desired_lateral_accel_magnitude + 0.5)
      desired_lateral_accel += float(np.clip(cut_guard, -max_cut_guard, max_cut_guard))

    # Apex guard is directional and stacks with CUT only when both curvatures
    # have the same nonzero sign and measured curvature is tighter.
    same_sign = np.sign(desired_curvature) == np.sign(measured_curvature) and np.sign(desired_curvature) != 0.0
    cutting_inside = same_sign and abs(measured_curvature) > abs(desired_curvature)
    if cutting_inside:
      inside_delta = abs(measured_curvature) - abs(desired_curvature)
      apex_guard = -K_APEX * v_ego ** 2 * inside_delta * np.sign(desired_curvature)
      max_apex_guard = 0.40 * (desired_lateral_accel_magnitude + 0.5)
      desired_lateral_accel += float(np.clip(apex_guard, -max_apex_guard, max_apex_guard))

    return float(desired_lateral_accel)

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = VERSION
    measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
    measurement = measured_curvature * CS.vEgo ** 2
    raw_measured_lateral_jerk = 0.0 if self._last_actual_lateral_accel is None else \
      (measurement - self._last_actual_lateral_accel) / self.dt
    measured_lateral_jerk = self.measured_jerk_filter.update(
      float(np.clip(raw_measured_lateral_jerk, -MEASURED_JERK_LIMIT, MEASURED_JERK_LIMIT)))
    predicted_measurement = measurement + measured_lateral_jerk * self.dt
    future_desired_lateral_accel = self._shape_desired_lateral_accel(desired_curvature, measured_curvature, CS.vEgo)
    self.lat_accel_request_buffer.append(future_desired_lateral_accel)

    roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
    curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
    lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

    delay_frames = int(np.clip(lat_delay / self.dt + 1, 1, self.lat_accel_request_buffer_len))
    expected_lateral_accel = self.lat_accel_request_buffer[-delay_frames]
    setpoint = expected_lateral_accel
    lateral_accel_error = setpoint - measurement
    raw_feedback_error = setpoint - predicted_measurement
    error = self._deadzone(raw_feedback_error, 0.5 * lateral_accel_deadzone)

    lookahead_idx = int(np.clip(-delay_frames + self.lookahead_frames, -self.lat_accel_request_buffer_len+1, -2))
    raw_lateral_jerk = (self.lat_accel_request_buffer[lookahead_idx+1] - self.lat_accel_request_buffer[lookahead_idx-1]) / (2 * self.dt)
    desired_lateral_jerk = self.jerk_filter.update(raw_lateral_jerk)
    gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
    ff = gravity_adjusted_future_lateral_accel
    # latAccelOffset corrects roll compensation bias from device roll misalignment relative to car roll
    ff -= self.torque_params.latAccelOffset
    ff += get_friction(lateral_accel_error + JERK_GAIN * desired_lateral_jerk,
                       lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)
    ff -= MEASURED_JERK_DAMP_GAIN * measured_lateral_jerk

    if not active:
      output_torque = 0.0
      pid_log.active = False
      self._last_output_lataccel = 0.0
      self._last_feedback_error = 0.0
    else:
      # do error correction in lateral acceleration space, convert at end to handle non-linear torque responses correctly
      pid_log.error = float(error)

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
      if not freeze_integrator and self._last_feedback_error * raw_feedback_error < 0.0 and \
         self.pid.i * raw_feedback_error < 0.0 and abs(raw_feedback_error) < 2.0 * lateral_accel_deadzone:
        self.pid.i *= 0.9

      previous_integral = self.pid.i
      output_lataccel = self.pid.update(pid_log.error, speed=CS.vEgo, feedforward=ff, freeze_integrator=freeze_integrator)
      desired_curvature_rate = 0.0 if self._last_desired_curvature is None else \
        (desired_curvature - self._last_desired_curvature) / self.dt
      curvature_rate_scale = float(np.clip(1.0 / (1.0 + CURVATURE_RATE_JERK_GAIN * abs(desired_curvature_rate)),
                                           MIN_CURVATURE_RATE_JERK_SCALE, 1.0))
      raw_output_lataccel = output_lataccel
      output_lataccel = self._apply_jerk_limit(raw_output_lataccel, CS.vEgo, curvature_rate_scale)
      integral_delta = self.pid.i - previous_integral
      if not freeze_integrator and (raw_output_lataccel - output_lataccel) * integral_delta > 0.0:
        self.pid.i = previous_integral
      output_torque = self.torque_from_lateral_accel(output_lataccel, self.torque_params)
      self._last_feedback_error = raw_feedback_error

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque) # TODO: log lat accel?
      pid_log.actualLateralAccel = float(measurement)
      pid_log.desiredLateralAccel = float(setpoint)
      pid_log.desiredLateralJerk = float(desired_lateral_jerk)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    # Track plant/request histories while inactive to avoid derivative spikes on engage.
    self._last_actual_lateral_accel = measurement
    self._last_desired_curvature = desired_curvature

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
