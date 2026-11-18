"""Full-speed fusion base for retained range, filtered future, and model buffer.

The unrestricted class is for offline comparisons. The on-device POC uses the
guarded LeadResponsiveMotion subclass. The two Kalman streams share camera
errors, so their outputs use covariance intersection, not an independent update.
"""

import math
import numpy as np

from openpilot.selfdrive.controls.lib.lead_distance_blend import LeadDistanceBlend
from openpilot.selfdrive.controls.lib.lead_future_motion import FutureMotion
from openpilot.selfdrive.controls.lib.lead_kalman_motion import KalmanLeadMotion


class FutureVelocityKalman:
  def __init__(self):
    self.reset()

  def reset(self):
    self.x = self.P = None
    self.time = self.measurement_time = self.started = None
    self.updates = 0
    self.variance_floor = math.inf

  def update(self, speed, std, timestamp):
    if not all(math.isfinite(v) for v in (speed, std, timestamp)) or not 0. <= speed <= 70. or not 0. < std <= 24.:
      self.reset()
      return False
    dt = timestamp - self.time if self.time is not None else 0.
    if self.time is not None and not .01 <= dt <= .2 + 1e-6:
      self.reset()
    variance = max(1., std**2)
    if self.x is None:
      self.x = np.array([speed, 0.])
      self.P = np.diag([variance, 4.])
      self.time = self.measurement_time = self.started = timestamp
      self.updates = 1
    else:
      F = np.array([[1., dt], [0., 1.]])
      Q = np.array([[dt**3 / 3., dt**2 / 2.], [dt**2 / 2., dt]])
      self.x = F @ self.x
      self.P = F @ self.P @ F.T + Q
      if timestamp - self.measurement_time >= .25 - 1e-6:
        innovation = speed - self.x[0]
        S = self.P[0, 0] + variance
        if innovation**2 > 9. * S:
          self.reset()
          return self.update(speed, std, timestamp)
        K = self.P[:, 0] / S
        self.x += K * innovation
        A = np.eye(2) - np.outer(K, [1., 0.])
        self.P = A @ self.P @ A.T + np.outer(K, K) * variance
        self.P = .5 * (self.P + self.P.T)
        self.measurement_time = timestamp
        self.updates += 1
      self.time = timestamp
    # Repeated camera forecasts are correlated: smoothing the mean must not
    # make raw forecast uncertainty disappear through frame-count averaging.
    self.variance_floor = variance
    if not np.isfinite(self.x).all() or not np.isfinite(self.P).all() or not 0. <= self.x[0] <= 70. or abs(self.x[1]) > 5.:
      self.reset()
      return False
    return timestamp - self.started >= .5 and self.updates >= 3

  @property
  def velocity_variance(self):
    return max(float(self.P[0, 0]), self.variance_floor)


class RetainedRangeFutureFusion(KalmanLeadMotion):
  def __init__(self, use_future=True, information_weight=.25):
    if not 0. <= information_weight < 1.:
      raise ValueError('future information weight must be in [0, 1)')
    self.enable_forecast = use_future
    self.information_weight = information_weight
    self.forecast = FutureVelocityKalman()
    super().__init__(use_future=False)

  def reset(self, reason='reset'):
    super().reset(reason)
    self.forecast.reset()
    self.output_speed = 0.
    self.fusion_gain = 0.
    self.fused_variance = None
    self.future_available = False

  def _fit(self):
    fit = super()._fit()
    self.fusion_gain = 0.
    self.future_available = False
    self.output_speed = float(self.kalman.x[1])
    self.fused_variance = float(self.kalman.P[1, 1])
    if not self.enable_forecast:
      return fit
    models = [m for m in self.models if m.prob >= .9]
    motions = [FutureMotion.from_model(m, max_speed_std=24.) for m in models]
    if not motions or any(m is None for m in motions) or max(m.speed for m in motions) - min(m.speed for m in motions) >= 2.:
      self.forecast.reset()
      return fit
    speed = sum(m.speed for m in motions) / len(motions)
    std = max(m.speed_std for m in motions)
    ready = self.forecast.update(speed, std, self.history[-1].time)
    if fit is None or not ready:
      return fit
    # Inflate the range covariance to its empirical trend-uncertainty floor.
    # Keep range association history independent of the forecast fusion result.
    covariance = self.kalman.P.copy()
    covariance[1, 1] = max(covariance[1, 1], fit.error**2)
    information = np.linalg.inv(covariance)
    variance = self.forecast.velocity_variance
    weight = self.information_weight
    fused = np.linalg.inv((1. - weight) * information + weight * np.diag([0., 1. / variance]))
    state = fused @ ((1. - weight) * information @ self.kalman.x
                     + weight * np.array([0., self.forecast.x[0] / variance]))
    if np.isfinite(state).all() and 0. <= state[1] <= 70.:
      self.output_speed = float(state[1])
      self.fused_variance = float(fused[1, 1])
      self.fusion_gain = float(fused[1, 1] * weight / variance)
      self.future_available = True
    return fit


class LeadCombinedMotion(LeadDistanceBlend):
  independent_motion = True

  def __init__(self, use_future=True, use_buffer=True, information_weight=.25):
    super().__init__('buffer_5' if use_buffer else 'buffer_0')
    self.motion = RetainedRangeFutureFusion(use_future, information_weight)

  def update(self, *args, **kwargs):
    output = super().update(*args, **kwargs)
    self.future_weight = self.motion.fusion_gain if self.reason == 'blend' else 0.
    return output
