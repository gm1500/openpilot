"""POC: adapt retained range motion to confirmed deceleration.

Future and direct-model estimates remain correlated priors. Existing published
lead braking has priority; extra range-based slowing needs model corroboration.
"""

import math
import numpy as np

from openpilot.selfdrive.controls.lib.lead_combined_motion import LeadCombinedMotion, RetainedRangeFutureFusion
from openpilot.selfdrive.controls.lib.vision_lead_tracker import _DistanceTrack


class ResponsiveDistanceTrack(_DistanceTrack):
  maneuver_noise = 0.0

  def acceleration_noise(self, distance, ego):
    return max(super().acceleration_noise(distance, ego), self.maneuver_noise)


class ResponsiveRangeFutureFusion(RetainedRangeFutureFusion):
  TRACK_CLASS = ResponsiveDistanceTrack

  def __init__(self, use_future=True, information_weight=.9, adaptive_noise=True):
    self.adaptive_noise = adaptive_noise
    super().__init__(use_future, information_weight)

  def reset(self, reason='reset'):
    super().reset(reason)
    self.evidence_time = 0.0
    self.last_fit_time = None
    self.fast_response = False
    self.range_acceleration = None
    self.acceleration_error = None

  def _deceleration_fit(self):
    history = self.history
    if len(history) < 24 or history[-1].time - history[0].time < 1.2:
      return None
    t = np.array([s.time - history[-1].time for s in history])
    x = np.array([s.distance + s.travel - history[-1].travel for s in history])
    A = np.column_stack((np.ones(len(t)), t, .5 * t**2))
    original = 1. / np.maximum([s.std for s in history], 2.)**2
    original = np.maximum(original, original.max() / 4.)
    weights = original.copy()
    for _ in range(3):
      coefficients = np.linalg.lstsq(A * np.sqrt(weights[:, None]), x * np.sqrt(weights), rcond=None)[0]
      residual = x - A @ coefficients
      scatter = max(.5, 1.4826 * float(np.median(abs(residual - np.median(residual)))))
      weights = original * np.minimum(1., 2. * scatter / np.maximum(abs(residual), 1e-6))
    normalized = weights / weights.sum()
    effective_n = min(float(weights.sum()**2 / (weights @ weights)), 4. * (t[-1] - t[0]))
    variance = max(.25, float(np.average(residual**2, weights=weights)))
    covariance = np.linalg.inv(A.T @ (normalized[:, None] * A)) * variance / max(effective_n - 3., 1.)
    speed, acceleration = coefficients[1:]
    speed_error, accel_error = np.sqrt(np.diag(covariance)[1:])
    if not all(math.isfinite(v) for v in (speed, acceleration, speed_error, accel_error)):
      return None
    return float(speed), float(acceleration), float(speed_error), float(accel_error)

  def _fit(self):
    now = self.history[-1].time
    dt = now - self.last_fit_time if self.last_fit_time is not None else 0.
    self.last_fit_time = now
    estimate = self._deceleration_fit()
    supported = False
    if estimate is not None and self.kalman is not None:
      speed, acceleration, speed_error, accel_error = estimate
      self.range_acceleration, self.acceleration_error = acceleration, accel_error
      supported = (0. <= speed <= 70. and -5. <= acceleration < -.5 and
                   acceleration + 2. * accel_error < 0. and
                   speed + 2. * speed_error < self.kalman.x[1] - .35)
    self.evidence_time = self.evidence_time + dt if supported and 0. < dt <= .1 + 1e-6 else 0.
    self.fast_response = self.adaptive_noise and self.evidence_time >= .15 - 1e-6
    if self.kalman is not None:
      # Increase process uncertainty when a sustained curved range history
      # rejects constant velocity. Keep state/history; do not assimilate the
      # same range observations a second time as an independent measurement.
      self.kalman.maneuver_noise = float(np.clip(abs(estimate[1]), 1., 4.)**2) if self.fast_response else 0.
    fit = super()._fit()
    if self.fast_response:
      # Optimistic model/future velocity must not undo observed deceleration.
      self.output_speed = min(self.output_speed, float(self.kalman.x[1]))
    return fit


class LeadResponsiveMotion(LeadCombinedMotion):
  CORRECTION_RATE = 2.5  # m/s per second; no old 3 m/s total correction cap

  def __init__(self, use_future=True, use_buffer=True, information_weight=.9, adaptive_noise=True):
    super().__init__(use_future, use_buffer, information_weight)
    self.motion = ResponsiveRangeFutureFusion(use_future, information_weight, adaptive_noise)

  def reset(self):
    super().reset()
    self.extra_slowing = 0.
    self.last_time = None

  def update(self, models, observations, raw_leads, baseline, timestamp, ego, yaw_rate, valid=True, advance=True):
    previous_time = self.last_time
    output = super().update(models, observations, raw_leads, baseline, timestamp, ego, yaw_rate, valid, advance)
    self.last_time = timestamp
    dt = timestamp - previous_time if previous_time is not None else 0.
    if self.reason != 'blend' or not 0. < dt <= .1 + 1e-6:
      self.extra_slowing = 0.
      self.future_weight, self.delta = 0., 0.
      return baseline
    accepted = [i for i, lead in enumerate(output) if lead is not baseline[i]]
    # Depth drift can imitate closing. The current model is a corroborating
    # veto, never a source of extra confidence in the range covariance.
    if not accepted or any(ego - raw_leads[i]['vLead'] < .5 for i in accepted):
      self.extra_slowing = 0.
      self.reason = 'uncorroborated_closing'
      self.future_weight, self.delta = 0., 0.
      return baseline
    # Let the qualified retained-motion estimate catch a large closing-speed
    # difference without flooring it at the range's upper confidence bound.
    # Keep the range uncertainty/association checks, model closing veto and
    # gradual entry. Correlated range/model bias can still add unwanted slowing.
    desired = max(0., min(baseline[i]['vLead'] - output[i]['vLead'] for i in accepted))
    self.extra_slowing = min(desired, self.extra_slowing + self.CORRECTION_RATE * dt)
    if self.extra_slowing == 0.:
      self.future_weight = 0.
    # Never replace a lower accepted lead speed with the slower-adapting
    # private estimate. In particular, existing braking cues remain immediate.
    for i in accepted:
      speed = max(0., baseline[i]['vLead'] - self.extra_slowing)
      output[i] = dict(baseline[i], vLead=speed, vLeadK=speed, vRel=speed - ego)
    self.delta = -self.extra_slowing
    return output
