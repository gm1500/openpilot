"""Experimental vision-distance lead speed for established highway tracks.

Estimate [gap, absolute lead speed]. Wheel speed supplies ego displacement;
only measured distance updates the Kalman state after initialization. Keep
raw distance and model acceleration, and return the original lead on fallback.
"""
import math

import numpy as np


class VisionLeadKalman:
  # Continuous white acceleration spectral density, (m/s^2)^2 * s.
  ACCEL_NOISE = 0.1
  MIN_SPEED = 15.0
  MIN_DISTANCE = 10.0
  MIN_PROBABILITY = 0.95
  WARMUP_TIME = 2.0
  BLEND_TIME = 1.0
  MAX_CORRECTION = 1.5  # m/s: bound disagreement with the original estimate

  def __init__(self):
    self.reset()

  def reset(self, reason: str = 'reset') -> None:
    self.x: np.ndarray | None = None
    self.P = np.zeros((2, 2))
    self.time = 0.0
    self.ego = 0.0
    self.y = 0.0
    self.distance = 0.0
    self.model_speed = 0.0
    self.age = 0.0
    self.active = False
    self.reason = reason
    self.correction = 0.0

  def update(self, lead: dict, timestamp: float, v_ego: float, probability: float, distance_std: float) -> dict:
    self.active = False
    self.correction = 0.0
    if not lead.get('present', False) or lead.get('radar', False):
      self.reset('radar' if lead.get('radar', False) else 'no lead')
      return lead

    distance, y, model_speed, acceleration = (lead[k] for k in ('dRel', 'yRel', 'vLead', 'aLeadK'))
    if not all(math.isfinite(v) for v in (timestamp, v_ego, probability, distance_std, distance, y, model_speed, acceleration)):
      self.reset('invalid')
      return lead
    if not self.MIN_PROBABILITY <= probability <= 1.0 or not 0.0 < distance_std <= 8.0:
      self.reset('confidence')
      return lead
    if not self.MIN_SPEED < v_ego < 60.0 or not self.MIN_DISTANCE < distance < 150.0:
      self.reset('range')
      return lead
    if acceleration < -0.2 or model_speed - v_ego < -2.5:
      self.reset('braking')
      return lead

    variance = max(distance_std, 1.0)**2
    if self.x is None:
      self.x = np.array([distance, model_speed], dtype=float)
      self.P = np.diag([variance, 9.0])
      self.time, self.ego, self.y, self.distance, self.model_speed = timestamp, v_ego, y, distance, model_speed
      self.reason = 'warmup'
      return lead

    dt = timestamp - self.time
    if not 0.01 <= dt <= 0.2:
      self.reset('time')
      return lead
    if abs(y - self.y) > 0.75 or abs(distance - self.distance - (self.x[1] - self.ego)*dt) > 5.0 or abs(model_speed - self.model_speed) > 3.0:
      self.reset('lead change')
      return lead

    F = np.array([[1.0, dt], [0.0, 1.0]])
    Q = self.ACCEL_NOISE * np.array([[dt**3/3, dt**2/2], [dt**2/2, dt]])
    self.x = F @ self.x
    self.x[0] -= 0.5 * (v_ego + self.ego) * dt
    self.P = F @ self.P @ F.T + Q
    innovation = distance - self.x[0]
    innovation_var = self.P[0, 0] + variance
    if innovation**2 > 16.0 * innovation_var:
      self.reset('innovation')
      return lead

    K = self.P[:, 0] / innovation_var
    self.x += K * innovation
    # Joseph covariance update remains positive semidefinite under roundoff.
    A = np.eye(2) - np.outer(K, [1.0, 0.0])
    self.P = A @ self.P @ A.T + np.outer(K, K) * variance
    self.P = (self.P + self.P.T) * 0.5
    self.time, self.ego, self.y, self.distance, self.model_speed = timestamp, v_ego, y, distance, model_speed
    self.age += dt
    if not np.all(np.isfinite(self.x)) or not np.all(np.isfinite(self.P)) or not 0.0 <= self.x[1] <= 70.0:
      self.reset('state')
      return lead

    weight = min(1.0, max(0.0, (self.age - self.WARMUP_TIME) / self.BLEND_TIME))
    weight *= min(1.0, (v_ego - self.MIN_SPEED) / 5.0, (distance - self.MIN_DISTANCE) / 10.0)
    if weight == 0.0 or self.P[1, 1] > 4.0:
      self.reason = 'warmup'
      return lead
    self.correction = float(np.clip(self.x[1] - model_speed, -self.MAX_CORRECTION, self.MAX_CORRECTION)) * weight
    speed = model_speed + self.correction
    self.active, self.reason = True, 'active'
    return dict(lead, vRel=float(speed - v_ego), vLead=float(speed), vLeadK=float(speed))
