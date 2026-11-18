"""Experimental single-lead motion evidence independent of range-track resets.

Current observations own presence and distance. A short robust range history and
an uncertainty-weighted future-position prior may only lower an optimistic lead
speed. No Kalman state is copied across targets or updated with correlated model
predictions. The base correction policy is an offline comparison; the responsive
POC subclass reuses its history and qualification rules.
"""

from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from openpilot.selfdrive.controls.lib.lead_future_motion import FutureMotion
from openpilot.selfdrive.controls.lib.vision_lead_tracker import VisionLeadTracker


@dataclass(frozen=True)
class _Sample:
  time: float
  distance: float
  lateral: float
  std: float
  probability: float
  ego: float
  yaw: float
  travel: float
  lateral_motion: float


@dataclass(frozen=True)
class MotionFit:
  speed: float
  error: float
  distance: float
  scatter: float


class LeadMotionFusion:
  WINDOW = 2.4
  MIN_HISTORY = 1.2
  MIN_SAMPLES = 18
  HOLD_TIME = 0.20
  CONFIRM_TIME = 0.20
  MAX_CORRECTION = 3.0
  CORRECTION_RATE = 2.0
  MAX_FUTURE_WEIGHT = 0.25

  def __init__(self, use_future=True, require_model_closing=True):
    self.use_future = use_future
    self.require_model_closing = require_model_closing
    self.reset()

  def reset(self, reason='reset'):
    self.history = deque()
    self.time = None
    self.fit = None
    self.correction = self.confirmed = 0.0
    self.future_weight = self.future_speed = 0.0
    self.speed_upper = 0.0
    self.reason = reason

  def _revoke(self, reason):
    self.correction = self.confirmed = 0.0
    self.reason = reason

  def _fit(self):
    h = self.history
    if len(h) < self.MIN_SAMPLES or h[-1].time - h[0].time < self.MIN_HISTORY:
      return None
    t = np.array([s.time - h[-1].time for s in h])
    x = np.array([s.distance + s.travel - h[-1].travel for s in h])
    std = np.array([s.std for s in h])
    weights = 1.0 / np.maximum(std, 2.0)**2
    weights = np.maximum(weights, weights.max() / 4.0)
    original = weights.copy()
    for _ in range(3):
      mt, mx = np.average(t, weights=weights), np.average(x, weights=weights)
      centred = t - mt
      denominator = float(np.sum(weights * centred**2))
      speed = float(np.sum(weights * centred * (x - mx)) / denominator)
      residual = x - mx - speed * centred
      scatter = max(0.5, 1.4826 * float(np.median(abs(residual - np.median(residual)))))
      weights = original * np.minimum(1.0, 2.0 * scatter / np.maximum(abs(residual), 1e-6))
    variance = max(0.5**2, float(np.average(residual**2, weights=weights)))
    # A stream of camera frames is correlated: never claim 20 independent
    # position measurements per second, or extra confidence from duplicate slots.
    effective_n = min(float(weights.sum()**2 / np.sum(weights**2)), 4.0 * (t[-1] - t[0]))
    error = math.sqrt(variance / (max(effective_n - 2.0, 1.0) * float(np.average(centred**2, weights=weights))))
    if not all(math.isfinite(v) for v in (speed, error, scatter)) or not 0.0 <= speed <= 70.0:
      return None
    return MotionFit(speed, max(error, 0.5), float(mx - speed * mt), scatter)

  def update(self, models, observations, raw_leads, leads, timestamp, ego, yaw_rate, valid=True):
    self.future_weight = self.future_speed = 0.0
    if (not valid or not all(math.isfinite(v) for v in (timestamp, ego)) or yaw_rate is None
        or not math.isfinite(yaw_rate) or abs(yaw_rate) > 0.1 or not 15.0 < ego < 50.0
        or not len(models) == len(observations) == len(raw_leads) == len(leads) == 2):
      self.reset('unsupported')
      return leads
    dt = timestamp - self.time if self.time is not None else 0.0
    if self.time is not None and not 0.01 <= dt <= 0.1 + 1e-6:
      self.reset('observation_gap')
      dt = 0.0
    self.time = timestamp
    groups = VisionLeadTracker._groups(observations)
    if len(groups) != 1:
      self.reset('distinct_or_missing')
      return leads
    indices, obs = groups[0]
    accepted = [i for i in indices if raw_leads[i].get('present', False) and leads[i].get('present', False)]
    if not accepted:
      self.reset('not_accepted')
      return leads
    if any(raw_leads[i].get('radar', False) or leads[i].get('radar', False)
           or not all(math.isfinite(lead.get(k, math.nan)) for lead in (raw_leads[i], leads[i])
                      for k in ('dRel', 'vLead', 'aLeadK')) for i in accepted):
      self.reset('invalid_lead')
      return leads
    if not 30.0 < obs.distance < 150.0 or obs.probability < 0.7 or obs.std > 20.0:
      self.reset('uncertain_observation')
      return leads
    # Braking, close/stopped leads and published presence retain their owners.
    if any(lead['aLeadK'] < -0.5 or lead['vLead'] < 2.0 or lead['dRel'] < 4.0 * max(ego - lead['vLead'], 0.0)
           for i in accepted for lead in (raw_leads[i], leads[i])):
      self.reset('braking_or_short_gap')
      return leads

    travel = lateral_motion = 0.0
    if self.history:
      old = self.history[-1]
      elapsed = timestamp - old.time
      yaw = 0.5 * (old.yaw + yaw_rate)
      step = (0.5 * (old.ego + ego) + yaw * 0.5 * (old.lateral + obs.lateral)) * elapsed
      lateral_step = obs.lateral - old.lateral - yaw * 0.5 * (old.distance + obs.distance) * elapsed
      speed = self.fit.speed if self.fit else min(raw_leads[i]['vLead'] for i in accepted)
      distance = self.fit.distance if self.fit else old.distance
      residual = obs.distance - (distance + speed * elapsed - step)
      # The robust history predicts from all accepted samples, rather than
      # comparing two individually noisy ranges. An apparent jump suppresses
      # output immediately. Only private history gets a short opportunity to recover.
      gate = max(3.0, min(10.0, 2.5 * self.fit.scatter)) if self.fit else max(5.0, min(10.0, obs.std))
      if abs(lateral_step) > 0.35 or elapsed > self.HOLD_TIME + 1e-6:
        self.reset('target_change')
        self.time = timestamp
      elif abs(residual) > gate:
        self._revoke('range_quarantine')
        return leads
      else:
        travel = old.travel + step
        lateral_motion = old.lateral_motion + lateral_step
    self.history.append(_Sample(timestamp, obs.distance, obs.lateral, obs.std, obs.probability,
                                ego, yaw_rate, travel, lateral_motion))
    while self.history[-1].time - self.history[0].time > self.WINDOW:
      self.history.popleft()
    self.fit = self._fit()
    if self.fit is None:
      self._revoke('acquiring')
      return leads
    # Correlated depth drift can look like real closing even after robust
    # regression. Require a small closing indication from every accepted
    # current hypothesis too. This is a veto, not an independent measurement
    # or a requirement that model and observed velocities agree numerically.
    if self.require_model_closing and any(ego - raw_leads[i]['vLead'] < 0.5 for i in accepted):
      self._revoke('model_not_closing')
      return leads
    if obs.probability < 0.9 or max(s.lateral_motion for s in self.history) - min(s.lateral_motion for s in self.history) > 0.45:
      self._revoke('uncertain_motion')
      return leads
    if self.fit.error > 3.0:
      self._revoke('range_uncertainty')
      return leads

    speed, error = self.fit.speed, self.fit.error
    if self.use_future:
      motions = [FutureMotion.from_model(models[i], max_speed_std=24.0) for i in accepted]
      if (all(m is not None for m in motions) and max(m.speed for m in motions) - min(m.speed for m in motions) < 2.0):
        future = sum(m.speed for m in motions) / len(motions)
        future_error = max(m.speed_std for m in motions)
        weight = min(self.MAX_FUTURE_WEIGHT, error**2 / (error**2 + future_error**2))
        # A correlated forecast is a bounded prior, not an independent sensor.
        # Use the conservative uncertainty bound; fusion must not shrink it.
        speed = (1.0 - weight) * speed + weight * future
        error = max(error, (1.0 - weight) * error + weight * future_error)
        self.future_weight, self.future_speed = weight, future
    self.speed_upper = speed + max(1.0, 2.0 * error)
    baseline = min(leads[i]['vLead'] for i in accepted)
    closing = ego - self.speed_upper
    if closing < 2.0 or self.speed_upper >= baseline - 0.35:
      self._revoke('no_supported_closing')
      return leads
    self.confirmed += dt
    if self.confirmed < self.CONFIRM_TIME - 1e-6:
      self.reason = 'confirming'
      return leads
    desired = min(self.MAX_CORRECTION, baseline - self.speed_upper)
    self.correction = min(desired, self.correction + self.CORRECTION_RATE * dt)
    self.reason = 'closing'
    output = list(leads)
    for i in accepted:
      speed = max(0.0, leads[i]['vLead'] - self.correction)
      output[i] = dict(leads[i], vLead=speed, vLeadK=speed, vRel=speed - ego)
    return output
