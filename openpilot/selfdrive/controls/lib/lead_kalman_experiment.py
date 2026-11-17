"""Shared retained-distance Kalman core and optional offline fusion comparison.

Reuses the production distance/velocity Kalman equations, but not its live state.
Future velocity enters that private state via covariance intersection (unknown
correlation), at most twice per second. The responsive runtime subclass disables
this optional prior update and uses its own separately filtered forecast.
"""

import math

import numpy as np

from openpilot.selfdrive.controls.lib.lead_future_motion import FutureMotion
from openpilot.selfdrive.controls.lib.lead_motion_fusion import LeadMotionFusion, MotionFit
from openpilot.selfdrive.controls.lib.vision_lead_tracker import VisionLeadObservation, _DistanceTrack


class KalmanLeadMotionExperiment(LeadMotionFusion):
  TRACK_CLASS = _DistanceTrack

  def __init__(self, use_future=True):
    self.kalman_use_future = use_future
    self.models = []
    super().__init__(use_future=False, require_model_closing=False)

  def reset(self, reason='reset'):
    super().reset(reason)
    self.kalman = None
    self.future_time = None

  def update(self, models, observations, raw_leads, leads, timestamp, ego, yaw_rate, valid=True):
    self.models = models
    return super().update(models, observations, raw_leads, leads, timestamp, ego, yaw_rate, valid)

  def _fit(self):
    trend = super()._fit()
    for s in self.history:
      if self.kalman is not None and s.time <= self.kalman.time:
        continue
      obs = VisionLeadObservation(s.distance, s.lateral, s.std, s.probability)
      if self.kalman is None:
        self.kalman = self.TRACK_CLASS(obs, s.time, s.ego)
      elif not self.kalman.update(obs, s.time, s.ego):
        self.kalman = self.TRACK_CLASS(obs, s.time, s.ego)
    if trend is None:
      return None

    now = self.history[-1].time
    if self.kalman_use_future and (self.future_time is None or now - self.future_time >= 0.5):
      motions = [FutureMotion.from_model(m, max_speed_std=24.0) for m in self.models if m.prob >= 0.9]
      if motions and all(m is not None for m in motions) and max(m.speed for m in motions) - min(m.speed for m in motions) < 2.0:
        future_speed = sum(m.speed for m in motions) / len(motions)
        variance = max(1.0, max(m.speed_std for m in motions)**2)
        weight = 0.1
        information = np.linalg.inv(self.kalman.P)
        prior_information = np.diag([0.0, 1.0 / variance])
        covariance = np.linalg.inv((1.0 - weight) * information + weight * prior_information)
        self.kalman.x = covariance @ ((1.0 - weight) * information @ self.kalman.x
                                     + weight * np.array([0.0, future_speed / variance]))
        self.kalman.P = covariance
        self.future_time = now
        self.future_weight, self.future_speed = weight, future_speed

    # The short observed trend is still an uncertainty floor: a narrow Kalman
    # covariance must not declare correlated depth drift to be certain motion.
    error = max(0.5, trend.error, math.sqrt(self.kalman.P[1, 1]))
    return MotionFit(float(self.kalman.x[1]), error, trend.distance, trend.scatter)
