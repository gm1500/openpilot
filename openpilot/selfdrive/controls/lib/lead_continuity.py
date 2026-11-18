"""Conservative motion evidence for one continuously associated vision target.

This is not a vehicle identity detector. It qualifies a short observed trajectory
without using model velocity, and never combines histories from different tracks.
"""

from collections import deque
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class _Sample:
  time: float
  distance: float
  lateral: float
  ego: float
  yaw: float
  travel: float
  lateral_motion: float


class LeadContinuity:
  WINDOW = 1.2
  MIN_HISTORY = 0.8
  MIN_SAMPLES = 12
  MAX_LATERAL_MOTION = 0.35
  MAX_RANGE_RESIDUAL = 4.0
  MAX_SPEED_ERROR = 1.5

  def __init__(self):
    self.history: deque[_Sample] = deque()
    self.continuous = False
    self.speed_upper: float | None = None
    self.speed = 0.0
    self.fitted = False
    self.reason = 'acquiring'

  def reset(self, reason='acquiring'):
    self.__init__()
    self.reason = reason

  def observe(self, observation, timestamp, ego, yaw_rate, single_target):
    self.continuous, self.speed_upper = False, None
    if not single_target:
      self.reset('distinct_targets')
      return
    if (yaw_rate is None or not all(math.isfinite(v) for v in (timestamp, ego, yaw_rate))
        or not observation.valid() or observation.probability < 0.9 or observation.std > 20.0
        or not 15.0 < ego < 50.0 or abs(yaw_rate) > 0.1):
      self.reset('uncertain_motion')
      return

    travel = lateral_motion = 0.0
    if self.history:
      old = self.history[-1]
      dt = timestamp - old.time
      if not 0.01 <= dt <= 0.1:
        self.reset('observation_gap')
      else:
        yaw = 0.5 * (old.yaw + yaw_rate)
        step = (0.5 * (old.ego + ego) + yaw * 0.5 * (old.lateral + observation.lateral)) * dt
        lateral_step = observation.lateral - old.lateral - yaw * 0.5 * (old.distance + observation.distance) * dt
        # Once fitted, reject discontinuities against measured motion, never
        # against the possibly optimistic model speed. Existing tracker gates
        # still own identity changes and the Kalman state.
        prediction = old.distance + self.speed * dt - step
        if abs(lateral_step) > 0.3 or (self.fitted and abs(observation.distance - prediction) > 5.0):
          self.reset('motion_jump')
        else:
          travel = old.travel + step
          lateral_motion = old.lateral_motion + lateral_step
    self.history.append(_Sample(timestamp, observation.distance, observation.lateral, ego, yaw_rate, travel, lateral_motion))
    while timestamp - self.history[0].time > self.WINDOW:
      self.history.popleft()
    if len(self.history) < self.MIN_SAMPLES or timestamp - self.history[0].time < self.MIN_HISTORY:
      self.reason = 'acquiring'
      return

    times = np.array([s.time - timestamp for s in self.history])
    positions = np.array([s.distance + s.travel for s in self.history])
    times -= times.mean()
    positions -= positions.mean()
    denominator = float(times @ times)
    self.speed = float(times @ positions) / denominator
    self.fitted = True
    residual = positions - self.speed * times
    # Camera samples are correlated. Keep a speed margin even on an exactly
    # straight synthetic trace; two agreeing slots are one observation.
    error = math.sqrt(max(float(residual @ residual) / (len(times) - 2), 0.25**2) / denominator)
    lateral = [s.lateral_motion for s in self.history]
    if max(lateral) - min(lateral) > self.MAX_LATERAL_MOTION:
      self.reason = 'lateral_motion'
    elif error > self.MAX_SPEED_ERROR or max(abs(residual)) > self.MAX_RANGE_RESIDUAL or not 0.0 <= self.speed <= 70.0:
      self.reason = 'range_noise'
    else:
      self.continuous = True
      self.speed_upper = self.speed + max(0.75, 2.0 * error)
      self.reason = 'continuous'
