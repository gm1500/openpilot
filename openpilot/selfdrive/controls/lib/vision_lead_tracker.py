"""Experimental distance-derived speed with private, pre-acceptance lead history.

Control acceptance and raw distance/acceleration remain owned by radard. Model
velocity never updates the Kalman state; it remains a fallback and brake guard.
"""

from dataclasses import dataclass
from itertools import product
import math

import numpy as np


@dataclass(frozen=True)
class VisionLeadObservation:
  distance: float
  lateral: float
  std: float
  probability: float

  def valid(self) -> bool:
    return (
      all(math.isfinite(v) for v in (self.distance, self.lateral, self.std, self.probability))
      and 0.2 <= self.probability <= 1.0
      and 0.0 < self.std <= 50.0
      and 2.0 < self.distance < 200.0
    )

  @property
  def variance(self) -> float:
    return max(self.std, 1.0) ** 2 / self.probability**2


class _DistanceTrack:
  ACCEL_NOISE = 0.1  # continuous white acceleration spectral density
  MIN_HISTORY = 1.5
  MAX_SPEED_VARIANCE = 2.25

  def __init__(self, observation: VisionLeadObservation, timestamp: float, ego: float):
    self.x = np.array([observation.distance, ego], dtype=float)
    self.P = np.diag([observation.variance, 1000.0])
    self.time, self.ego = timestamp, ego
    self.distance, self.lateral = observation.distance, observation.lateral
    self.age = 0.0
    self.dt = 0.0
    self.output: float | None = None
    self.mode = 'fallback'
    self.offset = 0.0
    self.transition = 1.0
    self.last_braking = -math.inf

  @property
  def ready(self) -> bool:
    return self.age >= self.MIN_HISTORY and self.P[1, 1] <= self.MAX_SPEED_VARIANCE and 0.0 <= self.x[1] <= 70.0

  def association_cost(self, observation: VisionLeadObservation, timestamp: float, ego: float) -> float:
    dt = timestamp - self.time
    # Compare successive observations; filtered distance can lag during braking.
    prediction = self.distance + (self.x[1] - 0.5 * (ego + self.ego)) * dt
    distance_error = abs(observation.distance - prediction)
    lateral_error = abs(observation.lateral - self.lateral)
    if not 0.01 <= dt <= 0.2 or distance_error > 5.0 or lateral_error > 0.75:
      return math.inf
    return distance_error / 5.0 + lateral_error / 0.75

  def update(self, observation: VisionLeadObservation, timestamp: float, ego: float) -> bool:
    dt = timestamp - self.time
    F = np.array([[1.0, dt], [0.0, 1.0]])
    Q = self.ACCEL_NOISE * np.array([[dt**3 / 3, dt**2 / 2], [dt**2 / 2, dt]])
    self.x = F @ self.x
    self.x[0] -= 0.5 * (ego + self.ego) * dt
    self.P = F @ self.P @ F.T + Q
    innovation = observation.distance - self.x[0]
    variance = self.P[0, 0] + observation.variance
    if innovation**2 > 16.0 * variance:
      return False
    K = self.P[:, 0] / variance
    self.x += K * innovation
    A = np.eye(2) - np.outer(K, [1.0, 0.0])
    self.P = A @ self.P @ A.T + np.outer(K, K) * observation.variance
    self.P = 0.5 * (self.P + self.P.T)
    if not np.all(np.isfinite(self.x)) or not np.all(np.isfinite(self.P)) or not -5.0 <= self.x[1] <= 70.0:
      return False
    self.time, self.ego, self.dt = timestamp, ego, dt
    self.distance, self.lateral = observation.distance, observation.lateral
    self.age += dt
    return True

  def speed(self, leads: list[dict], observation: VisionLeadObservation, ego: float) -> float:
    baseline = min(lead['vLead'] for lead in leads)
    use_distance = self.ready and observation.probability >= 0.5 and 15.0 < ego < 60.0 and 10.0 < observation.distance < 150.0
    target = float(self.x[1]) if use_distance else baseline
    mode = 'distance' if use_distance else 'fallback'
    closing = ego - baseline
    urgent = min(lead['aLeadK'] for lead in leads) < -0.5 or (closing > 0.0 and min(lead['dRel'] for lead in leads) < 5.0 * closing)
    if urgent:
      self.last_braking = self.time
    recovering = self.time - self.last_braking < 2.0 and target > baseline + 0.5
    if urgent or recovering:
      target = min(target, baseline)
      mode = 'braking'
      self.offset, self.transition = 0.0, 1.0
    elif self.output is not None and mode != self.mode:
      self.offset, self.transition = self.output - target, 0.0
    else:
      self.transition = min(1.0, self.transition + self.dt)
    self.mode = mode
    self.output = float(np.clip(target + self.offset * (1.0 - self.transition), 0.0, 70.0))
    return self.output


class VisionLeadTracker:
  def __init__(self):
    self.reset()

  def reset(self) -> None:
    self.tracks: list[_DistanceTrack] = []
    self.time: float | None = None
    self.slots: list[_DistanceTrack | None] = [None, None]
    self.history_slots: list[_DistanceTrack | None] = [None, None]

  @staticmethod
  def _groups(observations: list[VisionLeadObservation]) -> list[tuple[list[int], VisionLeadObservation]]:
    valid = [i for i, obs in enumerate(observations) if obs.valid()]
    if len(valid) == 2:
      a, b = observations
      if abs(a.distance - b.distance) <= 1.5 and abs(a.lateral - b.lateral) <= 0.35:
        weights = [obs.probability / max(obs.std, 1.0) ** 2 for obs in observations]
        total = sum(weights)
        # Two outputs of the same camera are correlated, not two measurements.
        merged = VisionLeadObservation(
          sum(w * obs.distance for w, obs in zip(weights, observations, strict=True)) / total,
          sum(w * obs.lateral for w, obs in zip(weights, observations, strict=True)) / total,
          min(a.std, b.std),
          max(a.probability, b.probability),
        )
        return [(valid, merged)]
    return [([i], observations[i]) for i in valid]

  def update(self, observations: list[VisionLeadObservation], leads: list[dict], timestamp: float, ego: float, valid: bool = True) -> list[dict]:
    if not valid or len(observations) != 2 or len(leads) != 2 or not all(math.isfinite(v) for v in (timestamp, ego)):
      self.reset()
      return leads
    if self.time is not None and not 0.01 <= timestamp - self.time <= 0.2:
      self.reset()
      return leads
    self.time = timestamp
    previous_slots = self.history_slots
    tracks = [track for track in self.tracks if timestamp - track.time <= 0.2]
    groups = self._groups(observations)
    costs = [[track.association_cost(obs, timestamp, ego) for track in tracks] for _, obs in groups]
    # Exhaustive assignment is tiny (at most two tracks); retain history on slot swaps.
    options = []
    for assignment in product(range(-1, len(tracks)), repeat=len(groups)):
      assigned = [i for i in assignment if i >= 0]
      if len(set(assigned)) != len(assigned) or any(i >= 0 and not math.isfinite(costs[g][i]) for g, i in enumerate(assignment)):
        continue
      options.append(((-len(assigned), sum(costs[g][i] for g, i in enumerate(assignment) if i >= 0)), assignment))
    assignment = min(options)[1]
    self.slots = [None, None]
    self.history_slots = [None, None]
    used = []
    output = list(leads)
    for g, (indices, observation) in enumerate(groups):
      track = tracks[assignment[g]] if assignment[g] >= 0 else None
      if track is not None and not track.update(observation, timestamp, ego):
        # A rejected state must not survive through the other, missing slot.
        track = None
        previous_slots = [None if old is tracks[assignment[g]] else old for old in previous_slots]
      if track is None:
        track = _DistanceTrack(observation, timestamp, ego)
      used.append(track)
      for i in indices:
        self.slots[i] = self.history_slots[i] = track
      accepted = [i for i in indices if leads[i].get('present', False) and not leads[i].get('radar', False)]
      supported = (
        accepted
        and 15.0 < ego < 60.0
        and all(10.0 < leads[i]['dRel'] < 150.0 and all(math.isfinite(leads[i][key]) for key in ('vLead', 'aLeadK')) for i in accepted)
      )
      if not supported:
        track.output, track.mode, track.offset, track.transition = None, 'fallback', 0.0, 1.0
        continue
      speed = track.speed([leads[i] for i in accepted], observation, ego)
      if (track.mode == 'fallback' and track.transition >= 1.0) or (track.mode == 'braking' and not track.ready):
        continue
      for i in accepted:
        output[i] = dict(leads[i], vLead=speed, vLeadK=speed, vRel=speed - ego)

    # Brief missing observations retain only private history, never lead presence.
    for i, observation in enumerate(observations):
      track = previous_slots[i]
      if not observation.valid() and track in tracks:
        self.history_slots[i] = track
        if track in used:
          continue
        used.append(track)
        speed = leads[i].get('vLead', math.nan)
        if leads[i].get('present', False) and not leads[i].get('radar', False) and math.isfinite(speed):
          track.output, track.mode, track.offset, track.transition = speed, 'fallback', 0.0, 1.0
    self.tracks = used[:2]
    return output
