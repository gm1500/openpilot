"""Private vision history with bounded handoffs and mild planner-range correction.

Control acceptance and acceleration remain owned by radard. Model velocity
never updates the Kalman state; it remains the fallback and immediate brake guard.
"""

from collections import deque
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
  ACCEL_NOISE = 0.1  # nominal continuous white acceleration spectral density
  MIN_HISTORY = 1.5
  MAX_SPEED_VARIANCE = 2.25
  UNCERTAIN_MAX_DISTANCE = 15.0
  UNCERTAIN_MAX_LATERAL = 0.15
  UNCERTAIN_MIN_HISTORY = 5.0
  DISTANCE_CORRECTION_GAIN = 0.60  # mild kinematic range correction at 20 Hz
  PRE_CLOSING_HISTORY = 0.7
  PRE_CLOSING_WINDOW = 0.8
  PRE_CLOSING_TTC = 13.0
  PRE_CLOSING_WEIGHT = 0.35
  PRE_CLOSING_MAX_OFFSET = 3.0  # m/s, never accelerate an immature lead estimate
  PRE_CLOSING_OFFSET_RATE = 2.0  # m/s per second, additional downward cue only

  def __init__(self, observation: VisionLeadObservation, timestamp: float, ego: float):
    self.x = np.array([observation.distance, ego], dtype=float)
    self.P = np.diag([observation.variance, 1000.0])
    self.time, self.ego = timestamp, ego
    self.distance, self.lateral = observation.distance, observation.lateral
    self.distance_std = observation.std
    self.age = 0.0
    self.dt = 0.0
    self.output: float | None = None
    self.output_active = False
    self.mode = 'fallback'
    self.transition_from = 0.0
    self.transition = 1.0
    self.last_braking = -math.inf
    self.uncertain_handoff_until = -math.inf
    self.filtered_distance = observation.distance
    self.range_active = False
    self.range_history = deque([(timestamp, observation.distance, observation.probability)])
    self.pre_closing_offset = 0.0

  def reset_output(self) -> None:
    # Unsupported or missing output must not leave a stale range/transition
    # behind. Private association/Kalman history can still survive a brief gap.
    self.output, self.mode = None, 'fallback'
    self.output_active = False
    self.transition_from, self.transition = 0.0, 1.0
    self.filtered_distance = self.distance
    self.range_active = False
    self.pre_closing_offset = 0.0

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

  def uncertain_handoff_cost(self, observation: VisionLeadObservation, timestamp: float, ego: float) -> float:
    # Range uncertainty cannot establish target identity. A plausible match may
    # hand off its published speed, but must never transfer the Kalman state.
    dt = timestamp - self.time
    if (
      not self.ready or self.mode != 'distance' or self.age < self.UNCERTAIN_MIN_HISTORY
      or not 0.01 <= dt <= 0.1 or self.output is None or not math.isfinite(self.output)
      or not np.isfinite(self.x).all() or not np.isfinite(self.P).all()
      or observation.probability < 0.9 or observation.std < max(10.0, 2.0 * self.distance_std)
    ):
      return math.inf
    travel = (float(self.x[1]) - 0.5 * (ego + self.ego)) * dt
    distance_error = abs(observation.distance - (self.distance + travel))
    lateral_error = abs(observation.lateral - self.lateral)
    innovation = observation.distance - (float(self.x[0]) + travel)
    predicted_variance = self.P[0, 0] + 2.0 * dt * self.P[0, 1] + dt**2 * self.P[1, 1]
    if (
      distance_error > self.UNCERTAIN_MAX_DISTANCE or lateral_error > self.UNCERTAIN_MAX_LATERAL
      or innovation**2 > predicted_variance + observation.variance
    ):
      return math.inf
    return distance_error / self.UNCERTAIN_MAX_DISTANCE + lateral_error / self.UNCERTAIN_MAX_LATERAL

  def acceleration_noise(self, distance: float, ego: float) -> float:
    # Keep the original response within a one-second gap; gradually smooth farther leads.
    headway = distance / max(ego, 5.0)
    factor = float(np.interp(headway, [1.0, 2.5], [1.0, 0.5]))
    closing = max(ego - float(self.x[1]), 0.0)
    if closing > 0.0:
      # Distance alone must not slow the response to a rapidly closing lead.
      factor = max(factor, float(np.interp(distance / closing, [6.0, 12.0], [2.0, 0.5])))
    return self.ACCEL_NOISE * factor

  def update(self, observation: VisionLeadObservation, timestamp: float, ego: float) -> bool:
    dt = timestamp - self.time
    F = np.array([[1.0, dt], [0.0, 1.0]])
    Q = self.acceleration_noise(observation.distance, ego) * np.array([[dt**3 / 3, dt**2 / 2], [dt**2 / 2, dt]])
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
    self.distance_std = observation.std
    self.age += dt
    if dt > 0.1:
      self.range_history.clear()
    self.range_history.append((timestamp, observation.distance, observation.probability))
    while timestamp - self.range_history[0][0] > self.PRE_CLOSING_WINDOW:
      self.range_history.popleft()
    return True

  def pre_closing_target(self, baseline: float, observation: VisionLeadObservation, ego: float) -> float:
    # An immature Kalman speed is only a small downward cue. Require both its
    # conservative closing estimate and a sustained raw-range trend to agree.
    offset = 0.0
    available_offset = min(self.PRE_CLOSING_MAX_OFFSET, self.PRE_CLOSING_WEIGHT * max(baseline - float(self.x[1]), 0.0))
    history = self.range_history
    if (
      self.age >= self.PRE_CLOSING_HISTORY and len(history) >= 7
      and history[-1][0] - history[0][0] >= 0.6
      and min(point[2] for point in history) > 0.9
      and 0.0 <= self.x[1] < baseline and self.P[1, 1] <= 36.0
      and self.time >= self.uncertain_handoff_until and self.transition >= 1.0
    ):
      times = np.array([point[0] - self.time for point in history])
      distances = np.array([point[1] for point in history])
      times -= times.mean()
      distances -= distances.mean()
      time_variance = float(times @ times)
      rate = float(times @ distances) / time_variance
      residual = distances - rate * times
      rate_std = math.sqrt(float(residual @ residual) / ((len(history) - 2) * time_variance))
      closing = min(-rate - 2.0 * rate_std, ego - float(self.x[1]) - math.sqrt(self.P[1, 1]))
      if closing > observation.distance / self.PRE_CLOSING_TTC:
        offset = available_offset
    step = self.PRE_CLOSING_OFFSET_RATE * self.dt
    self.pre_closing_offset += float(np.clip(offset - self.pre_closing_offset, -step, step))
    self.pre_closing_offset = min(self.pre_closing_offset, available_offset)
    return max(0.0, baseline - self.pre_closing_offset)

  def filtered_range(self, raw_distance: float, speed: float, ego: float, active: bool) -> float:
    # Predict range from the same lead speed sent to planning, then accept most
    # of each new camera range sample. This rejects single-frame range noise
    # without the braking lag of a conventional low-pass filter.
    active = active and self.dt > 0.0 and all(math.isfinite(v) for v in (raw_distance, speed, ego, self.filtered_distance))
    if not active or not self.range_active:
      self.range_active = active
      self.filtered_distance = raw_distance
      return raw_distance
    predicted = self.filtered_distance + (speed - ego) * self.dt
    self.filtered_distance = predicted + self.DISTANCE_CORRECTION_GAIN * (raw_distance - predicted)
    return float(self.filtered_distance)

  def speed(self, leads: list[dict], observation: VisionLeadObservation, ego: float) -> float:
    baseline = min(lead['vLead'] for lead in leads)
    use_distance = self.ready and observation.probability >= 0.5 and 15.0 < ego < 60.0 and 10.0 < observation.distance < 150.0
    target = float(self.x[1]) if use_distance else baseline
    mode = 'distance' if use_distance else 'fallback'
    if not use_distance:
      target = self.pre_closing_target(baseline, observation, ego)
      if self.pre_closing_offset > 0.0:
        mode = 'pre-closing'
    else:
      self.pre_closing_offset = 0.0
    closing = ego - baseline
    closing_time = 8.0 if self.time < self.uncertain_handoff_until else 5.0
    urgent = min(lead['aLeadK'] for lead in leads) < -0.5 or (closing > 0.0 and min(lead['dRel'] for lead in leads) < closing_time * closing)
    if urgent:
      self.last_braking = self.time
    recovering = self.time - self.last_braking < 2.0 and target > baseline + 0.5
    if urgent or recovering:
      target = min(target, baseline)
      mode = 'braking'
      self.transition_from, self.transition = 0.0, 1.0
      self.pre_closing_offset = 0.0
    elif mode == 'pre-closing' or (not use_distance and (not self.output_active or self.mode == 'pre-closing')):
      # Pre-closing already ramps its bounded offset. Pure model fallback must
      # not blend from a shared internal minimum that was never published.
      self.transition_from, self.transition = 0.0, 1.0
    elif self.output is not None and mode != self.mode:
      # A fixed starting value keeps a changing target inside the blend.
      self.transition_from, self.transition = self.output, 0.0
    else:
      self.transition = min(1.0, self.transition + self.dt)
    self.mode = mode
    self.output = float(np.clip(self.transition * target + (1.0 - self.transition) * self.transition_from, 0.0, 70.0))
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
    # A brief split may bridge the published speed, but must not widen the
    # grouping gates or copy a stale Kalman state into a new track.
    shared = previous_slots[0]
    bridge = None
    if (
      shared is not None and shared is previous_slots[1] and shared.ready and shared.mode == 'distance'
      and all(obs.valid() and math.isfinite(shared.association_cost(obs, timestamp, ego)) for obs in observations)
      and abs(observations[0].distance - observations[1].distance) <= 3.0
      and abs(observations[0].lateral - observations[1].lateral) <= 0.5
    ):
      bridge = shared.output
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
    handoffs = [None] * len(groups)
    for j, track in enumerate(tracks):
      if j in assignment:
        continue
      matches = []
      for g, (indices, observation) in enumerate(groups):
        # Only an unassigned old track can hand off to one unambiguous group
        # in its previous slots. Never borrow speed from another matched car.
        supported = assignment[g] < 0 and 15.0 < ego < 60.0 and all(
          previous_slots[i] is track and leads[i].get('present', False) and not leads[i].get('radar', False)
          and all(math.isfinite(leads[i][key]) for key in ('dRel', 'vLead', 'aLeadK'))
          and 10.0 < leads[i]['dRel'] < 150.0 and leads[i]['aLeadK'] >= -0.5
          and leads[i]['dRel'] >= 8.0 * max(ego - leads[i]['vLead'], 0.0)
          for i in indices
        )
        if supported and math.isfinite(track.uncertain_handoff_cost(observation, timestamp, ego)):
          matches.append(g)
      if len(matches) == 1:
        handoffs[matches[0]] = track.output
    self.slots = [None, None]
    self.history_slots = [None, None]
    used = []
    output = list(leads)
    for g, (_indices, observation) in enumerate(groups):
      track = tracks[assignment[g]] if assignment[g] >= 0 else None
      if track is not None and not track.update(observation, timestamp, ego):
        # A rejected state must not survive through the other, missing slot.
        track = None
        bridge = None
        previous_slots = [None if old is tracks[assignment[g]] else old for old in previous_slots]
      if track is None:
        track = _DistanceTrack(observation, timestamp, ego)
      used.append(track)

    for (indices, observation), track, handoff in zip(groups, used, handoffs, strict=True):
      initial_output = handoff if handoff is not None else bridge
      if track.age == 0.0 and initial_output is not None:
        # The existing one-second fallback transition expires this bridge.
        # Current braking/closing guards still override it immediately.
        track.output, track.mode = initial_output, 'distance'
        track.output_active = True
        if handoff is not None:
          track.uncertain_handoff_until = timestamp + 1.0
      for i in indices:
        self.slots[i] = self.history_slots[i] = track
      accepted = [i for i in indices if leads[i].get('present', False) and not leads[i].get('radar', False)]
      supported = (
        accepted
        and 15.0 < ego < 60.0
        and all(10.0 < leads[i]['dRel'] < 150.0 and all(math.isfinite(leads[i][key]) for key in ('vLead', 'aLeadK')) for i in accepted)
      )
      if not supported:
        track.reset_output()
        continue
      speed = track.speed([leads[i] for i in accepted], observation, ego)
      urgent_range = track.mode == 'braking' or timestamp < track.uncertain_handoff_until
      filtered_group_distance = track.filtered_range(observation.distance, speed, ego, track.mode == 'distance' and not urgent_range)
      if (track.mode == 'fallback' and track.transition >= 1.0) or (track.mode == 'braking' and not track.ready):
        track.output_active = False
        continue
      track.output_active = True
      for i in accepted:
        # Preserve any small per-hypothesis range offset when both slots share a track.
        d_rel = filtered_group_distance + (leads[i]['dRel'] - observation.distance)
        # Fallback hypotheses can disagree on model speed. The immature cue
        # must not replace either baseline with the other slot's lower speed.
        slot_speed = max(0.0, leads[i]['vLead'] - track.pre_closing_offset) if track.mode == 'pre-closing' else speed
        output[i] = dict(leads[i], dRel=d_rel, vLead=slot_speed, vLeadK=slot_speed, vRel=slot_speed - ego)

    # Brief missing observations retain only private history, never lead presence.
    for i, observation in enumerate(observations):
      track = previous_slots[i]
      if not observation.valid() and track in tracks:
        self.history_slots[i] = track
        if track in used:
          continue
        used.append(track)
        track.reset_output()
    self.tracks = used[:2]
    return output
