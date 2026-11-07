"""Bounded vision-speed recovery during lead acquisition and target transitions.

The ordinary tracker remains authoritative. This observer uses a fresh, short
history, never a distance difference across a target handoff, and can only raise
an underestimated speed when the model hypotheses agree on the target and
its measured gap is opening.
"""

from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from openpilot.selfdrive.controls.lib.vision_lead_tracker import VisionLeadObservation


@dataclass(frozen=True)
class _Sample:
  time: float
  distance: float
  lateral: float
  std: float
  ego: float
  model_speed: float
  yaw_rate: float
  travel: float
  model_travel: float


class LeadTransitionRecovery:
  ACQUISITION_TIME = 3.0
  HISTORY_TIME = 1.5
  MIN_HISTORY = 0.7
  MIN_SAMPLES = 10
  CONFIRM_TIME = 0.15
  MIN_OPENING = 0.5
  RANGE_NOISE_FLOOR = 1.0
  FRAME_RANGE_JUMP = 5.0
  MAX_CORRECTION = 2.0  # m/s above the raw model; never a cruise-speed boost
  CORRECTION_RATE = 2.0  # m/s per second, upward only; hazards revoke immediately
  HANDOVER_RATE = 2.0  # release a settled correction smoothly; never delay a hazard reset

  def __init__(self):
    self.reset()

  def reset(self):
    self.time: float | None = None
    self._clear('inactive')

  def _clear(self, reason):
    self.history: deque[_Sample] = deque()
    self.acquired_at: float | None = None
    self.correction = 0.0
    self.confirm_time = 0.0
    self.speed_lower = 0.0
    self.reason = reason

  def _uncertain(self, reason, timestamp):
    # A brief rejected observation revokes the correction immediately, but
    # does not destroy private history. A persistent change starts a new car.
    self.correction = self.confirm_time = 0.0
    self.reason = reason
    if self.history and timestamp - self.history[-1].time > 0.2:
      self._clear(reason)

  def _observe(self, observation, timestamp, ego, model_speed, yaw_rate):
    travel = model_travel = 0.0
    if self.history:
      previous = self.history[-1]
      dt = timestamp - previous.time
      yaw = 0.5 * (yaw_rate + previous.yaw_rate)
      # yRel is positive left; model yaw is positive right. A left turn
      # therefore brings a car on our left toward the centre of the image.
      lateral_prediction = previous.lateral + yaw * previous.distance * dt
      ego_travel = (0.5 * (ego + previous.ego) + yaw * 0.5 * (observation.lateral + previous.lateral)) * dt
      model_step = 0.5 * (model_speed + previous.model_speed) * dt
      prediction = previous.distance + model_step - ego_travel
      gate = min(12.0, max(self.FRAME_RANGE_JUMP, math.hypot(previous.std, observation.std) * 0.75))
      travel = previous.travel + ego_travel
      model_travel = previous.model_travel + model_step
      first = self.history[0]
      cumulative_error = observation.distance - first.distance + travel - first.travel - (model_travel - first.model_travel)
      # An uncertainty-scaled frame gate alone can join two different cars.
      # Also reject sustained displacement incompatible with model motion.
      distance_error = abs(observation.distance - prediction)
      if distance_error > gate or abs(observation.lateral - lateral_prediction) > 0.5:
        self._clear('target_change')
        travel = model_travel = 0.0
      elif distance_error > self.FRAME_RANGE_JUMP or abs(cumulative_error) > 8.0:
        # Model uncertainty may justify retaining private history, but not
        # publishing its opening speed across a possible small cut-in. One
        # outlier can recover; a persistent displacement starts a fresh track.
        self._uncertain('range_jump', timestamp)
        if self.history:
          return False
        travel = model_travel = 0.0
    if not self.history:
      self.acquired_at = timestamp
    self.history.append(_Sample(timestamp, observation.distance, observation.lateral, observation.std,
                                ego, model_speed, yaw_rate, travel, model_travel))
    while timestamp - self.history[0].time > self.HISTORY_TIME:
      self.history.popleft()
    return True

  def _opening_speed(self):
    """Conservative speed from causal, robust range regression, in m/s."""
    if len(self.history) < self.MIN_SAMPLES or self.history[-1].time - self.history[0].time < self.MIN_HISTORY:
      return None
    t = np.array([s.time - self.history[-1].time for s in self.history])
    x = np.array([s.distance + s.travel - self.history[0].travel for s in self.history])
    std = np.array([s.std for s in self.history])
    weights = 1.0 / np.maximum(std, 2.0) ** 2
    # Correlated hypotheses are one observation. Limit how much a sudden
    # change in reported uncertainty can weight one end of this short window.
    weights = np.maximum(weights, weights.max() / 4.0)
    for iteration in range(2):
      mean_t, mean_x = np.average(t, weights=weights), np.average(x, weights=weights)
      centred = t - mean_t
      denominator = float(np.sum(weights * centred ** 2))
      speed = float(np.sum(weights * centred * (x - mean_x)) / denominator)
      residual = x - mean_x - speed * centred
      scale = max(self.RANGE_NOISE_FLOOR, 1.4826 * float(np.median(np.abs(residual - np.median(residual)))))
      if iteration == 0:
        weights *= np.minimum(1.0, 2.5 * scale / np.maximum(np.abs(residual), 1e-6))
    # Normalize weights and retain a noise floor: quiet, correlated frames
    # cannot claim zero uncertainty about a newly acquired car's velocity.
    variance = max(self.RANGE_NOISE_FLOOR ** 2, float(np.average(residual ** 2, weights=weights)))
    effective_n = float(weights.sum() ** 2 / np.sum(weights ** 2))
    time_variance = float(np.average(centred ** 2, weights=weights))
    error = math.sqrt(variance / (max(effective_n - 2.0, 1.0) * time_variance))
    lower = speed - max(0.75, 2.0 * error)
    return lower, float(np.mean([s.ego for s in self.history]))

  def update(self, observations: list[VisionLeadObservation], raw_leads: list[dict], leads: list[dict], speed_stds: list[float],
             timestamp: float, ego: float, yaw_rate: float | None, valid: bool = True) -> list[dict]:
    if not valid or not math.isfinite(timestamp) or not math.isfinite(ego):
      self.reset()
      return leads
    dt = timestamp - self.time if self.time is not None else 0.0
    if self.time is not None and not 0.01 <= dt <= 0.1:
      self.reset()
      dt = 0.0
    self.time = timestamp
    if not (len(observations) == len(raw_leads) == len(leads) == len(speed_stds) == 2 and
            yaw_rate is not None and math.isfinite(yaw_rate) and abs(yaw_rate) < 0.5 and 15.0 < ego < 50.0):
      self._clear('unsupported')
      return leads
    for obs, raw, lead, speed_std in zip(observations, raw_leads, leads, speed_stds, strict=True):
      if not (obs.valid() and obs.probability >= 0.9 and obs.std <= 30.0 and
              raw.get('present', False) and lead.get('present', False) and not raw.get('radar', False) and not lead.get('radar', False) and
              all(math.isfinite(raw.get(k, math.nan)) and math.isfinite(lead.get(k, math.nan)) for k in ('dRel', 'vLead', 'aLeadK')) and
              math.isfinite(speed_std) and 0.0 < speed_std <= 5.0):
        self._clear('uncertain')
        return leads
      closing = max(ego - raw['vLead'], ego - lead['vLead'], 0.0)
      if (min(raw['dRel'], lead['dRel']) < max(30.0, 1.5 * ego, 12.0 * closing) or raw['vLead'] < ego - 3.5 or
          min(raw['aLeadK'], lead['aLeadK']) < -0.15 or lead['vLead'] < raw['vLead'] - 0.1):
        self._clear('closing_or_braking')
        return leads
    a, b = observations
    # Do not pick the faster hypothesis or use a distance change between them.
    if abs(a.distance - b.distance) > 1.5 or abs(a.lateral - b.lateral) > 0.35:
      self._uncertain('ambiguous_target', timestamp)
      return leads
    weights = [obs.probability / max(obs.std, 1.0) ** 2 for obs in observations]
    observation = VisionLeadObservation(
      float(np.average([a.distance, b.distance], weights=weights)),
      float(np.average([a.lateral, b.lateral], weights=weights)), min(a.std, b.std), min(a.probability, b.probability))
    baseline_speed = min(lead['vLead'] for lead in raw_leads)
    if not self._observe(observation, timestamp, ego, baseline_speed, yaw_rate):
      return leads
    # Observe continuously, but only add a correction while acquiring a new
    # target. Keep settled history alive to detect the next cut-in, cut-out or
    # reacquisition; expiry must not itself create another recovery window.
    settled = self.acquired_at is None or timestamp - self.acquired_at > self.ACQUISITION_TIME
    if settled and self.correction <= 0.0:
      self.correction = self.confirm_time = 0.0
      self.reason = 'tracking'
      return leads
    estimate = self._opening_speed()
    if estimate is None:
      self.reason = 'acquiring'
      return leads
    lower, mean_ego = estimate
    self.speed_lower = lower
    if lower <= max(mean_ego, ego) + self.MIN_OPENING:
      self.correction = self.confirm_time = 0.0
      self.reason = 'no_opening'
      return leads
    self.confirm_time += dt
    if self.confirm_time < self.CONFIRM_TIME:
      self.reason = 'confirming'
      return leads
    correction = min(self.MAX_CORRECTION, min(speed_stds), max(0.0, lower - baseline_speed))
    if settled:
      self.correction = min(correction, max(0.0, self.correction - self.HANDOVER_RATE * dt))
      self.reason = 'handover' if self.correction > 0.0 else 'tracking'
    else:
      self.correction = min(correction, self.correction + self.CORRECTION_RATE * dt)
      self.reason = 'opening' if self.correction > 0.0 else 'no_correction'
    # Only speed is changed. Presence, distance, braking acceleration and all
    # planner/cruise limits retain their original owners and immediate response.
    output = list(leads)
    for i, (lead, raw) in enumerate(zip(leads, raw_leads, strict=True)):
      speed = min(raw['vLead'] + self.correction, lower)
      if speed > lead['vLead']:
        output[i] = dict(lead, vLead=speed, vLeadK=speed, vRel=speed - ego)
    return output
