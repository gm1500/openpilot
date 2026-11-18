"""POC: bounded vision lead corrections from the model's 0/2/4 second positions.

The future x samples share a stationary ego-start frame: dx/dt is lead speed,
not relative speed. These are correlated predictions, not extra measurements;
they never update the distance track's Kalman state or covariance.
"""

from dataclasses import dataclass
import math

from openpilot.selfdrive.controls.lib.vision_lead_tracker import _DistanceTrack


def clip(value: float, lower: float, upper: float) -> float:
  return min(max(value, lower), upper)


@dataclass(frozen=True)
class FutureMotion:
  speed: float
  acceleration: float
  speed_std: float

  @classmethod
  def from_model(cls, lead, max_speed_std=8.0):
    if not 0.9 <= lead.prob <= 1.0 or min(len(lead.t), len(lead.x), len(lead.xStd)) < 3:
      return None
    t, x, std = ([float(values[i]) for i in range(3)] for values in (lead.t, lead.x, lead.xStd))
    if not all(math.isfinite(v) for v in t + x + std):
      return None
    if any(abs(a - b) > 1e-3 for a, b in zip(t, (0.0, 2.0, 4.0), strict=True)) or min(std) <= 0.0:
      return None
    # Quadratic endpoint derivative. Subtracting a constant camera offset has
    # no effect. In particular, do not add vEgo to this velocity a second time.
    first, second = x[1] - x[0], x[2] - x[1]
    speed = (3.0 * first - second) / 4.0
    acceleration = (second - first) / 4.0
    # Unknown cross-horizon covariance: use the sum of absolute contributions,
    # not an independent-noise assumption or reduced uncertainty from two slots.
    speed_std = max(1.0, 0.75 * std[0] + std[1] + 0.25 * std[2])
    if (not 0.0 <= speed <= 70.0 or not -5.0 <= acceleration <= 3.0 or speed_std > max_speed_std
        or x[1] < x[0] or x[2] < x[1] or not -0.5 <= speed + 4.0 * acceleration <= 70.0):
      return None
    return cls(speed, acceleration, speed_std)


@dataclass(frozen=True)
class FutureMotionStatus:
  reason: str = 'reset'
  speed: float = 0.0
  speed_std: float = 0.0
  weight: float = 0.0
  delta_speed: float = 0.0
  delta_distance: float = 0.0


@dataclass
class _State:
  identity: tuple[int, int]
  timestamp: float
  track_age: float
  ego: float
  distance: float
  motion: FutureMotion
  qualified: float = 0.0
  delta_speed: float = 0.0


class LeadFutureMotion:
  MAX_WEIGHT = 0.35
  MAX_SLOWER = 2.0  # m/s relative to the existing estimator, not per update
  MAX_FASTER = 1.0
  SLOWER_RATE = 2.0  # m/s per second of additional correction
  FASTER_RATE = 1.0
  RANGE_BOUND = 0.30  # m relative to the existing estimator
  CONFIRM_TIME = 0.20

  def __init__(self):
    self.reset()

  def reset(self):
    self.states: dict[_DistanceTrack, _State] = {}
    self.status = [FutureMotionStatus(), FutureMotionStatus()]

  @staticmethod
  def _reason(indices, raw_leads, leads, track, ego):
    if any(not lead.get('present', False) or lead.get('radar', False)
           for i in indices for lead in (raw_leads[i], leads[i])):
      return 'not_vision'
    if not all(math.isfinite(lead.get(key, math.nan)) for i in indices
               for lead in (raw_leads[i], leads[i]) for key in ('dRel', 'vLead', 'aLeadK')):
      return 'invalid'
    if not 15.0 < ego < 50.0 or any(not 10.0 < raw_leads[i]['dRel'] < 150.0 for i in indices):
      return 'unsupported'
    if not track.continuity.continuous or track.transition < 1.0 or not math.isfinite(track.continuity.speed):
      return 'continuity'
    if track.mode in ('braking', 'stop-hold', 'stopped-lead'):
      return 'braking'
    if any(lead['aLeadK'] < -0.5 or lead['vLead'] < 1.0 or lead['dRel'] < 6.0 * max(ego - lead['vLead'], 0.0)
           for i in indices for lead in (raw_leads[i], leads[i])):
      return 'braking'
    if max(leads[i]['vLead'] for i in indices) - min(leads[i]['vLead'] for i in indices) > 1.0:
      return 'slot_disagreement'
    return 'eligible'

  def update(self, model_leads, raw_leads: list[dict], leads: list[dict], tracks: list[_DistanceTrack | None],
             timestamp: float, ego: float, valid: bool = True) -> list[dict]:
    if (not valid or len(model_leads) < 2 or any(len(values) != 2 for values in (raw_leads, leads, tracks))
        or not all(math.isfinite(v) for v in (timestamp, ego))):
      self.reset()
      return leads
    output = list(leads)
    states = {}
    self.status = [FutureMotionStatus('no_track'), FutureMotionStatus('no_track')]
    # Key state by the existing associated track, not the changing model slot.
    # A shared track gets one correction and no extra confidence for two slots.
    groups = {}
    for i, track in enumerate(tracks):
      if track is not None:
        groups.setdefault(track, []).append(i)
    for track, indices in groups.items():
      reason = self._reason(indices, raw_leads, leads, track, ego)
      motions = [FutureMotion.from_model(model_leads[i]) for i in indices] if reason == 'eligible' else []
      if reason == 'eligible' and any(motion is None for motion in motions):
        reason = 'prediction_invalid'
      if reason != 'eligible':
        for i in indices:
          self.status[i] = FutureMotionStatus(reason)
        continue
      if (max(m.speed for m in motions) - min(m.speed for m in motions) > 2.0
          or max(m.acceleration for m in motions) - min(m.acceleration for m in motions) > 1.0):
        for i in indices:
          self.status[i] = FutureMotionStatus('prediction_disagreement')
        continue
      motion = FutureMotion(sum(m.speed for m in motions) / len(motions),
                            sum(m.acceleration for m in motions) / len(motions), max(m.speed_std for m in motions))
      distance = sum(leads[i]['dRel'] for i in indices) / len(indices)
      base_speed = sum(leads[i]['vLead'] for i in indices) / len(indices)
      old = self.states.get(track)
      dt = timestamp - old.timestamp if old is not None else 0.0
      # An in-place tracker reset can retain object identity but resets age.
      reanchor = old is None or old.identity != track.identity or not 0.01 <= dt <= 0.1 or track.age <= old.track_age
      if not reanchor:
        predicted = old.distance + old.motion.speed * dt + 0.5 * old.motion.acceleration * dt**2 - 0.5 * (old.ego + ego) * dt
        reanchor = abs(distance - predicted) > 0.75 or abs(motion.speed - old.motion.speed) > 3.0
      else:
        predicted = distance
      state = _State(track.identity, timestamp, track.age, ego, distance, motion)
      states[track] = state
      weight = self.MAX_WEIGHT / (1.0 + (motion.speed_std / 4.0)**2)
      reason = 'reanchor'
      delta_distance = 0.0
      if not reanchor:
        state.qualified = old.qualified + dt
        reason = 'confirming'
        if state.qualified >= self.CONFIRM_TIME:
          reason = 'active'
          # Require observed motion to support the direction. Predicted positions
          # and direct model velocity are correlated; agreement is not evidence
          # that can overrule a well-established observed closing trend.
          measured = track.continuity.speed
          if motion.speed < base_speed:
            target = min(base_speed, max(motion.speed, measured + 0.75))
          else:
            target = max(base_speed, min(motion.speed, measured - 0.75))
          desired = clip(weight * (target - base_speed), -self.MAX_SLOWER, self.MAX_FASTER)
          # Never retain an old correction after the supporting evidence disappears.
          previous = old.delta_speed if old.delta_speed * desired > 0.0 else 0.0
          delta = previous + clip(desired - previous, -self.SLOWER_RATE * dt, self.FASTER_RATE * dt)
          state.delta_speed = clip(delta, min(0.0, desired), max(0.0, desired))
          # Positive corrections must not undo existing braking/pre-closing cues.
          if (track.mode != 'distance' or any(lead['aLeadK'] < -0.15
              for i in indices for lead in (raw_leads[i], leads[i]))):
            state.delta_speed = min(state.delta_speed, 0.0)
          # The previous forecast is a short prior for *current* range, corrected
          # by the current observation. Never insert x[1] as today's dRel.
          if track.ready and track.mode == 'distance':
            delta_distance = clip(min(weight, 0.25) * (predicted - distance), -self.RANGE_BOUND, self.RANGE_BOUND)
            if any(ego > lead['vLead'] + 0.5 or lead['aLeadK'] < -0.15
                   for i in indices for lead in (raw_leads[i], leads[i])):
              delta_distance = min(delta_distance, 0.0)
          state.distance += delta_distance
          for i in indices:
            speed = max(0.0, leads[i]['vLead'] + state.delta_speed)
            output[i] = dict(leads[i], dRel=leads[i]['dRel'] + delta_distance,
                             vLead=speed, vLeadK=speed, vRel=speed - ego)
      for i in indices:
        self.status[i] = FutureMotionStatus(reason, motion.speed, motion.speed_std, weight, state.delta_speed, delta_distance)
    # Missing, rejected or replaced tracks cannot carry correction into a new lead.
    self.states = states
    return output
