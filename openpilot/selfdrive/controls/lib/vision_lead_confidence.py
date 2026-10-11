"""Experimental, bounded confidence filtering after the V3 lead estimator.

Only published range and mild acceleration are shaped. Lead acceptance, speed,
association, Kalman state and planner policy remain owned by the existing code.
"""

from dataclasses import dataclass
import math

from openpilot.selfdrive.controls.lib.vision_lead_tracker import _DistanceTrack


@dataclass(frozen=True)
class LeadUncertainty:
  probability: float
  distance_std: float
  speed_std: float
  acceleration_std: float


@dataclass
class _FilterState:
  track: _DistanceTrack
  timestamp: float
  distance: float
  acceleration: float
  raw_distance: float
  raw_acceleration: float


@dataclass(frozen=True)
class ConfidenceStatus:
  reason: str = 'reset'
  range_gain: float = 1.0
  accel_gain: float = 1.0


def _clip(value: float, lower: float, upper: float) -> float:
  return min(max(value, lower), upper)


class VisionLeadConfidenceFilter:
  RANGE_SCALE = 5.0
  ACCEL_SCALE = 0.35
  MIN_RANGE_GAIN = 0.30
  MIN_ACCEL_GAIN = 0.25
  RANGE_BOUND = 0.75  # m, relative to the current V3 output
  ACCEL_BOUND = 0.08  # m/s^2, relative to the current model acceleration
  MIN_AGE = 2.0
  MIN_TTC = 12.0
  MIN_HEADWAY = 1.5
  BRAKING_BYPASS = -0.15
  RANGE_JUMP = 3.0
  ACCEL_JUMP = 0.20

  def __init__(self):
    self.reset()

  def reset(self) -> None:
    self.states: list[_FilterState | None] = [None, None]
    self.status = [ConfidenceStatus(), ConfidenceStatus()]

  def _reason(self, index: int, raw_leads: list[dict], leads: list[dict], uncertainty: LeadUncertainty,
              tracks: list[_DistanceTrack | None], timestamp: float, ego: float) -> str:
    raw, lead, track, old = raw_leads[index], leads[index], tracks[index], self.states[index]
    # radard intentionally publishes only {'present': False} for a missing lead.
    if not raw.get('present', False) or not lead.get('present', False) or raw.get('radar', False) or lead.get('radar', False):
      return 'not_vision'
    values = (uncertainty.probability, uncertainty.distance_std, uncertainty.speed_std, uncertainty.acceleration_std,
              raw.get('dRel', math.nan), raw.get('vLead', math.nan), raw.get('aLeadK', math.nan),
              lead.get('dRel', math.nan), lead.get('vLead', math.nan), lead.get('aLeadK', math.nan))
    if not all(math.isfinite(value) for value in values):
      return 'invalid'
    if not (15.0 < ego < 60.0 and 10.0 < raw['dRel'] < 150.0):
      return 'unsupported'
    if track is None or track.age < self.MIN_AGE or not track.ready:
      return 'new_track'
    if track.mode != 'distance' or track.transition < 1.0 or timestamp < track.uncertain_handoff_until:
      return 'transition_or_braking'
    # Either accepted hypothesis can bypass smoothing for the shared track.
    shared_accels = [lead.get('aLeadK', math.nan) for lead, other in zip(raw_leads, tracks, strict=True)
                     if other is track and lead.get('present', False)]
    if not all(math.isfinite(accel) for accel in shared_accels):
      return 'invalid'
    if min(shared_accels) < self.BRAKING_BYPASS:
      return 'deceleration'
    if raw['dRel'] / max(ego, 1.0) < self.MIN_HEADWAY:
      return 'short_gap'
    speed, variance = float(track.x[1]), float(track.P[1, 1])
    if not math.isfinite(speed) or not math.isfinite(variance):
      return 'invalid'
    closing = max(ego - raw['vLead'], ego - lead['vLead'], ego - speed + math.sqrt(max(variance, 0.0)), 0.0)
    if raw['dRel'] < self.MIN_TTC * closing:
      return 'closing'
    if not 0.5 <= uncertainty.probability <= 1.0 or uncertainty.distance_std <= 0.0 or uncertainty.acceleration_std <= 0.0:
      return 'bad_uncertainty'
    if old is None or old.track is not track or not 0.01 <= timestamp - old.timestamp <= 0.1:
      return 'reanchor'
    if abs(raw['dRel'] - (old.raw_distance + (lead['vLead'] - ego) * (timestamp - old.timestamp))) > self.RANGE_JUMP:
      return 'range_jump'
    if abs(raw['aLeadK'] - old.raw_acceleration) > self.ACCEL_JUMP:
      return 'acceleration_jump'
    return 'active'

  def update(self, raw_leads: list[dict], leads: list[dict], uncertainties: list[LeadUncertainty],
             tracks: list[_DistanceTrack | None], timestamp: float, ego: float, valid: bool = True) -> list[dict]:
    if not valid or not all(len(items) == 2 for items in (raw_leads, leads, uncertainties, tracks)) or not all(
      math.isfinite(value) for value in (timestamp, ego)
    ):
      self.reset()
      return leads

    output = list(leads)
    for i, (raw, lead, uncertainty, track) in enumerate(zip(raw_leads, leads, uncertainties, tracks, strict=True)):
      reason = self._reason(i, raw_leads, leads, uncertainty, tracks, timestamp, ego)
      self.status[i] = ConfidenceStatus(reason)
      if reason not in ('active', 'reanchor'):
        self.states[i] = None
        continue
      assert track is not None
      distance, acceleration = lead['dRel'], lead['aLeadK']
      if reason == 'active':
        old = self.states[i]
        assert old is not None
        dt = timestamp - old.timestamp
        effective_std = max(uncertainty.distance_std, 1.0) / uncertainty.probability
        range_gain = _clip(1.0 / (1.0 + (effective_std / self.RANGE_SCALE)**2), self.MIN_RANGE_GAIN, 1.0)
        accel_gain = _clip(1.0 / (1.0 + (uncertainty.acceleration_std / self.ACCEL_SCALE)**2), self.MIN_ACCEL_GAIN, 1.0)
        # Normalize the 20 Hz gains to elapsed time. No future samples are used.
        range_gain = 1.0 - (1.0 - range_gain)**(dt / 0.05)
        accel_gain = 1.0 - (1.0 - accel_gain)**(dt / 0.05)
        predicted = old.distance + (lead['vLead'] - ego) * dt
        distance = _clip(predicted + range_gain * (distance - predicted), distance - self.RANGE_BOUND, distance + self.RANGE_BOUND)
        acceleration = _clip(old.acceleration + accel_gain * (acceleration - old.acceleration),
                             acceleration - self.ACCEL_BOUND, acceleration + self.ACCEL_BOUND)
        output[i] = dict(lead, dRel=distance, aLeadK=acceleration)
        self.status[i] = ConfidenceStatus(reason, range_gain, accel_gain)
      self.states[i] = _FilterState(track, timestamp, distance, acceleration, raw['dRel'], raw['aLeadK'])
    return output
