"""Read-only, bounded lead reacquisition experiment. Never returns control leads.

Production track objects are identity keys only. Snapshots contain copied values;
no Kalman state, association, acceptance, or published lead is modified here.
"""

from collections import deque
from dataclasses import dataclass, field
import math
from statistics import median

from openpilot.selfdrive.controls.lib.vision_lead_tracker import VisionLeadObservation, VisionLeadTracker


@dataclass(frozen=True)
class ReacquisitionStatus:
  reason: str = 'inactive'
  active: bool = False
  age: float = 0.0
  distance: float = 0.0
  speed: float = 0.0
  delta_distance: float = 0.0
  delta_speed: float = 0.0
  std_ratio: float = 0.0


@dataclass(frozen=True)
class _Snapshot:
  track: object
  time: float
  ego: float
  yaw: float
  distance: float
  lateral: float
  speed: float
  std: float


@dataclass
class _Evidence:
  snapshot: _Snapshot
  samples: deque = field(default_factory=lambda: deque(maxlen=20))


@dataclass
class _Episode:
  anchor: _Snapshot
  started: float
  time: float
  ego: float
  yaw: float
  distance: float
  lateral: float
  candidate: VisionLeadObservation
  candidate_speed: float
  consistent: int = 1


class LeadReacquisitionShadow:
  MAX_HOLD = 0.20
  MIN_BASELINE_TIME = 0.45
  MIN_BASELINE_SAMPLES = 8
  MAX_BASELINE_AGE = 1.0
  MIN_TRACK_AGE = 1.5
  MAX_RANGE_ERROR = 15.0
  MAX_LATERAL_ERROR = 0.30
  MAX_SPEED_DIFFERENCE = 5.0
  MIN_TTC = 8.0
  MIN_HEADWAY = 1.0
  BRAKING_BYPASS = -0.15

  def __init__(self):
    self.time = None
    self.evidence: dict[object, _Evidence] = {}
    self.episodes: list[_Episode] = []
    self.status = [ReacquisitionStatus(), ReacquisitionStatus()]
    self.bridge_frames = [0, 0]

  def reset(self, reason='inactive'):
    # Keep lifetime counters for the low-rate diagnostic log.
    self.time = None
    self.evidence.clear()
    self.episodes.clear()
    self.status = [ReacquisitionStatus(reason), ReacquisitionStatus(reason)]

  @staticmethod
  def _predict(distance, lateral, speed, old_ego, ego, old_yaw, yaw, dt):
    rotation = 0.5 * (old_yaw + yaw)
    new_lateral = lateral + rotation * distance * dt  # model yaw is positive right
    travel = (0.5 * (old_ego + ego) + rotation * 0.5 * (lateral + new_lateral)) * dt
    return distance + speed * dt - travel, new_lateral

  @staticmethod
  def _vision(lead):
    return lead.get('present', False) and not lead.get('radar', False) and all(math.isfinite(lead.get(k, math.nan)) for k in ('dRel', 'vLead', 'aLeadK'))

  def _hazard(self, raw_leads, leads, ego):
    for lead in [*raw_leads, *leads]:
      if not lead.get('present', False):
        continue
      if not self._vision(lead):
        return 'unsupported_lead'
      if lead['aLeadK'] < self.BRAKING_BYPASS:
        return 'braking'
      distance, closing = lead['dRel'], max(ego - lead['vLead'], 0.0)
      if distance < max(20.0, self.MIN_HEADWAY * ego) or (closing > 0.0 and distance < self.MIN_TTC * closing):
        return 'short_gap_or_closing'
    return None

  def _record(self, observations, leads, tracks, timestamp, ego, yaw):
    # Do not count duplicate model hypotheses as independent observations.
    for track in dict.fromkeys(t for t in tracks if t is not None):
      indices = [i for i, t in enumerate(tracks) if t is track]
      if not (
        track.ready
        and track.age >= self.MIN_TRACK_AGE
        and track.mode == 'distance'
        and track.transition >= 1.0
        and timestamp >= track.uncertain_handoff_until
        and all(self._vision(leads[i]) and observations[i].valid() and observations[i].probability >= 0.9 for i in indices)
      ):
        continue
      i = min(indices, key=lambda j: observations[j].std)
      obs, lead = observations[i], leads[i]
      snapshot = _Snapshot(track, timestamp, ego, yaw, lead['dRel'], obs.lateral, lead['vLead'], obs.std)
      ev = self.evidence.setdefault(track, _Evidence(snapshot))
      # Freeze the baseline when noise grows, rather than comparing only with
      # the immediately previous (already degraded) measurement.
      baseline = median(s for _, s in ev.samples) if ev.samples else obs.std
      if obs.std <= 1.25 * baseline:
        if ev.samples and timestamp - ev.samples[-1][0] > 0.1 + 1e-6:
          ev.samples.clear()
        ev.samples.append((timestamp, obs.std))
      ev.snapshot = snapshot

  def update(self, observations, raw_leads, leads, tracks, timestamp, ego, yaw_rate, valid=True):
    """Compute diagnostic candidates only; inputs and production outputs are untouched."""
    if not valid or not all(math.isfinite(v) for v in (timestamp, ego)):
      self.reset('invalid')
      return
    if (
      len(observations) != 2
      or len(raw_leads) != 2
      or len(leads) != 2
      or len(tracks) != 2
      or yaw_rate is None
      or not math.isfinite(yaw_rate)
      or abs(yaw_rate) > 0.1
      or not 15.0 < ego < 50.0
    ):
      self.reset('unsupported')
      return
    if self.time is not None and not 0.01 <= timestamp - self.time <= 0.1 + 1e-6:
      self.reset('observation_gap')
    self.time = timestamp
    self.status = [ReacquisitionStatus('no_anchor'), ReacquisitionStatus('no_anchor')]
    hazard = self._hazard(raw_leads, leads, ego)
    if hazard:
      self.reset(hazard)
      self.time = timestamp
      return

    groups = [
      (indices, obs) for indices, obs in VisionLeadTracker._groups(observations) if all(self._vision(raw_leads[i]) and self._vision(leads[i]) for i in indices)
    ]
    active_tracks = {track for i, track in enumerate(tracks) if self._vision(leads[i]) and track is not None}
    # Existing episodes keep their original deadline, even through another reset.
    sources = [(ep.anchor, ep) for ep in self.episodes]
    episode_tracks = {anchor.track for anchor, _ in sources}
    for track, ev in self.evidence.items():
      history = ev.samples
      if (
        track not in episode_tracks
        and timestamp - ev.snapshot.time <= 0.1 + 1e-6
        and len(history) >= self.MIN_BASELINE_SAMPLES
        and history[-1][0] - history[0][0] >= self.MIN_BASELINE_TIME - 1e-6
        and timestamp - history[-1][0] <= self.MAX_BASELINE_AGE
      ):
        anchor = ev.snapshot
        sources.append(
          (_Snapshot(anchor.track, anchor.time, anchor.ego, anchor.yaw, anchor.distance, anchor.lateral, anchor.speed, median(s for _, s in history)), None)
        )

    matches = []
    for indices, obs in groups:
      options = []
      for anchor, ep in sources:
        if ep is None and all(tracks[i] is anchor.track for i in indices):
          continue
        if ep is None:
          distance, lateral = self._predict(anchor.distance, anchor.lateral, anchor.speed, anchor.ego, ego, anchor.yaw, yaw_rate, timestamp - anchor.time)
        else:
          distance, lateral = self._predict(ep.distance, ep.lateral, anchor.speed, ep.ego, ego, ep.yaw, yaw_rate, timestamp - ep.time)
        if (
          obs.valid()
          and obs.probability >= 0.9
          and obs.std >= max(10.0, 2.0 * anchor.std)
          and abs(obs.distance - distance) <= min(self.MAX_RANGE_ERROR, math.hypot(anchor.std, obs.std))
          and abs(obs.lateral - lateral) <= self.MAX_LATERAL_ERROR
        ):
          options.append((anchor, ep, distance, lateral))
      matches.append((indices, obs, options))

    continuing = []
    for indices, obs, options in matches:
      if len(options) != 1:
        reason = 'ambiguous_anchor' if options else 'no_match'
        for i in indices:
          self.status[i] = ReacquisitionStatus(reason)
        continue
      anchor, ep, distance, lateral = options[0]
      age = timestamp - ep.started if ep else 0.0
      ratio = obs.std / max(anchor.std, 1e-6)
      reason = None
      if anchor.track in active_tracks:
        reason = 'reacquired' if all(tracks[i] is anchor.track for i in indices) else 'anchor_still_assigned'
      elif sum(any(a.track is anchor.track for a, *_ in opts) for _, _, opts in matches) != 1:
        reason = 'ambiguous_target'
      elif ep is not None and age >= self.MAX_HOLD - 1e-6:
        reason = 'expired'
      elif any(abs(anchor.speed - leads[i]['vLead']) > self.MAX_SPEED_DIFFERENCE for i in indices):
        reason = 'speed_disagreement'
      candidate_speed = min(raw_leads[i]['vLead'] for i in indices)
      if ep is not None:
        dt = timestamp - ep.time
        predicted = ep.candidate.distance + (ep.candidate_speed - 0.5 * (ep.ego + ego)) * dt
        consistent = abs(obs.distance - predicted) <= 2.0 and abs(obs.lateral - ep.candidate.lateral) <= 0.3
        ep.consistent = ep.consistent + 1 if consistent else 1
        if ep.consistent >= 3 and (obs.distance < distance - 3.0 or candidate_speed < anchor.speed - 2.0):
          reason = reason or 'consistent_replacement'
      if reason is not None:
        for i in indices:
          self.status[i] = ReacquisitionStatus(reason, age=age, std_ratio=ratio)
        # Retire this baseline; repeated uncertain frames cannot re-arm it.
        self.evidence.pop(anchor.track, None)
        continue
      if ep is None:
        ep = _Episode(anchor, timestamp, timestamp, ego, yaw_rate, distance, lateral, obs, candidate_speed)
        self.evidence.pop(anchor.track, None)
      else:
        ep.time, ep.ego, ep.yaw = timestamp, ego, yaw_rate
        ep.distance, ep.lateral = distance, lateral
        ep.candidate, ep.candidate_speed = obs, candidate_speed
      continuing.append(ep)
      for i in indices:
        self.status[i] = ReacquisitionStatus(
          'predicting', True, age, distance, anchor.speed, distance - leads[i]['dRel'], anchor.speed - leads[i]['vLead'], ratio
        )
        self.bridge_frames[i] += 1
    self.episodes = continuing
    self.evidence = {t: ev for t, ev in self.evidence.items() if timestamp - ev.snapshot.time <= 0.1 + 1e-6}
    self._record(observations, leads, tracks, timestamp, ego, yaw_rate)
