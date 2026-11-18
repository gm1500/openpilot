"""Bounded observation of diverging lead hypotheses during an ego lane change.

Coordinates are raw model calibrated FRD (forward/right/down), including x's
camera origin. Speeds are model absolute speeds corrected for model ego-speed
error. The two model heads are hypotheses, not persistent vehicle identities.

A warning means that a previously coherent pair has split. A real closer
vehicle can produce the same evidence. No lane occupancy, camera association,
replacement speed, or permission to ignore a lead is inferred here.
"""

from dataclasses import dataclass, field
import math


@dataclass(frozen=True)
class DepartureHead:
  x: float
  y: float
  speed: float
  probability: float
  x_std: float
  y_std: float

  def valid(self):
    return (all(isinstance(v, (int, float)) and math.isfinite(v)
                for v in (self.x, self.y, self.speed, self.probability, self.x_std, self.y_std))
            and 0. < self.x <= 250. and abs(self.y) <= 20. and 0. <= self.speed <= 80.
            and .5 <= self.probability <= 1. and 0. < self.x_std <= 100. and 0. < self.y_std <= 20.)


@dataclass(frozen=True)
class DepartureStatus:
  reason: str = 'inactive'
  active: bool = False
  age: float = 0.
  range_residual: float = 0.
  range_separation: float = 0.
  speed_separation: float = 0.
  lateral_separation: float = 0.
  near_head: int = -1
  can_ignore_lead: bool = field(default=False, init=False)


@dataclass(frozen=True)
class _Anchor:
  time: float
  x: float
  y: float
  speed: float


class LeadDepartureShadow:
  ARM_SECONDS = .5
  MAX_AGE = 2.5
  CONFIRM_SECONDS = .15
  MAX_GAP = .1

  def __init__(self):
    self.reset()

  def reset(self, reason='inactive'):
    self.last = None
    self.coherent_since = None
    self.anchor = None
    self.direction = None
    self.pose = (0., 0., 0.)
    self.split_since = None
    self.status = DepartureStatus(reason)

  @staticmethod
  def _coherent(a, b):
    return abs(a.x - b.x) <= 1.5 and abs(a.y - b.y) <= .35 and abs(a.speed - b.speed) <= 2.

  def update(self, heads, timestamp, ego_speed, measured_yaw, lane_change_state, lane_change_direction, valid=True):
    """Update diagnostics only. Never returns, edits, or retains control leads.

    The caller must reject invalid/stale ego and measured yaw samples. Invalid
    observations revoke the anchor. Acquisition is allowed only before a
    maneuver, so gaps or repeated splits cannot renew an ongoing episode.
    """
    if (not valid or not isinstance(heads, (list, tuple)) or len(heads) != 2
        or not all(isinstance(h, DepartureHead) and h.valid() for h in heads)
        or not all(isinstance(v, (float, int)) and math.isfinite(v) for v in (timestamp, ego_speed, measured_yaw))
        or timestamp < 0. or not 10. <= ego_speed <= 50. or abs(measured_yaw) > .2):
      self.reset('invalid_context')
      return
    previous = self.last
    if previous is not None and not .01 <= timestamp - previous[0] <= self.MAX_GAP + 1e-6:
      self.reset('observation_gap')
      self.last = (timestamp, ego_speed, measured_yaw)
      return
    self.last = (timestamp, ego_speed, measured_yaw)
    maneuver = lane_change_state in ('laneChangeStarting', 'laneChangeFinishing')
    if not maneuver:
      if lane_change_state not in ('off', 'preLaneChange'):
        self.reset('unsupported_maneuver')
        return
      if self.direction is not None:
        self.reset('maneuver_ended')
        self.last = (timestamp, ego_speed, measured_yaw)
      if self._coherent(*heads):
        if self.coherent_since is None:
          self.coherent_since = timestamp
        if timestamp - self.coherent_since >= self.ARM_SECONDS - 1e-9:
          self.anchor = _Anchor(timestamp, (heads[0].x + heads[1].x) / 2.,
                                (heads[0].y + heads[1].y) / 2., (heads[0].speed + heads[1].speed) / 2.)
      else:
        self.coherent_since = self.anchor = None
      self.status = DepartureStatus('coherent_pair_armed' if self.anchor is not None else 'coherent_pair_warmup')
      return
    if lane_change_direction not in ('left', 'right'):
      self.reset('unsupported_direction')
      return
    if self.direction is not None and lane_change_direction != self.direction:
      self.reset('direction_changed')
      return
    if self.anchor is None or previous is None:
      self.status = DepartureStatus('no_anchor')
      return
    if self.direction is None:
      if timestamp - self.anchor.time > self.MAX_GAP + 1e-6:
        self.reset('stale_anchor')
        return
      self.direction = lane_change_direction
    age = timestamp - self.anchor.time
    if age > self.MAX_AGE + 1e-9:
      self.reset('expired')
      return

    # Transport the frozen pre-maneuver hypothesis with measured ego motion.
    # Constant target motion along its original forward axis is an assumption,
    # so this residual is evidence of disagreement, not proof of a false lead.
    dt = timestamp - previous[0]
    angle = .5 * (previous[2] + measured_yaw) * dt
    distance = .5 * (previous[1] + ego_speed) * dt
    px, py, yaw = self.pose
    px += distance * math.cos(yaw + .5 * angle)
    py += distance * math.sin(yaw + .5 * angle)
    yaw += angle
    self.pose = (px, py, yaw)
    predicted_x = (math.cos(yaw) * (self.anchor.x + self.anchor.speed * age - px)
                   + math.sin(yaw) * (self.anchor.y - py))
    (near_index, near), (_, far) = sorted(enumerate(heads), key=lambda item: item[1].x)
    sign = -1. if self.direction == 'left' else 1.
    dx, dv, dy = far.x - near.x, far.speed - near.speed, sign * (far.y - near.y)
    candidate = measured_yaw * sign >= .005 and dx >= 4. and dv >= 1.4 and dy >= .55
    if not candidate:
      self.split_since = None
    elif self.split_since is None:
      self.split_since = timestamp
    active = self.split_since is not None and timestamp - self.split_since >= self.CONFIRM_SECONDS - 1e-9
    reason = 'split_ambiguous' if active else ('split_pending' if candidate else 'observing')
    self.status = DepartureStatus(reason, active, age, near.x - predicted_x, dx, dv, dy, near_index)
