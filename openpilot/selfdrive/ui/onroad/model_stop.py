"""Display-only model stop prediction in the calibrated model frame."""
import math
from dataclasses import dataclass, replace

import numpy as np

from openpilot.selfdrive.controls.lib.model_intent import model_intent
from openpilot.selfdrive.controls.lib.e2e_stop import real_lead_present
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.ui.onroad.braking import braking_active, fresh_ui_message

# Same nominal offset as radard's model-to-front conversion, not a measured
# bumper offset for every vehicle. Override ModelStopFrontOffset after measuring.
DEFAULT_FRONT_OFFSET = 1.52
MAX_MODEL_AGE = .3
SMOOTH_TIME = .12
PATH_HALF_WIDTH = .9  # shared with the C3x path ribbon


def front_offset(value):
  try:
    offset = float(value)
  except (TypeError, ValueError):
    return DEFAULT_FRONT_OFFSET
  return offset if math.isfinite(offset) and 0. <= offset <= 5. else DEFAULT_FRONT_OFFSET


@dataclass(frozen=True)
class StopMarker:
  position: tuple[float, float, float] | None  # None means status without usable road geometry
  yaw: float
  distance: float | None  # remaining travel; None for a stop action without a trajectory stop
  standstill: bool = False
  label: str = 'MODEL STOP'
  held: bool = False  # display-only retention; never fed back into the planner


class ModelStop:
  def __init__(self, offset=DEFAULT_FRONT_OFFSET):
    self.offset = front_offset(offset)
    self.reset()

  def reset(self):
    self.marker = None
    self._timestamp = None
    self._distance = None
    self._started_frame = None
    self._update_time = None
    self._speed = 0.

  def update(self, sm, started_frame, now):
    if self._started_frame != started_frame:
      self.reset()
      self._started_frame = started_frame

    def fresh(service):
      return fresh_ui_message(sm, service, started_frame, now)

    if not all(fresh(s) for s in ('modelV2', 'carState', 'selfdriveState')):
      self.reset()
      return
    car = sm['carState']
    if fresh('radarState') and real_lead_present(sm['radarState']):
      self.reset()  # real-lead UI takes precedence, including over display hold
      return
    if (str(car.gearShifter) not in ('drive', 'low', 'sport') or
        not math.isfinite(car.vEgo) or car.vEgo < -.1 or
        self._update_time is not None and not 0 <= now - self._update_time <= MAX_MODEL_AGE):
      self.reset()
      return
    previous = self.marker
    speed = max(0., car.vEgo)
    dt = now - self._update_time if self._update_time is not None else 0.
    state = sm['selfdriveState']
    candidate = None
    # Ordinary/conditional ACC uses the published target, without UI smoothing.
    if getattr(state, 'enabled', False) and not state.experimentalMode:
      self._distance = self._timestamp = None
      if not fresh('longitudinalPlan'):
        self.reset()
        return
      plan = sm['longitudinalPlan']
      if plan.stopTarget.active and math.isfinite(plan.stopTarget.distance) and plan.stopTarget.distance >= 0.:
        candidate = self.predict_target(sm['modelV2'], speed, plan.stopTarget.distance)
      elif plan.e2eStopActive:
        candidate = StopMarker(None, 0., None, speed < .3, 'STOP TARGET')
    else:
      timestamp = sm.logMonoTime['modelV2'] * 1e-9
      if self._timestamp is not None and timestamp < self._timestamp:
        self.reset()  # replay seek or restarted publisher must discard held visuals
        previous = None
      if timestamp == self._timestamp and previous is not None and not previous.held and previous.label == 'MODEL STOP':
        candidate = previous
      else:
        candidate = self.predict(sm['modelV2'], speed, timestamp)

    braking = braking_active(sm, started_frame, now) and not getattr(car, 'gasPressed', False)
    # A pedal takeover must not replace the last planner target with a different
    # raw prediction. Keep ONLY its display while any fresh brake source holds.
    if (previous is not None and braking and
        (candidate is None or previous.label == 'STOP TARGET' and candidate.label != 'STOP TARGET')):
      distance = previous.distance
      if distance is not None:
        distance = max(0., distance - .5 * (self._speed + speed) * dt)
        candidate = self.predict_target(sm['modelV2'], speed, distance)
      else:
        candidate = StopMarker(None, previous.yaw, None, speed < .3)
      candidate = replace(candidate, label=previous.label, held=True)
    elif previous is not None and previous.held and not braking and candidate is not None and candidate.label == 'MODEL STOP':
      candidate = None  # release the held stop on this frame; do not relabel it

    self.marker = candidate
    if candidate is None:
      self._distance = self._timestamp = None
    self._update_time, self._speed = now, speed

  def predict_target(self, model, speed, distance):
    """Project the published planner distance without a second UI filter."""
    marker = StopMarker(None, 0., distance, speed < .3, 'STOP TARGET')
    arrays = (model.position.x, model.position.y, model.position.z, model.orientation.y, model.orientation.z)
    if any(len(a) != len(ModelConstants.T_IDXS) or not all(math.isfinite(x) for x in a) for a in arrays):
      return marker
    path = np.asarray(arrays[:3], dtype=float).T
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1))]
    if distance > arc[-1] + 1e-6:
      return marker  # no invented projection beyond the available path
    unique = np.r_[True, np.diff(arc) > 1e-6]
    point = np.array([np.interp(distance, arc[unique], path[unique, i]) for i in range(3)])
    pitch = float(np.interp(distance, arc[unique], np.unwrap(model.orientation.y)[unique]))
    yaw = float(np.interp(distance, arc[unique], np.unwrap(model.orientation.z)[unique]))
    point += self.offset * np.array([math.cos(yaw) * math.cos(pitch), math.sin(yaw) * math.cos(pitch), -math.sin(pitch)])
    return StopMarker(tuple(point), yaw, distance, speed < .3, 'STOP TARGET')

  def predict(self, model, speed, timestamp):
    intent = model_intent(model, speed)
    stop_action = bool(getattr(getattr(model, 'action', None), 'shouldStop', False))
    if not intent.valid or intent.stop_distance < 0 and not stop_action:
      return None
    standstill = speed < .3
    if intent.stop_distance < 0:
      self._distance, self._timestamp = None, timestamp
      return StopMarker(None, 0., None, standstill)
    arrays = (model.position.z, model.orientation.y, model.orientation.z)
    if any(len(a) != len(ModelConstants.T_IDXS) or not all(math.isfinite(x) for x in a) for a in arrays):
      self._distance, self._timestamp = None, timestamp
      return StopMarker(None, 0., intent.stop_distance, standstill)
    path = np.asarray([model.position.x, model.position.y, model.position.z], dtype=float).T
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1))]
    # The first qualifying stop sample is also well-defined on stationary paths,
    # where multiple samples share the same distance (np.interp would pick last).
    stop_index = int(np.searchsorted(arc, max(0., intent.stop_distance - 1e-6), side='left'))
    distance = intent.stop_distance
    dt = timestamp - self._timestamp if self._timestamp is not None else 0.
    if self._distance is not None and 0 < dt <= MAX_MODEL_AGE:
      predicted = max(0., self._distance - speed * dt)
      if abs(distance - predicted) < 3.:
        distance = predicted + dt / (SMOOTH_TIME + dt) * (distance - predicted)
    distance = min(max(0., distance), float(arc[-1]))
    if abs(distance - intent.stop_distance) < 1e-6:
      point = path[stop_index].copy()
      pitch, yaw = model.orientation.y[stop_index], model.orientation.z[stop_index]
    else:
      # Interpolate on the current path, not in screen pixels or an old ego frame.
      unique = np.r_[True, np.diff(arc) > 1e-6]
      point = np.array([np.interp(distance, arc[unique], path[unique, i]) for i in range(3)])
      pitch = float(np.interp(distance, arc[unique], np.unwrap(model.orientation.y)[unique]))
      yaw = float(np.interp(distance, arc[unique], np.unwrap(model.orientation.z)[unique]))
    # Move FORWARD from the future model reference to the future bumper. Do not
    # subtract the offset from vehicle travel: both reference and bumper travel.
    forward = np.array([math.cos(yaw) * math.cos(pitch), math.sin(yaw) * math.cos(pitch), -math.sin(pitch)])
    point += self.offset * forward
    if not np.all(np.isfinite(point)) or distance > 150.:
      return None
    self._distance, self._timestamp = distance, timestamp
    return StopMarker(tuple(point), yaw, distance, standstill)


def distance_label(marker, metric):
  if marker.distance is None:
    return 'STOP REQUEST'
  distance = marker.distance if metric else marker.distance / .3048
  prefix = '' if marker.label == 'STOP TARGET' else '~'
  return f'{prefix}{distance:.1f} {"m" if metric else "ft"}'
