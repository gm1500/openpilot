"""Display-only model stop prediction in the calibrated model frame."""
import math
from dataclasses import dataclass

import numpy as np

from openpilot.selfdrive.controls.lib.conditional_experimental import model_intent
from openpilot.selfdrive.modeld.constants import ModelConstants

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


class ModelStop:
  def __init__(self, offset=DEFAULT_FRONT_OFFSET):
    self.offset = front_offset(offset)
    self.reset()

  def reset(self):
    self.marker = None
    self._timestamp = None
    self._distance = None
    self._started_frame = None

  def update(self, sm, started_frame, now):
    if self._started_frame != started_frame:
      self.reset()
      self._started_frame = started_frame

    def fresh(service):
      return (sm.valid[service] and sm.recv_frame[service] >= started_frame and
              0 <= now - sm.recv_time[service] <= MAX_MODEL_AGE and
              0 <= now - sm.logMonoTime[service] * 1e-9 <= MAX_MODEL_AGE)

    if not all(fresh(s) for s in ('modelV2', 'carState', 'selfdriveState')):
      self.reset()
      return
    car = sm['carState']
    if (str(car.gearShifter) not in ('drive', 'low', 'sport') or
        not math.isfinite(car.vEgo) or car.vEgo < -.1):
      self.reset()
      return
    # While ordinary ACC is engaged, draw the actual planner target. Raw model
    # predictions remain available in manual/full E2E mode with their own label.
    state = sm['selfdriveState']
    if getattr(state, 'enabled', False) and not state.experimentalMode:
      self.reset()
      if 'longitudinalPlan' not in sm.valid or not fresh('longitudinalPlan'):
        return
      plan = sm['longitudinalPlan']
      if plan.stopTarget.active and math.isfinite(plan.stopTarget.distance) and plan.stopTarget.distance >= 0.:
        self.marker = self.predict_target(sm['modelV2'], max(0., car.vEgo), plan.stopTarget.distance)
      elif plan.e2eStopActive:
        self.marker = StopMarker(None, 0., None, car.vEgo < .3, 'STOP TARGET')
      return
    timestamp = sm.logMonoTime['modelV2'] * 1e-9
    if timestamp == self._timestamp:
      return  # render rate and camera zoom must not advance the distance filter
    if self._timestamp is not None and timestamp < self._timestamp:
      self.reset()  # replay seek or restarted model publisher
    self.marker = self.predict(sm['modelV2'], max(0., car.vEgo), timestamp)
    if self.marker is None:
      self.reset()

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


def draw_stop_status(marker, rect, font, metric):
  """A screen-fixed flag, explicitly separate from a projected road location."""
  import pyray as rl

  scale = min(1., rect.width / 1000., rect.height / 500.)
  width, height = 280 * scale, 112 * scale
  x = rect.x + rect.width * .66
  x = min(x, rect.x + rect.width - width - 24 * scale)
  y = rect.y + rect.height - height - 80 * scale
  cyan, shadow = rl.Color(80, 220, 255, 255), rl.Color(8, 24, 30, 225)
  flag = rl.Rectangle(x, y, width, height)
  rl.draw_rectangle_rounded(flag, .12, 8, shadow)
  rl.draw_rectangle_rounded_lines_ex(flag, .12, 8, 2 * scale, cyan)
  rl.draw_text_ex(font, marker.label, (x + 12 * scale, y + 10 * scale), 22 * scale, 0, cyan)
  rl.draw_text_ex(font, distance_label(marker, metric), (x + 12 * scale, y + 40 * scale), 26 * scale, 0, rl.WHITE)
  if marker.position is None:
    caption = 'STOPPED / NO POSITION' if marker.standstill else 'POSITION UNAVAILABLE'
  else:
    caption = 'STOPPED / OFF SCREEN' if marker.standstill else 'OFF SCREEN'
  rl.draw_text_ex(font, caption, (x + 12 * scale, y + 80 * scale), 18 * scale, 0, cyan)


def draw_stop_flag(marker, camera_height, project, rect, font, metric):
  """Use exact road projection when visible, otherwise a labelled status flag."""
  import pyray as rl

  def fallback():
    draw_stop_status(marker, rect, font, metric)

  if marker.position is None:
    return fallback()
  x, y, z = marker.position
  # The path ribbon uses +/- y offsets in the model frame; +y is screen-right.
  # Shift only the display anchor, retaining the predicted bumper and travel.
  y += PATH_HALF_WIDTH
  if x < .5 or not math.isfinite(camera_height) or not .5 <= camera_height <= 3.:
    return fallback()
  ground_z = z + camera_height
  foot = project(x, y, ground_z)
  top = project(x, y, ground_z - 1.)
  if foot is None or top is None:
    return fallback()
  if not (rect.x + 8 <= foot[0] <= rect.x + rect.width - 8 and rect.y + 8 <= foot[1] <= rect.y + rect.height - 8):
    return fallback()
  # A readable flag on a one-metre virtual post; its foot remains geometrically exact.
  scale = min(1.15, max(.7, (foot[1] - top[1]) / 90.))
  height = min(180., max(65., foot[1] - top[1]))
  pole_top = (foot[0], foot[1] - height)
  width, flag_height = 182 * scale, 66 * scale
  flag_x = pole_top[0] + 4
  if flag_x + width > rect.x + rect.width - 8:
    flag_x = pole_top[0] - width - 4
  flag_y = pole_top[1]
  if flag_y < rect.y + 8 or flag_x < rect.x + 8:
    return fallback()
  cyan, shadow = rl.Color(80, 220, 255, 255), rl.Color(8, 24, 30, 225)
  rl.draw_line_ex(foot, pole_top, 7 * scale, shadow)
  rl.draw_line_ex(foot, pole_top, 3 * scale, cyan)
  # A short crossbar locates the predicted bumper on the road surface.
  lateral = (-math.sin(marker.yaw) * .45, math.cos(marker.yaw) * .45)
  left = project(x + lateral[0], y + lateral[1], ground_z)
  right = project(x - lateral[0], y - lateral[1], ground_z)
  if left is not None and right is not None:
    rl.draw_line_ex(left, right, 6 * scale, shadow)
    rl.draw_line_ex(left, right, 3 * scale, cyan)
  rl.draw_circle_v(foot, 4 * scale, cyan)
  flag = rl.Rectangle(flag_x, flag_y, width, flag_height)
  rl.draw_rectangle_rounded(flag, .12, 6, shadow)
  rl.draw_rectangle_rounded_lines_ex(flag, .12, 6, 2 * scale, cyan)
  label = distance_label(marker, metric)
  rl.draw_text_ex(font, marker.label, (flag_x + 10 * scale, flag_y + 7 * scale), 22 * scale, 0, cyan)
  rl.draw_text_ex(font, label, (flag_x + 10 * scale, flag_y + 34 * scale), 24 * scale, 0, rl.WHITE)
