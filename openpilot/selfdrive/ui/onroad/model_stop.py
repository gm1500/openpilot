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
  position: tuple[float, float, float]  # future bumper in the current model frame
  yaw: float
  distance: float  # remaining travel, not distance from camera to future bumper


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
    state, car = sm['selfdriveState'], sm['carState']
    if (not (state.experimentalMode or state.conditionalExperimental) or str(car.gearShifter) not in ('drive', 'low') or
        not math.isfinite(car.vEgo) or car.vEgo < -.1):
      self.reset()
      return
    timestamp = sm.logMonoTime['modelV2'] * 1e-9
    if timestamp == self._timestamp:
      return  # render rate and camera zoom must not advance the distance filter
    if self._timestamp is not None and timestamp < self._timestamp:
      self.reset()  # replay seek or restarted model publisher
    self.marker = self.predict(sm['modelV2'], max(0., car.vEgo), timestamp)
    if self.marker is None:
      self.reset()

  def predict(self, model, speed, timestamp):
    intent = model_intent(model, speed)
    if not intent.valid or intent.stop_distance < 0:
      return None
    arrays = (model.position.z, model.orientation.y, model.orientation.z)
    if any(len(a) != len(ModelConstants.T_IDXS) or not all(math.isfinite(x) for x in a) for a in arrays):
      return None
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
    if not np.all(np.isfinite(point)) or point[0] < .5 or distance > 150.:
      return None
    self._distance, self._timestamp = distance, timestamp
    return StopMarker(tuple(point), yaw, distance)


def draw_stop_flag(marker, camera_height, project, rect, font, metric):
  """Project the foot onto the road; never clamp it to a misleading screen location."""
  import pyray as rl

  x, y, z = marker.position
  # The path ribbon uses +/- y offsets in the model frame; +y is screen-right.
  # Shift only the display anchor, retaining the predicted bumper and travel.
  y += PATH_HALF_WIDTH
  if not math.isfinite(camera_height) or not .5 <= camera_height <= 3.:
    return
  ground_z = z + camera_height
  foot = project(x, y, ground_z)
  top = project(x, y, ground_z - 1.)
  if foot is None or top is None:
    return
  if not (rect.x + 8 <= foot[0] <= rect.x + rect.width - 8 and rect.y + 8 <= foot[1] <= rect.y + rect.height - 8):
    return
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
    return
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
  distance = marker.distance if metric else marker.distance / .3048
  label = f'~{distance:.1f} {"m" if metric else "ft"}'
  rl.draw_text_ex(font, 'MODEL STOP', (flag_x + 10 * scale, flag_y + 7 * scale), 22 * scale, 0, cyan)
  rl.draw_text_ex(font, label, (flag_x + 10 * scale, flag_y + 34 * scale), 24 * scale, 0, rl.WHITE)
