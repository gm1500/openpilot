"""Animate stop-bar edges and opacity without changing the selected target."""
import math
import numpy as np

from openpilot.selfdrive.controls.lib.model_intent import model_intent
from openpilot.selfdrive.ui.onroad.braking import fresh_ui_message
from openpilot.selfdrive.ui.onroad.model_stop import MAX_MODEL_AGE

LANE_EDGES = (-1.8, 1.8)
LANE_LINE_PROB = .5
EDGE_RC = .2
FADE_RC = .1
ENGAGEMENT_RC = .25  # same status transition as the mici display


def valid_path(path):
  if path is None:
    return None
  path = np.asarray(path, dtype=float)
  if path.ndim != 2 or path.shape[1] != 3 or len(path) < 2 or not np.all(np.isfinite(path)):
    return None
  if np.any(np.diff(path[:, 0]) < -1e-6):
    return None
  return path[np.r_[True, np.diff(path[:, 0]) > 1e-6]]


def model_path(model):
  try:
    return valid_path(np.asarray((model.position.x, model.position.y, model.position.z), dtype=float).T)
  except (AttributeError, ValueError):
    return None


def lane_edges_at_stop(marker, model):
  """Lateral offsets from the existing target, never another stop estimate."""
  path = model_path(model)
  if marker.position is None or path is None:
    return LANE_EDGES
  x = marker.position[0]
  path_y = np.interp(x, path[:, 0], path[:, 1])
  lines, probabilities = getattr(model, 'laneLines', ()), getattr(model, 'laneLineProbs', ())
  edges = list(LANE_EDGES)
  for side, index in enumerate((1, 2)):
    if len(lines) <= index or len(probabilities) <= index or not probabilities[index] > LANE_LINE_PROB:
      continue
    line = lines[index]
    try:
      points = np.asarray((line.x, line.y), dtype=float).T
    except (AttributeError, ValueError):
      continue
    if (points.ndim == 2 and points.shape[1] == 2 and len(points) >= 2 and np.all(np.isfinite(points)) and
        np.all(np.diff(points[:, 0]) >= 0.)):
      edges[side] = float(np.interp(x, points[:, 0], points[:, 1]) - path_y)
  return tuple(edges) if edges[0] < edges[1] else LANE_EDGES


def stop_bar_fade_allowed(sm, started_frame, now, previous):
  """Only ordinary confirmed-target withdrawal may leave a brief fading bar."""
  # Green already has its own bounded hold/fade. A released brake-held target
  # must also disappear immediately, rather than acquiring another visual hold.
  if previous is None or previous.trajectory_only or previous.held:
    return False
  if not all(fresh_ui_message(sm, service, started_frame, now)
             for service in ('modelV2', 'carState', 'selfdriveState', 'radarState')):
    return False
  calibration = 'extrinsicsCalibration'
  if (calibration not in sm.valid or not sm.valid[calibration] or sm.recv_frame[calibration] < started_frame or
      not 0 <= now - sm.recv_time[calibration] <= .8 or
      not 0 <= round(now * 1e9) - sm.logMonoTime[calibration] <= 800_000_000 or
      str(sm[calibration].calStatus) != 'calibrated'):
    return False
  car, state, model = sm['carState'], sm['selfdriveState'], sm['modelV2']
  if (str(car.gearShifter) not in ('drive', 'low', 'sport') or not math.isfinite(car.vEgo) or car.vEgo < 0. or
      getattr(car, 'gasPressed', False)):
    return False
  if any(lead.present for lead in (sm['radarState'].leadOne, sm['radarState'].leadTwo)):
    return False  # a new real lead must never inherit a fading virtual target
  if not model_intent(model, car.vEgo).valid or model_path(model) is None:
    return False
  orientation = getattr(model, 'orientation', None)
  if any(len(getattr(orientation, axis, ())) != len(model.position.x) or
         not all(math.isfinite(v) for v in getattr(orientation, axis, ())) for axis in ('y', 'z')):
    return False
  return not (getattr(state, 'enabled', False) and not state.experimentalMode and
              not fresh_ui_message(sm, 'longitudinalPlan', started_frame, now))


class StopBarPresentation:
  """Animate lateral edges and opacity without filtering target position/range."""
  def __init__(self, frame_dt=1 / 20):
    self.frame_dt = frame_dt
    self.reset()

  def reset(self):
    self.marker = None
    self.edges = np.asarray(LANE_EDGES, dtype=float)
    self.opacity = 0.
    self._fade = 0.
    self._engagement = None
    self._edge_initialized = False
    self._last_time = None
    self._started_frame = None
    self._model_time = None

  def update(self, marker, model, *, now, started_frame, engaged, allow_fade_out=False, model_time=None):
    discontinuity = (self._started_frame != started_frame or not math.isfinite(now) or
                     self._last_time is not None and not 0 <= now - self._last_time <= MAX_MODEL_AGE or
                     model_time is not None and self._model_time is not None and model_time < self._model_time)
    if discontinuity:
      self.reset()
    if not math.isfinite(now):
      return
    dt = self.frame_dt if self._last_time is None else max(0., now - self._last_time)
    self._started_frame, self._last_time, self._model_time = started_frame, now, model_time
    target_opacity = 1. if engaged else .6
    if self._engagement is None:
      self._engagement = target_opacity
    else:
      self._engagement += dt / (ENGAGEMENT_RC + dt) * (target_opacity - self._engagement)

    if marker is not None:
      self.marker = marker  # preserve the exact current authoritative marker
      target_edges = np.asarray(lane_edges_at_stop(marker, model))
      if not self._edge_initialized:
        self.edges = target_edges
        self._edge_initialized = True
      else:
        self.edges += dt / (EDGE_RC + dt) * (target_edges - self.edges)
      self._fade += dt / (FADE_RC + dt) * (1. - self._fade)
    elif (allow_fade_out and not discontinuity and self.marker is not None and
          not self.marker.trajectory_only and not self.marker.held):
      self._fade *= 1. - dt / (FADE_RC + dt)
      if self._fade < .01:
        self.marker = None
    else:
      self.marker = None
    if self.marker is None:
      self._fade = 0.
      self._edge_initialized = False
    self.opacity = 0. if self.marker is None else self._fade * self._engagement * self.marker.opacity
