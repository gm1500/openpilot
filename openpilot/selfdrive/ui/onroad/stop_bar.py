"""Lane-wide ground bars with Batmobile's approach and stop/hold colours."""
import math
import numpy as np

from openpilot.selfdrive.ui.onroad.lead_geometry import lead_fill_alpha
from openpilot.selfdrive.ui.onroad.model_stop import distance_label
from openpilot.selfdrive.ui.onroad.stop_bar_animation import LANE_EDGES, valid_path

STOP_BAR_LENGTH = 10.  # projected pixels at the base C3X drawing scale
STOP_DOCK_WIDTH = 450.


def stop_bar_geometry(marker, camera_height, project, rect, *, path=None, edges=LANE_EDGES):
  """Project a lane-wide ground quad at the marker's unchanged model position."""
  scale = min(rect.width / 1080., rect.height / 540.)
  w = min(STOP_DOCK_WIDTH, rect.width * .65)
  x, y = rect.x + rect.width / 2, rect.y + rect.height - 72 * scale
  h = STOP_BAR_LENGTH * scale
  points = np.array([(x - w / 2, y), (x + w / 2, y), (x + w / 2, y + h), (x - w / 2, y + h)])
  docked = True
  if (marker.position is not None and all(math.isfinite(v) for v in (*marker.position, marker.yaw, camera_height, *edges)) and
      marker.position[0] >= .5 and .5 <= camera_height <= 3. and edges[0] < edges[1]):
    point = np.asarray(marker.position, dtype=float)
    near = point - min(6., .25 * point[0]) * np.array([math.cos(marker.yaw), math.sin(marker.yaw), 0.])
    path = valid_path(path)
    if path is not None:
      near[0] = point[0] - min(6., .25 * point[0])
      for axis in (1, 2):
        near[axis] = point[axis] + np.interp(near[0], path[:, 0], path[:, axis]) - np.interp(point[0], path[:, 0], path[:, axis])
    corners = (point + (0., edges[0], camera_height), point + (0., edges[1], camera_height),
               near + (0., edges[1], camera_height), near + (0., edges[0], camera_height))
    projected = [project(*corner) for corner in corners]
    if all(p is not None and all(math.isfinite(v) for v in p) for p in projected):
      bar = np.asarray(projected, dtype=float)
      far, close = bar[[0, 1]], bar[[3, 2]]
      length = np.linalg.norm(close.mean(axis=0) - far.mean(axis=0))
      if length > 1e-6:
        bar[[3, 2]] = far + (close - far) * np.clip(length, 3 * scale, STOP_BAR_LENGTH * scale) / length
        if np.ptp(bar[:, 0]) <= rect.width - 32 * scale:
          points, docked = bar, False

  # Retain the close/held-stop dock when the target leaves the camera view.
  low = np.array([rect.x + 16 * scale, rect.y + 16 * scale])
  high = np.array([rect.x + rect.width - 16 * scale, rect.y + rect.height - 60 * scale])
  shift = np.maximum(low - points.min(axis=0), 0.) + np.minimum(high - points.max(axis=0), 0.)
  return points + shift, docked or bool(np.any(shift))


def _flat_fan(points):
  points = np.asarray(points)
  following = np.roll(points, -1, axis=0)
  if np.sum(points[:, 0] * following[:, 1] - following[:, 0] * points[:, 1]) > 0:
    points = points[::-1]
  return [tuple(points.mean(axis=0)), *map(tuple, points), tuple(points[0])]


def _inset_quad(points, amount):
  """Inset along the actual edges so a rolled/skewed fill stays in its outline."""
  directions = np.roll(points, -1, axis=0) - points
  lengths = np.linalg.norm(directions, axis=1)
  center = points.mean(axis=0)
  if np.any(lengths < 1e-6):
    return center + .25 * (points - center)
  normals = np.column_stack((-directions[:, 1], directions[:, 0])) / lengths[:, None]
  if np.sum(normals * (center - points)) < 0:
    normals *= -1
  limits = np.sum(normals * points, axis=1)
  amount = min(amount, .75 * np.min(np.sum(normals * center, axis=1) - limits))
  inner = []
  for i in range(4):
    matrix = normals[[i - 1, i]]
    if abs(np.linalg.det(matrix)) < 1e-6:
      return center + .25 * (points - center)
    inner.append(np.linalg.solve(matrix, limits[[i - 1, i]] + amount))
  return np.asarray(inner)


def draw_stop_bar(marker, speed, camera_height, project, rect, font, metric, *, path=None, edges=LANE_EDGES, opacity=None):
  import raylib_nb as rl
  from openpilot.system.ui.lib.text_measure import measure_text_cached

  points, docked = stop_bar_geometry(marker, camera_height, project, rect, path=path, edges=edges)
  if marker.trajectory_only and docked:
    return  # an approach cue belongs down the road, never in the near-stop dock
  scale = min(rect.width / 1080., rect.height / 540.)
  opacity = int(round(255 * np.clip(marker.opacity if opacity is None else opacity, 0., 1.)))
  if opacity <= 0:
    return
  # Trajectory-only targets stay green. Stop/hold targets retain the regular
  # lead's yellow outline and dynamic distance/closing-speed fill.
  alpha = lead_fill_alpha(marker.distance, -max(0., speed)) if marker.distance is not None and not marker.trajectory_only else 0
  base = np.array([23., 134., 68.]) if marker.trajectory_only else np.array([218., 202., 37.])
  red = np.array([201., 34., 49.])
  rgb = np.rint(base + alpha / 255. * (red - base)).astype(int)
  outline = _flat_fan(points)
  rl.draw_triangle_fan(outline, len(outline), rl.Color(*map(int, base), opacity))
  if alpha:
    inner = _inset_quad(points, 2 * scale)
    outline = _flat_fan(inner)
    rl.draw_triangle_fan(outline, len(outline), rl.Color(*map(int, rgb), opacity))

  suffix = '  BRAKE HOLD' if marker.held else '  OFF SCREEN' if docked and not marker.standstill else ''
  label = f'{marker.label}  {distance_label(marker, metric)}{suffix}'
  font_size = 22 * scale
  bounds = measure_text_cached(font, label, font_size)
  font_size *= min(1., (rect.width - 40 * scale) / max(bounds.x, 1.))
  bounds = measure_text_cached(font, label, font_size)
  width, height = bounds.x + 20 * scale, bounds.y + 12 * scale
  x = np.clip(points[:, 0].mean() - width / 2, rect.x + 8 * scale, rect.x + rect.width - width - 8 * scale)
  y = min(points[:, 1].max() + 8 * scale, rect.y + rect.height - height - 8 * scale)
  rl.draw_rectangle_rounded(rl.Rectangle(x, y, width, height), .2, 8, rl.Color(0, 0, 0, round(190 * opacity / 255)))
  rl.draw_text_ex(font, label, rl.Vector2(x + 10 * scale, y + 6 * scale), font_size, 0, rl.Color(255, 255, 255, opacity))
