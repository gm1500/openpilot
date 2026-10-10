"""Opaque stop bars: green trajectory cue, ordinary lead colours for stop/hold."""
import math
import numpy as np

from openpilot.selfdrive.ui.onroad.lead_geometry import lead_fill_alpha, rounded_polygon
from openpilot.selfdrive.ui.onroad.model_stop import distance_label

STOP_BAR_WIDTH = 450.  # ~2.5 times the close lead triangle's 182 px outer width
STOP_BAR_FAR_WIDTH_RATIO = .8  # foreshortened far edge of the ground-plane profile


def stop_bar_geometry(marker, camera_height, project, rect):
  """Project the stopping point, then draw a level ground-perspective trapezoid."""
  scale = min(rect.width / 1080., rect.height / 540.)
  x, y = rect.x + rect.width / 2, rect.y + rect.height - 72 * scale
  h = 24 * scale
  docked = True
  if (marker.position is not None and all(math.isfinite(v) for v in (*marker.position, marker.yaw, camera_height)) and
      marker.position[0] >= .5 and .5 <= camera_height <= 3.):
    point = np.asarray(marker.position) + (0., 0., camera_height)
    forward = np.array([math.cos(marker.yaw), math.sin(marker.yaw), 0.])
    near = point - min(6., .25 * marker.position[0]) * forward
    projected = [project(*p) for p in (point, near)]
    if all(p is not None and all(math.isfinite(v) for v in p) for p in projected):
      depth = np.linalg.norm(np.asarray(projected[1]) - projected[0])
      if depth > 1e-6:
        x, y = projected[0]
        h = 2 * np.clip(depth, 3 * scale, 12 * scale)
        docked = False

  # Road curvature and camera roll move the anchor, never rotate the bar.
  # Use the same fixed width and doubled thickness for docked stops.
  w = min(STOP_BAR_WIDTH, rect.width * .65) * (.5 if marker.trajectory_only else 1.)
  far_w = w * STOP_BAR_FAR_WIDTH_RATIO
  points = np.array([(x - far_w / 2, y), (x + far_w / 2, y),
                     (x + w / 2, y + h), (x - w / 2, y + h)])
  low = np.array([rect.x + 16 * scale, rect.y + 16 * scale])
  high = np.array([rect.x + rect.width - 16 * scale, rect.y + rect.height - 60 * scale])
  shift = np.maximum(low - points.min(axis=0), 0.) + np.minimum(high - points.max(axis=0), 0.)
  return points + shift, docked or bool(np.any(shift))


def draw_stop_bar(marker, speed, camera_height, project, rect, font, metric):
  import pyray as rl
  from openpilot.system.ui.lib.text_measure import measure_text_cached

  points, docked = stop_bar_geometry(marker, camera_height, project, rect)
  scale = min(rect.width / 1080., rect.height / 540.)
  # Trajectory-only targets stay green. Stop/hold targets retain the regular
  # lead's yellow outline and dynamic distance/closing-speed fill.
  alpha = lead_fill_alpha(marker.distance, -max(0., speed)) if marker.distance is not None and not marker.trajectory_only else 0
  base = np.array([23., 134., 68.]) if marker.trajectory_only else np.array([218., 202., 37.])
  red = np.array([201., 34., 49.])
  rgb = np.rint(base + alpha / 255. * (red - base)).astype(int)
  outline = rounded_polygon(points, 4 * scale)
  rl.draw_triangle_fan(outline, len(outline), rl.Color(*map(int, base), 255))
  if alpha:
    center = points.mean(axis=0)
    size = np.ptp(points, axis=0)
    inner = center + (points - center) * np.maximum(.25, 1 - 4 * scale / size)
    outline = rounded_polygon(inner, 2 * scale)
    rl.draw_triangle_fan(outline, len(outline), rl.Color(*map(int, rgb), 255))

  suffix = '  BRAKE HOLD' if marker.held else '  OFF SCREEN' if docked and not marker.standstill else ''
  label = f'{marker.label}  {distance_label(marker, metric)}{suffix}'
  font_size = 22 * scale
  bounds = measure_text_cached(font, label, font_size)
  font_size *= min(1., (rect.width - 40 * scale) / max(bounds.x, 1.))
  bounds = measure_text_cached(font, label, font_size)
  width, height = bounds.x + 20 * scale, bounds.y + 12 * scale
  x = np.clip(points[:, 0].mean() - width / 2, rect.x + 8 * scale, rect.x + rect.width - width - 8 * scale)
  y = min(points[:, 1].max() + 8 * scale, rect.y + rect.height - height - 8 * scale)
  rl.draw_rectangle_rounded(rl.Rectangle(x, y, width, height), .2, 8, rl.Color(0, 0, 0, 190))
  rl.draw_text_ex(font, label, (x + 10 * scale, y + 6 * scale), font_size, 0, rl.WHITE)
