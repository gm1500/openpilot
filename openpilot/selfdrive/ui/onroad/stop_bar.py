"""Opaque, rounded road-plane stop bar, using the ordinary lead colour response."""
import math
import numpy as np

from openpilot.selfdrive.ui.onroad.lead_geometry import lead_fill_alpha, rounded_polygon
from openpilot.selfdrive.ui.onroad.model_stop import PATH_HALF_WIDTH, distance_label

STOP_BAR_WIDTH = 450.  # ~2.5 times the close lead triangle's 182 px outer width


def stop_bar_geometry(marker, camera_height, project, rect):
  """Fixed-width road-aligned bar; retain perspective and close-stop visibility."""
  scale = min(rect.width / 1080., rect.height / 540.)
  points = None
  if (marker.position is not None and all(math.isfinite(v) for v in (*marker.position, marker.yaw, camera_height)) and
      marker.position[0] >= .5 and .5 <= camera_height <= 3.):
    point = np.asarray(marker.position) + (0., 0., camera_height)
    forward = np.array([math.cos(marker.yaw), math.sin(marker.yaw), 0.])
    lateral = np.array([-math.sin(marker.yaw), math.cos(marker.yaw), 0.]) * PATH_HALF_WIDTH
    near = point - min(6., .25 * marker.position[0]) * forward
    projected = [project(*p) for p in (point - lateral, point + lateral, near + lateral, near - lateral)]
    if all(p is not None and all(math.isfinite(v) for v in p) for p in projected):
      points = np.asarray(projected, dtype=float)
      far, close = points[:2], points[[3, 2]]
      depth = np.linalg.norm(close.mean(axis=0) - far.mean(axis=0))
      if depth > 1e-6 and np.ptp(points[:, 0]) > 1e-6:
        points[[3, 2]] = far + (close - far) * (2 * np.clip(depth, 3 * scale, 12 * scale)) / depth
      else:
        points = None

  docked = points is None
  if docked:
    # The same marker remains visible, never a substitute triangular lead/card.
    x, y = rect.x + rect.width / 2, rect.y + rect.height - 72 * scale
    w, h = min(STOP_BAR_WIDTH, rect.width * .65), 24 * scale
    points = np.array([(x - w / 2 + 3 * scale, y), (x + w / 2 - 3 * scale, y),
                       (x + w / 2, y + h), (x - w / 2, y + h)])
  # Keep the displayed width fixed with distance, including the docked bar.
  # Preserve its road-aligned profile and the lead marker's edge retention.
  center = points.mean(axis=0)
  points[:, 0] = center[0] + (points[:, 0] - center[0]) * min(STOP_BAR_WIDTH, rect.width * .65) / np.ptp(points[:, 0])
  low = np.array([rect.x + 16 * scale, rect.y + 16 * scale])
  high = np.array([rect.x + rect.width - 16 * scale, rect.y + rect.height - 60 * scale])
  shift = np.maximum(low - points.min(axis=0), 0.) + np.minimum(high - points.max(axis=0), 0.)
  return points + shift, docked or bool(np.any(shift))


def draw_stop_bar(marker, speed, camera_height, project, rect, font, metric):
  import pyray as rl
  from openpilot.system.ui.lib.text_measure import measure_text_cached

  points, docked = stop_bar_geometry(marker, camera_height, project, rect)
  scale = min(rect.width / 1080., rect.height / 540.)
  # Identical to a stationary ordinary lead, without making the road visible
  # through the bar. Red is composited over the solid yellow base.
  alpha = lead_fill_alpha(marker.distance, -max(0., speed)) if marker.distance is not None else 0
  yellow = np.array([218., 202., 37.])
  red = np.array([201., 34., 49.])
  rgb = np.rint(yellow + alpha / 255. * (red - yellow)).astype(int)
  outline = rounded_polygon(points, 4 * scale)
  rl.draw_triangle_fan(outline, len(outline), rl.Color(218, 202, 37, 255))
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
