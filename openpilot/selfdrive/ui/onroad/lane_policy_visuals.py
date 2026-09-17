"""Pure interaction and visual-state helpers for the on-road lane-policy UI."""

from collections.abc import Sequence


def lane_policy_visual_active(lane_policy_enabled: bool, openpilot_engaged: bool) -> bool:
  """The lane-color scheme applies only while a selected policy is engaged."""
  return bool(lane_policy_enabled and openpilot_engaged)


def point_in_polygon(point: tuple[float, float], polygon: Sequence[Sequence[float]]) -> bool:
  """Return whether point is inside a screen-space polygon, including its edges."""
  if len(polygon) < 3:
    return False

  x, y = point
  inside = False
  previous = polygon[-1]
  for current in polygon:
    x1, y1 = float(previous[0]), float(previous[1])
    x2, y2 = float(current[0]), float(current[1])
    on_segment = (min(x1, x2) <= x <= max(x1, x2) and
                  min(y1, y2) <= y <= max(y1, y2) and
                  abs((x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)) < 1e-6)
    if on_segment:
      return True
    if (y1 > y) != (y2 > y):
      intersection_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
      if x < intersection_x:
        inside = not inside
    previous = current
  return inside
