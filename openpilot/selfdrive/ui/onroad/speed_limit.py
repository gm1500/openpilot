import pyray as rl

from openpilot.common.constants import CV
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached


def draw_speed_limit(rect: rl.Rectangle, speed_limit: float | None, set_speed: float | None, is_metric: bool, set_color: rl.Color) -> None:
  """North American-style map sign, with the cruise target outside underneath."""
  font = gui_app.font(FontWeight.BOLD)
  medium = gui_app.font(FontWeight.MEDIUM)
  scale = rect.width / 180
  black = rl.Color(20, 22, 24, 255)
  rl.draw_rectangle_rounded(rect, 0.12, 8, rl.Color(250, 250, 247, 255))
  inset = 5 * scale
  border = rl.Rectangle(rect.x + inset, rect.y + inset, rect.width - 2 * inset, rect.height - 2 * inset)
  rl.draw_rectangle_rounded_lines_ex(border, 0.08, 8, 3 * scale, black)

  def text(value: str, y: float, height: float, size: float, color: rl.Color, bold: bool = True):
    face = font if bold else medium
    size *= scale
    bounds = measure_text_cached(face, value, size)
    size *= min(1., (rect.width - 20 * scale) / max(bounds.x, 1), height * scale / max(bounds.y, 1))
    bounds = measure_text_cached(face, value, size)
    pos = rl.Vector2(rect.x + (rect.width - bounds.x) / 2, rect.y + y * scale + (height * scale - bounds.y) / 2)
    rl.draw_text_ex(face, value, pos, size, 0, color)

  text("MAXIMUM" if is_metric else "SPEED LIMIT", 14, 34, 27, black)
  conversion = CV.MS_TO_KPH if is_metric else CV.MS_TO_MPH
  value = "–" if speed_limit is None else str(round(speed_limit * conversion))
  text(value, 48, 113, 104, black)
  text("km/h" if is_metric else "mph", 164, 26, 22, black, bold=False)

  # A separate dark strip keeps the actual cruise target readable against the camera.
  subtext = rl.Rectangle(rect.x, rect.y + rect.height + 7 * scale, rect.width, 48 * scale)
  rl.draw_rectangle_rounded(subtext, 0.2, 8, rl.Color(0, 0, 0, 166))
  text(f"SET {'–' if set_speed is None else round(set_speed)}", 212, 45, 35, set_color)
  text("(c) OpenStreetMap", 260, 20, 13, rl.Color(230, 230, 230, 255), bold=False)
