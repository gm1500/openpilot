import math
import pyray as rl

from openpilot.common.constants import CV
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.selfdrive.ui.ui_state import ui_state


def draw_speed_limit(rect: rl.Rectangle, speed_limit: float | None, set_speed: float | None, is_metric: bool, set_color: rl.Color,
                     state: str = 'unsupported', pending_elapsed: float | None = None) -> None:
  """North American-style map sign, with the cruise target outside underneath."""
  font = gui_app.font(FontWeight.BOLD)
  medium = gui_app.font(FontWeight.MEDIUM)
  scale = rect.width / 180
  black = rl.Color(20, 22, 24, 255)
  # A gentle 1 Hz green wash while qualifying an automatic change, never a
  # flashing number. Returning to None restores white immediately.
  pulse = 0. if pending_elapsed is None else 0.45 * (1 - math.cos(2 * math.pi * pending_elapsed)) / 2
  background = rl.Color(round(250 - 180 * pulse), 250, round(247 - 140 * pulse), 255)
  rl.draw_rectangle_rounded(rect, 0.12, 8, background)
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
  text(value, 48, 108, 104, black)
  text("km/h" if is_metric else "mph", 158, 34, 30, black, bold=False)

  # A separate dark strip keeps the actual cruise target readable against the camera.
  subtext = rl.Rectangle(rect.x, rect.y + rect.height + 7 * scale, rect.width, 48 * scale)
  rl.draw_rectangle_rounded(subtext, 0.2, 8, rl.Color(0, 0, 0, 166))
  text('–' if set_speed is None else str(round(set_speed)), 212, 45, 35, set_color)
  labels = {'active': 'MAP ON', 'armed': 'MAP READY', 'waiting': 'MAP WAIT', 'paused': 'MAP HOLD', 'off': 'MAP OFF'}
  if state in labels:
    color = rl.Color(13, 248, 122, 255) if state in ('active', 'armed') else rl.Color(255, 200, 96, 255) if state != 'off' else rl.LIGHTGRAY
    rl.draw_rectangle_rounded_lines_ex(border, 0.08, 8, 4 * scale, color)
    status = rl.Rectangle(rect.x, rect.y + 260 * scale, rect.width, 29 * scale)
    rl.draw_rectangle_rounded(status, 0.2, 8, rl.Color(0, 0, 0, 190))
    text(labels[state], 261, 27, 21, color)


class SpeedLimitButton(Widget):
  def __init__(self):
    super().__init__()
    self.set_speed: float | None = None
    self.set_color = rl.WHITE
    self.show_sign = True
    self._pending_since: float | None = None
    self._click_delay = 0.2  # consume release before the camera's tap handler
    self.set_enabled(lambda: self.show_sign and ui_state.map_cruise_supported)
    self.set_click_callback(lambda: ui_state.set_map_cruise_enabled(not ui_state.map_cruise_enabled))

  def _render(self, rect: rl.Rectangle):
    # Still run Widget.render while covered by an alert so it cancels touches.
    if not self.show_sign:
      self._click_release_time = None
      self._pending_since = None
      return
    state = ui_state.map_cruise_state if ui_state.map_cruise_enabled else 'off'
    if not ui_state.map_cruise_supported:
      state = 'unsupported'
    pending = ui_state.map_cruise_pending and ui_state.map_cruise_enabled and ui_state.map_cruise_supported
    now = rl.get_time()
    if not pending:
      self._pending_since = None
    elif self._pending_since is None:
      self._pending_since = now
    elapsed = None if self._pending_since is None else now - self._pending_since
    draw_speed_limit(rect, ui_state.speed_limit, self.set_speed, ui_state.is_metric, self.set_color, state, elapsed)
