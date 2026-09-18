import pyray as rl
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget


class LanePolicyIcon(Widget):
  """Compact on-road selector for the lane-centering policy."""

  def __init__(self, button_size: int, wheel_icon_size: int):
    super().__init__()
    self._lane_policy_enabled = True
    self._wheel = gui_app.texture('icons/chffr_wheel.png', wheel_icon_size, wheel_icon_size)
    self._background = rl.Color(0, 0, 0, 166)
    self._enabled_color = rl.Color(128, 216, 166, 255)
    self._disabled_color = rl.Color(255, 255, 255, 255)

  def _update_state(self) -> None:
    self._lane_policy_enabled = ui_state.lane_policy_enabled

  def _handle_mouse_release(self, mouse_pos) -> None:
    super()._handle_mouse_release(mouse_pos)
    # UIState persists the value and updates its in-memory copy before the
    # parameter refresh worker runs, so consecutive taps always alternate.
    ui_state.set_lane_policy_enabled(not ui_state.lane_policy_enabled)

  def _render(self, rect: rl.Rectangle) -> None:
    center_x = int(rect.x + rect.width / 2)
    center_y = int(rect.y + rect.height / 2)
    icon_color = self._enabled_color if self._lane_policy_enabled else self._disabled_color
    tint = rl.Color(icon_color.r, icon_color.g, icon_color.b, 180 if self.is_pressed else 255)

    rl.draw_circle(center_x, center_y, rect.width / 2, self._background)

    # The wheel asset matches the stock control; the two tapered side marks
    # identify this as the lane-centering selector.
    lane_top = center_y - rect.height * 0.27
    lane_bottom = center_y + rect.height * 0.27
    left_top = center_x - rect.width * 0.28
    left_bottom = center_x - rect.width * 0.34
    right_top = center_x + rect.width * 0.28
    right_bottom = center_x + rect.width * 0.34
    thickness = rect.width * 0.035
    rl.draw_line_ex(rl.Vector2(left_top, lane_top), rl.Vector2(left_bottom, lane_bottom), thickness, tint)
    rl.draw_line_ex(rl.Vector2(right_top, lane_top), rl.Vector2(right_bottom, lane_bottom), thickness, tint)

    rl.draw_texture_ex(self._wheel,
                       rl.Vector2(center_x - self._wheel.width / 2, center_y - self._wheel.height / 2),
                       0.0, 1.0, tint)
