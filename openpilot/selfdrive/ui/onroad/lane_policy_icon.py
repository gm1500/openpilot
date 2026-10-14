import pyray as rl
from openpilot.selfdrive.ui.onroad.lane_icon import LANE_GREEN, draw_lane_icon_frame
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget


class LanePolicyIcon(Widget):
  """Compact on-road selector for the lane-centering policy."""

  def __init__(self, wheel_icon_size: int):
    super().__init__()
    self._wheel = gui_app.texture('icons/chffr_wheel.png', wheel_icon_size, wheel_icon_size)
    self._enabled_color = LANE_GREEN
    self._disabled_color = rl.Color(255, 255, 255, 255)

  def _handle_mouse_release(self, mouse_pos) -> None:
    super()._handle_mouse_release(mouse_pos)
    # UIState persists the value and updates its in-memory copy before the
    # parameter refresh worker runs, so consecutive taps always alternate.
    ui_state.set_lane_policy_enabled(not ui_state.lane_policy_enabled)

  def _render(self, rect: rl.Rectangle) -> None:
    center_x = int(rect.x + rect.width / 2)
    center_y = int(rect.y + rect.height / 2)
    icon_color = self._enabled_color if ui_state.lane_policy_enabled else self._disabled_color
    tint = rl.Color(icon_color.r, icon_color.g, icon_color.b, 180 if self.is_pressed else 255)

    draw_lane_icon_frame(rect, tint)

    rl.draw_texture_ex(self._wheel,
                       rl.Vector2(center_x - self._wheel.width / 2, center_y - self._wheel.height / 2),
                       0.0, 1.0, tint)
