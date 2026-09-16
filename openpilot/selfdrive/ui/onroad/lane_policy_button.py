import pyray as rl
from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget


class LanePolicyButton(Widget):
  """On-road, per-drive selector for the strict lane-centering policy."""
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._lane_policy_enabled = False
    self._lane_policy_active = False
    self._font = gui_app.font(FontWeight.SEMI_BOLD)
    self._on = rl.Color(128, 216, 166, 255)
    self._off = rl.Color(166, 166, 166, 255)
    self._fallback = rl.Color(218, 111, 37, 255)
    self._background = rl.Color(0, 0, 0, 166)

  def _update_state(self) -> None:
    self._lane_policy_enabled = ui_state.lane_policy_enabled
    self._lane_policy_active = ui_state.lane_policy_active

  def _handle_mouse_release(self, mouse_pos) -> None:
    super()._handle_mouse_release(mouse_pos)
    self._lane_policy_enabled = not self._lane_policy_enabled
    self._params.put_bool("LanePolicyEnabled", self._lane_policy_enabled)

  def _render(self, rect: rl.Rectangle) -> None:
    if not self._lane_policy_enabled:
      color, label = self._off, "LANE: E2E"
    elif self._lane_policy_active:
      color, label = self._on, "LANE: CENTER"
    else:
      color, label = self._fallback, "LANE: FALLBACK"
    rl.draw_rectangle_rounded(rect, 0.35, 8, self._background)
    rl.draw_rectangle_rounded_lines_ex(rect, 0.35, 8, 3, color)
    size = measure_text_cached(self._font, label, 30)
    rl.draw_text_ex(self._font, label,
                    rl.Vector2(rect.x + (rect.width - size.x) / 2, rect.y + (rect.height - size.y) / 2),
                    30, 0, color)
