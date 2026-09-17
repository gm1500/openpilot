import pyray as rl
from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget


class AnchorLaneButton(Widget):
  """On-road per-drive selector for anchored E2E lane centering."""
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._enabled = True
    self._active = False
    self._holding = False
    self._font = gui_app.font(FontWeight.SEMI_BOLD)
    self._center = rl.Color(128, 216, 166, 255)
    self._hold = rl.Color(97, 183, 230, 255)
    self._off = rl.Color(166, 166, 166, 255)
    self._ready = rl.Color(218, 175, 47, 255)
    self._background = rl.Color(0, 0, 0, 166)

  def _update_state(self) -> None:
    self._enabled = ui_state.anchor_lane_policy_enabled
    self._active = ui_state.anchor_lane_policy_active
    self._holding = ui_state.anchor_lane_policy_holding

  def _handle_mouse_release(self, mouse_pos) -> None:
    super()._handle_mouse_release(mouse_pos)
    self._enabled = not self._enabled
    self._params.put_bool("AnchorLanePolicyEnabled", self._enabled)

  def _render(self, rect: rl.Rectangle) -> None:
    if not self._enabled:
      color, label = self._off, "ANCHOR: E2E"
    elif self._active:
      color, label = self._center, "ANCHOR: CENTER"
    elif self._holding:
      color, label = self._hold, "ANCHOR: HOLD"
    else:
      color, label = self._ready, "ANCHOR: READY"
    rl.draw_rectangle_rounded(rect, 0.35, 8, self._background)
    rl.draw_rectangle_rounded_lines_ex(rect, 0.35, 8, 3, color)
    size = measure_text_cached(self._font, label, 30)
    rl.draw_text_ex(self._font, label,
                    rl.Vector2(rect.x + (rect.width - size.x) / 2, rect.y + (rect.height - size.y) / 2),
                    30, 0, color)
