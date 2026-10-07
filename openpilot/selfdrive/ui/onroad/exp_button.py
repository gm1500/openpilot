import time
import pyray as rl
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.longitudinal_mode import LongitudinalMode, selected_mode, set_longitudinal_mode
from openpilot.selfdrive.ui.onroad.conditional_icon import draw_conditional_icon, conditional_ring_state
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget


class ExpButton(Widget):
  def __init__(self, button_size: int, icon_size: int):
    super().__init__()
    self._params = Params()
    self._mode = LongitudinalMode.voacc
    self._icon_size = icon_size
    self._engageable: bool = False

    # State hold mechanism
    self._hold_duration = 2.0  # seconds
    self._held_mode: LongitudinalMode | None = None
    self._hold_end_time: float | None = None

    self._white_color: rl.Color = rl.Color(255, 255, 255, 255)
    self._black_bg: rl.Color = rl.Color(0, 0, 0, 166)
    self._txt_wheel: rl.Texture = gui_app.texture('icons/chffr_wheel.png', icon_size, icon_size)
    self._txt_exp: rl.Texture = gui_app.texture('icons/experimental.png', icon_size, icon_size)
    self._rect = rl.Rectangle(0, 0, button_size, button_size)

  def set_rect(self, rect: rl.Rectangle) -> None:
    self._rect.x, self._rect.y = rect.x, rect.y

  def _update_state(self) -> None:
    selfdrive_state = ui_state.sm["selfdriveState"]
    self._mode = selected_mode(selfdrive_state.experimentalMode, selfdrive_state.conditionalExperimental)
    self._engageable = selfdrive_state.engageable or selfdrive_state.enabled

  def _handle_mouse_release(self, _):
    super()._handle_mouse_release(_)
    if self._is_toggle_allowed():
      new_mode = LongitudinalMode((int(self._held_or_actual_mode()) + 1) % 3)
      set_longitudinal_mode(self._params, new_mode)

      # Hold new state temporarily
      self._held_mode = new_mode
      self._hold_end_time = time.monotonic() + self._hold_duration

  def _render(self, rect: rl.Rectangle) -> None:
    center_x = int(self._rect.x + self._rect.width // 2)
    center_y = int(self._rect.y + self._rect.height // 2)

    self._white_color.a = 180 if self.is_pressed or not self._engageable else 255

    mode = self._held_or_actual_mode()
    rl.draw_circle(center_x, center_y, self._rect.width / 2, self._black_bg)
    if mode == LongitudinalMode.conditional:
      state = conditional_ring_state(ui_state.sm, ui_state.started_frame, time.monotonic())
      color = self._white_color
      if state != 'ready':
        color = rl.Color(255, 190, 0, self._white_color.a) if state == 'assisting' else rl.Color(0, 220, 255, self._white_color.a)
        radius = self._rect.width * .49
        width = self._rect.width * .04
        rl.draw_ring(rl.Vector2(center_x, center_y), radius - width, radius, 0, 360, 96, color)
      draw_conditional_icon(center_x, center_y, self._icon_size, color)
      return
    texture = self._txt_exp if mode == LongitudinalMode.experimental else self._txt_wheel
    rl.draw_texture_ex(texture, rl.Vector2(center_x - texture.width / 2, center_y - texture.height / 2), 0.0, 1.0, self._white_color)

  def _held_or_actual_mode(self):
    now = time.monotonic()
    if self._hold_end_time and now < self._hold_end_time:
      return self._held_mode

    if self._hold_end_time and now >= self._hold_end_time:
      self._hold_end_time = self._held_mode = None

    return self._mode

  def _is_toggle_allowed(self):
    if not self._params.get_bool("ExperimentalModeConfirmed"):
      return False

    # Mirror exp mode toggle using persistent car params
    return ui_state.has_longitudinal_control
