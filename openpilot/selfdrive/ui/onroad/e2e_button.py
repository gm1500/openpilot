import time
from dataclasses import dataclass

import raylib_nb as rl

from openpilot.selfdrive.ui.onroad.gap_lane_gesture import GapLaneGesture
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget, device


@dataclass(frozen=True)
class E2EButtonState:
  requested: bool = False
  active: bool = False
  fresh: bool = False


def read_button_state(ui, now):
  sm = ui.sm

  def message_fresh(service):
    return (ui.started and sm.valid[service] and sm.recv_frame[service] >= ui.started_frame and
            0 <= now - sm.recv_time[service] <= .5 and 0 <= now - sm.logMonoTime[service] * 1e-9 <= .5)

  fresh = all(message_fresh(service) for service in ('selfdriveState', 'carControl'))
  ss, cc = sm['selfdriveState'], sm['carControl']
  return E2EButtonState(
    requested=ui.experimental_mode,
    active=bool(fresh and ss.experimentalMode and cc.longActive),
    fresh=fresh)


def button_colors(state, pressed=False):
  alpha = 180 if pressed else 255
  background = rl.Color(0, 0, 0, 166)
  mode = rl.Color(255, 255, 255, alpha if state.fresh else round(alpha * .6))
  return background, mode


def draw_e2e_button(rect, state, pressed=False):
  scale = rect.width / 192.
  background, mode_color = button_colors(state, pressed)
  center = rl.Vector2(rect.x + rect.width / 2, rect.y + rect.height / 2)
  rl.draw_circle(int(center.x), int(center.y), rect.width / 2, background)
  # Preserve the original comma button's 192 px circle and 144 px mode asset.
  name = 'experimental.png' if state.requested or state.active else 'chffr_wheel.png'
  icon = gui_app.texture('icons/' + name, round(144 * scale), round(144 * scale))
  rl.draw_texture_ex(icon, rl.Vector2(center.x - icon.width / 2, center.y - icon.height / 2), 0., 1., mode_color)


class E2EButton(Widget):
  """Tap selects full E2E; a long hold is consumed without changing modes."""

  def __init__(self):
    super().__init__()
    self._gesture = GapLaneGesture()
    self._consume_until = 0.
    self._state = E2EButtonState()
    self._drive_frame: int | None = None

  @property
  def is_pressed(self):
    # Consume release as well as press so the camera cannot also open its menu.
    return self._gesture.pressed or time.monotonic() < self._consume_until

  def _reset_touch(self):
    self._gesture.reset()
    self._consume_until = 0.

  def show_event(self):
    self._reset_touch()
    super().show_event()

  def hide_event(self):
    self._reset_touch()
    super().hide_event()

  def _update_state(self):
    if self._drive_frame != ui_state.started_frame:
      self._reset_touch()
      self._drive_frame = ui_state.started_frame
    self._state = read_button_state(ui_state, time.monotonic())
    if not ui_state.started or not device.awake or not self.enabled or not self.is_visible:
      self._reset_touch()

  def _act(self, action):
    if action is None:
      return
    self._consume_until = time.monotonic() + .2
    if action == 'tap' and ui_state.map_cruise_supported:
      ui_state.set_experimental_mode(not ui_state.params.get_bool('ExperimentalMode'))

  def _process_mouse_events(self):
    if not ui_state.started or not device.awake or not self.enabled or not self.is_visible or not self._touch_valid():
      self._reset_touch()
      return
    for event in gui_app.mouse_events:
      inside = rl.check_collision_point_rec(rl.Vector2(event.pos.x, event.pos.y), self._hit_rect)
      releasing_contact = self._gesture.pressed and event.left_released
      self._act(self._gesture.event(event, inside))
      if releasing_contact:
        self._consume_until = time.monotonic() + .2
    self._act(self._gesture.advance(time.monotonic()))

  def _render(self, rect):
    draw_e2e_button(rect, self._state, self._gesture.pressed)
