import time
import pyray as rl

from openpilot.selfdrive.ui.onroad.lane_icon import LANE_GREEN, draw_lane_icon_frame
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets import Widget


BAR_GREY = rl.Color(110, 110, 110, 255)


class GapButton(Widget):
  def __init__(self):
    super().__init__()
    self._personality: int | None = None
    self._requested_personality: int | None = None
    self._request_time = 0.
    self.set_visible(lambda: ui_state.has_longitudinal_control)

  def _update_state(self) -> None:
    sm = ui_state.sm
    now = time.monotonic()
    fresh = (ui_state.has_longitudinal_control and sm.valid['selfdriveState'] and
             sm.recv_frame['selfdriveState'] >= ui_state.started_frame and
             0 <= now - sm.recv_time['selfdriveState'] <= .5)
    personality = sm['selfdriveState'].personality.raw if fresh else None
    if personality not in (0, 1, 2):
      personality = None
    # Render acknowledged state; pending taps only determine the next request.
    # A stock-button change or missing acknowledgement clears the pending value.
    if personality != self._personality or personality == self._requested_personality or now - self._request_time >= .5:
      self._requested_personality = None
    self._personality = personality
    self.set_enabled(personality is not None)

  def _handle_mouse_release(self, mouse_pos) -> None:
    super()._handle_mouse_release(mouse_pos)
    self._update_state()
    if not self.enabled:
      return
    current = self._requested_personality if self._requested_personality is not None else self._personality
    self._requested_personality = (current - 1) % 3  # match the stock gap button
    self._request_time = time.monotonic()
    ui_state.params.put('LongitudinalPersonality', self._requested_personality)

  def _render(self, rect: rl.Rectangle) -> None:
    alpha = 180 if self.is_pressed or not self.enabled else 255
    green = rl.Color(LANE_GREEN.r, LANE_GREEN.g, LANE_GREEN.b, alpha)
    grey = rl.Color(BAR_GREY.r, BAR_GREY.g, BAR_GREY.b, alpha)
    draw_lane_icon_frame(rect, green if self.enabled else grey)
    bars = self._personality + 1 if self._personality is not None else 0
    for index, width in enumerate((.36, .42, .48)):
      bar = rl.Rectangle(rect.x + rect.width * (1 - width) / 2,
                         rect.y + rect.height * (.30 + .17 * index), rect.width * width, rect.height * .065)
      rl.draw_rectangle_rounded(bar, .6, 8, green if index >= 3 - bars else grey)
