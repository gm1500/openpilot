import time
import pyray as rl

from openpilot.selfdrive.ui.onroad.gap_lane_gesture import GapLaneGesture
from openpilot.selfdrive.ui.ui_state import ui_state, get_lane_policy_enabled
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget, device


LANE_GREEN = rl.Color(13, 248, 122, 255)
ICON_GREY = rl.Color(110, 110, 110, 255)
ICON_BACKGROUND = rl.Color(0, 0, 0, 166)


def draw_gap_lane_icon(rect, personality, lane_enabled, pressed=False):
  alpha = 180 if pressed else 255
  green = rl.Color(LANE_GREEN.r, LANE_GREEN.g, LANE_GREEN.b, alpha)
  grey = rl.Color(ICON_GREY.r, ICON_GREY.g, ICON_GREY.b, alpha)
  center_x, center_y = rect.x + rect.width / 2, rect.y + rect.height / 2
  rl.draw_circle(int(center_x), int(center_y), rect.width / 2, ICON_BACKGROUND)
  for side in (-1, 1):
    rl.draw_line_ex(rl.Vector2(center_x + side * rect.width * .28, center_y - rect.height * .27),
                    rl.Vector2(center_x + side * rect.width * .34, center_y + rect.height * .27),
                    rect.width * .035, green if lane_enabled else grey)

  # Bottom-up: aggressive=1, standard=2, relaxed=3. Lane state is independent.
  bars = personality + 1 if personality in (0, 1, 2) else 0
  for index, width in enumerate((.36, .42, .48)):
    bar = rl.Rectangle(rect.x + rect.width * (1 - width) / 2,
                       rect.y + rect.height * (.28 + .17 * index), rect.width * width, rect.height * .095)
    rl.draw_rectangle_rounded(bar, .6, 8, green if index >= 3 - bars else grey)


class GapLaneButton(Widget):
  """Tap for following distance; hold for the lane-centering policy."""

  def __init__(self):
    super().__init__()
    self._gesture = GapLaneGesture()
    self._personality: int | None = None
    self._requested_personality: int | None = None
    self._request_time = 0.

  @property
  def is_pressed(self):
    return self._gesture.pressed

  def show_event(self):
    self._gesture.reset()
    self._requested_personality = None
    super().show_event()

  def hide_event(self):
    self._gesture.reset()
    super().hide_event()

  def _update_state(self):
    sm, now = ui_state.sm, time.monotonic()
    fresh = (ui_state.started and ui_state.has_longitudinal_control and sm.valid['selfdriveState'] and
             sm.recv_frame['selfdriveState'] >= ui_state.started_frame and
             0 <= now - sm.recv_time['selfdriveState'] <= .5)
    personality = sm['selfdriveState'].personality.raw if fresh else None
    if personality not in (0, 1, 2):
      personality = None
    # Display acknowledged state. Pending taps only choose the next request.
    if personality != self._personality or personality == self._requested_personality or now - self._request_time >= .5:
      self._requested_personality = None
    self._personality = personality
    if not ui_state.started or not device.awake or not self.enabled or not self.is_visible:
      self._gesture.reset()

  def _act(self, action):
    if action == 'hold':
      # Read the latest value so a recent physical LKAS transition is respected.
      ui_state.set_lane_policy_enabled(not get_lane_policy_enabled(ui_state.params))
    elif action == 'tap' and self._personality is not None:
      current = self._requested_personality if self._requested_personality is not None else self._personality
      self._requested_personality = (current - 1) % 3  # same order as the physical gap button
      self._request_time = time.monotonic()
      ui_state.params.put('LongitudinalPersonality', self._requested_personality)

  def _process_mouse_events(self):
    if not ui_state.started or not self._touch_valid():
      self._gesture.reset()
      return
    for event in gui_app.mouse_events:
      inside = rl.check_collision_point_rec(event.pos, self._hit_rect)
      self._act(self._gesture.event(event, inside))
    self._act(self._gesture.advance(time.monotonic()))

  def _render(self, rect):
    draw_gap_lane_icon(rect, self._personality, ui_state.lane_policy_enabled, self.is_pressed)
