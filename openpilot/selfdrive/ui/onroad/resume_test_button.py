"""Momentary manual resume-test control; release/heartbeat loss cancels."""
import time
import pyray as rl

from openpilot.cereal import messaging
from openpilot.selfdrive.controls.lib.resume_test import fresh
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget


class ResumeTestButton(Widget):
  def __init__(self):
    super().__init__()
    self.pm = messaging.PubMaster(['resumeTestRequest'])
    self.request_id = 0
    self.last_send = 0.
    self.visible = False
    self._font = gui_app.font(FontWeight.BOLD)

  def _send(self, held):
    msg = messaging.new_message('resumeTestRequest')
    msg.valid = True
    msg.resumeTestRequest.requestId = self.request_id
    msg.resumeTestRequest.held = held
    self.pm.send('resumeTestRequest', msg)
    self.last_send = time.monotonic()

  def _handle_mouse_press(self, _):
    if self.enabled:
      self.request_id = time.monotonic_ns()
      self._send(True)

  def _handle_mouse_release(self, _):
    self._send(False)

  def _update_state(self):
    sm, now = ui_state.sm, time.monotonic()
    valid = ui_state.started and fresh(sm, 'controlsState', now) and fresh(sm, 'carState', now)
    state = sm['controlsState'].resumeTest
    self.visible = valid and (state.ready or state.active or state.holding)
    self.set_enabled(valid and (state.ready or state.active))
    # Continue sending cancellation even if a touch or the button disappears.
    if now - self.last_send >= .05:
      self._send(self.visible and self.enabled and self.is_pressed)

  def _render(self, rect):
    if not self.visible:
      return
    state = ui_state.sm['controlsState'].resumeTest
    title = 'TEST RESUME'
    subtitle = 'HOLD TO TEST / 1 SECOND MAX'
    if state.active:
      title = f'TESTING {state.remaining:.1f}s'
      subtitle = 'RELEASE TO HOLD BRAKES'
    elif state.holding and not state.ready:
      title, subtitle = 'BRAKES HELD', 'TEST ENDED'
    elif state.holding:
      subtitle = 'HOLD TO RETEST / RESUME TO EXIT'
    color = rl.Color(196, 106, 15, 240) if state.active else rl.Color(27, 61, 82, 240)
    rl.draw_rectangle_rounded(rect, .18, 12, color)
    rl.draw_rectangle_rounded_lines_ex(rect, .18, 12, 3, rl.Color(255, 187, 80, 255))
    for text, size, y in ((title, rect.height * .28, .17), (subtitle, rect.height * .13, .63)):
      width = measure_text_cached(self._font, text, size).x
      rl.draw_text_ex(self._font, text, rl.Vector2(rect.x + (rect.width - width) / 2, rect.y + rect.height * y), size, 0, rl.WHITE)
