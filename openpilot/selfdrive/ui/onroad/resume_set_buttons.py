"""Mutually exclusive, momentary Resume/Set HUD controls; no persistent Params."""
import time
import pyray as rl

from openpilot.cereal import messaging
from openpilot.selfdrive.controls.lib.resume_set_test import fresh
from openpilot.selfdrive.ui.onroad.resume_set_gesture import ResumeSetGesture
from openpilot.selfdrive.ui.ui_state import ui_state, device
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget


class ResumeSetButtons(Widget):
  def __init__(self):
    super().__init__()
    self.pm = messaging.PubMaster(['resumeSetTestRequest'])
    self.gesture = ResumeSetGesture()
    self.last_send = 0.
    self.show = False
    self._font = gui_app.font(FontWeight.BOLD)

  def _buttons(self):
    rect = self.rect
    gap = rect.width * .025
    width = (rect.width - gap) / 2
    return {2: rl.Rectangle(rect.x, rect.y, width, rect.height * .68),
            3: rl.Rectangle(rect.x + width + gap, rect.y, width, rect.height * .68)}

  def _send(self, now):
    msg = messaging.new_message('resumeSetTestRequest')
    msg.valid = True
    msg.resumeSetTestRequest.requestId = self.gesture.request_id
    msg.resumeSetTestRequest.held = bool(self.gesture.button)
    msg.resumeSetTestRequest.button = self.gesture.button
    self.pm.send('resumeSetTestRequest', msg)
    self.last_send = now

  def _update_state(self):
    sm, now = ui_state.sm, time.monotonic()
    state = sm['controlsState'].resumeSetTest
    valid = (ui_state.started and device.awake and fresh(sm, 'controlsState', now) and fresh(sm, 'carState', now))
    self.show = valid and state.supported and (sm['carState'].vEgo < 3. or state.holding)
    self.set_enabled(self.show)
    before = (self.gesture.button, self.gesture.request_id)
    if not self.show or not (state.ready or state.active):
      self.gesture.cancel()
    # Examine all slots, including touches outside this widget. Two fingers,
    # sliding to the other button, and leaving the button all cancel a hold.
    for event in gui_app.mouse_events:
      button = next((b for b, r in self._buttons().items() if rl.check_collision_point_rec(event.pos, r)), 0)
      self.gesture.event(event.slot, button, event.left_pressed, event.left_released, time.monotonic_ns(),
                         self.show and state.ready)
    poll_time, held_slots = gui_app.touch_hold_snapshot()
    self.gesture.confirm_hold(time.monotonic(), poll_time, held_slots)
    if (self.gesture.button, self.gesture.request_id) != before or now - self.last_send >= .05:
      self._send(now)

  def _render(self, rect):
    if not self.show:
      return
    state = ui_state.sm['controlsState'].resumeSetTest
    for button, r in self._buttons().items():
      active = state.active and state.button == button
      color = rl.Color(185, 99, 15, 240) if active else rl.Color(27, 61, 82, 240)
      rl.draw_rectangle_rounded(r, .18, 12, color)
      rl.draw_rectangle_rounded_lines_ex(r, .18, 12, 3, rl.Color(255, 187, 80, 255))
      title = 'RESUME TEST' if button == 2 else 'SET TEST'
      subtitle = 'RELEASE TO STOP' if active else 'HOLD TO ROLL + PRESS'
      for text, size, y in ((title, r.height * .25, .16), (subtitle, r.height * .135, .62)):
        self._text(text, size, r.x + r.width / 2, r.y + r.height * y,
                   rl.WHITE if state.ready or active else rl.Color(160, 160, 160, 255))
    cruise = {0: 'OFF', 1: 'ACTIVE', 2: 'LIMITED', 3: 'FAULT', 4: 'STANDSTILL'}.get(state.cruiseState, '?')
    if state.active:
      status = f'CRUISE {cruise}  |  {state.remaining:.1f}s  |  {state.distance:.1f}m  |  TX {state.buttonCount}'
    elif state.holding:
      hold = 'STOPPED / BRAKE COMMANDED' if state.brakeHeld else 'STOP / HOLD REQUESTED'
      status = f'{hold}  |  CRUISE {cruise}'
    else:
      status = f'CRUISE {cruise}  |  ' + ('READY' if state.ready else state.reason.upper())
    self._text(status, rect.height * .10, rect.x + rect.width / 2, rect.y + rect.height * .73, rl.WHITE)
    self._text('10s / 7.2 km/h / 10m MAX  -  PEDALS OR CANCEL TO EXIT', rect.height * .085,
               rect.x + rect.width / 2, rect.y + rect.height * .88, rl.WHITE)

  def _text(self, text, size, cx, y, color):
    width = measure_text_cached(self._font, text, size).x
    rl.draw_text_ex(self._font, text, rl.Vector2(cx - width / 2, y), size, 0, color)

  def _offroad_transition(self):
    self.gesture.cancel()
    self.gesture.down.clear()
    self._send(time.monotonic())
