import pyray as rl
import time

from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state, device
from openpilot.system.sentry.config import setting, RANGES, ARM_DELAYS
from openpilot.system.sentry.clock import time_status
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import ListItem, MultipleButtonAction, toggle_item, text_item, button_item
from openpilot.system.ui.widgets.scroller_tici import Scroller


class SentrySlider(Widget):
  def __init__(self, params, key, title, unit='', step=1):
    super().__init__()
    self.params, self.key, self.title, self.unit, self.step = params, key, title, unit, step
    self.low, self.high = RANGES[key]
    self.value = setting(params, key)
    self.dragging = False
    self._rect.height = 190
    self.set_enabled(lambda: not ui_state.ignition)

  def set_parent_rect(self, rect):
    super().set_parent_rect(rect)
    self._rect.width = rect.width

  def _render(self, rect):
    if not self.dragging:
      self.value = setting(self.params, self.key)
    color = rl.WHITE if self.enabled else rl.GRAY
    value = f'{self.value:g}{self.unit}'
    font = gui_app.font(FontWeight.NORMAL)
    rl.draw_text_ex(font, f'{self.title}: {value}', rl.Vector2(rect.x+10, rect.y+25), 43, 0, color)
    self.track = rl.Rectangle(rect.x+35, rect.y+120, rect.width-70, 12)
    x = self.track.x + (self.value-self.low)/(self.high-self.low)*self.track.width
    rl.draw_rectangle_rounded(self.track, 1, 12, rl.Color(80, 80, 80, 255))
    rl.draw_rectangle_rounded(rl.Rectangle(self.track.x, self.track.y, max(1, x-self.track.x), 12), 1, 12, rl.MAROON)
    rl.draw_circle(int(x), int(self.track.y+6), 23, color)
    if not self._touch_valid() or not self.enabled:
      self.dragging = False

  def _handle_mouse_press(self, pos):
    self.dragging = abs(pos.y-self.track.y) < 45

  def _handle_mouse_event(self, event):
    if self.dragging and self._touch_valid() and self.enabled:
      fraction = max(0., min(1., (event.pos.x-self.track.x)/self.track.width))
      self.value = round(round((self.low+fraction*(self.high-self.low))/self.step)*self.step, 2)
      if event.left_released:
        # One persistent write per completed gesture, never per video/UI frame.
        self.params.put(self.key, self.value)
        self.dragging = False

  def hide_event(self):
    self.dragging = False
    super().hide_event()


class SentryCameraButtons(MultipleButtonAction):
  KEYS = ('SentryNarrowCamera', 'SentryWideCamera', 'SentryCabinCamera')

  def __init__(self, params):
    super().__init__(['Narrow', 'Wide', 'Cabin'], button_width=210)
    self.params = params
    self.set_enabled(lambda: not ui_state.ignition)

  def is_button_selected(self, index):
    return setting(self.params, self.KEYS[index])

  def select_button(self, index):
    if self.enabled:
      self.params.put_bool(self.KEYS[index], not self.is_button_selected(index))


class SentryArmButtons(MultipleButtonAction):
  def __init__(self, params):
    super().__init__([f'{delay}s' for delay in ARM_DELAYS], button_width=210)
    self.params = params
    self.set_enabled(lambda: not ui_state.ignition)

  def is_button_selected(self, index):
    return setting(self.params, 'SentryArmDelay') == ARM_DELAYS[index]

  def select_button(self, index):
    if self.enabled:
      self.params.put('SentryArmDelay', ARM_DELAYS[index], block=True)


class SentryLayout(Widget):
  def __init__(self):
    super().__init__()
    self.params = Params()
    self._enable_toggle = toggle_item('Enable Sentry',
                                     'Arms after ignition turns off. Switch off or press STOP to end Sentry until you enable it again.',
                                     initial_state=setting(self.params, 'SentryEnabled'),
                                     callback=self._set_sentry_enabled, enabled=self._can_toggle_sentry)
    self._stop_button = button_item('Sentry', 'STOP',
                                   'Stops parked capture and the red eye. Saved events stay in the upload queue.',
                                   callback=lambda: self._set_sentry_enabled(False),
                                   enabled=lambda: setting(self.params, 'SentryEnabled'))
    self._stop_button.action_item.set_value(self.phase_status)
    items = [
      self._stop_button,
      self._enable_toggle,
      ListItem('Cameras', description='Each camera switches independently. Green means on. Cabin infrared adjusts automatically.',
               action_item=SentryCameraButtons(self.params)),
      self.toggle('SentryEye', 'Red eye', 'A large, bright eye follows motion after the screen times out. Tap to dismiss until the next timeout.'),
      self.toggle('SentryWatermark', 'Video watermark',
                  'Local timestamp with UTC offset, camera name and parking coordinates when available. Uses UTC until the timezone is known.'),
      ListItem('Arming delay', description='Camera exposure warmup after live video is ready. Footage buffers during the delay; motion and the eye wait.',
               action_item=SentryArmButtons(self.params)),
      SentrySlider(self.params, 'SentrySensitivity', 'Motion sensitivity'),
      button_item('Save an event now', 'SAVE', callback=lambda: self.params.put_bool('SentryManualTrigger', True),
                  enabled=lambda: ui_state.sentry_active),
      text_item('Buffered footage', self.buffer_status),
      text_item('Current time', lambda: time_status(self.params),
                'Uses the device clock, synchronized by the system through the network or GPS. ' +
                'The local timezone is remembered from the last known position; continuous GPS is not needed.'),
      text_item('Battery voltage', self.voltage_status),
    ]
    for key, title, unit, step in (
      ('SentryCutoffVoltage', 'Battery cutoff', ' V', .1),
      ('SentryPreMinutes', 'Pre-event buffer', ' min', 1),
      ('SentryPostSeconds', 'Record after motion stops', ' s', 10),
      ('SentryMaxHours', 'Maximum parked time', ' h', 1),
      ('SentryMaxTemp', 'Pause capture above', ' °C', 1),
    ):
      items.append(SentrySlider(self.params, key, title, unit, step))
    items.extend([
      text_item('Event uploads', 'Automatic',
                'Saved footage from every selected camera uploads on unmetered connections, usually Wi-Fi. ' +
                'Events use normal route storage cleanup; upload requires Connect to accept the files.'),
      text_item('Battery protection', 'Sentry cutoff',
                'While Sentry is active, its voltage and time limits replace the stock offroad software limits. ' +
                'Default cutoff is 11.8 V; adjustable down to 11.0 V. Hardware protections remain active.'),
    ])
    self.scroller = self._child(Scroller(items, spacing=0, line_separator=True))

  def _can_toggle_sentry(self):
    return setting(self.params, 'SentryEnabled') or (not ui_state.ignition and not ui_state.started)

  def _set_sentry_enabled(self, enabled):
    if enabled and (ui_state.ignition or ui_state.started):
      return
    # Persist the command before refreshing the switch or evaluating process gates.
    self.params.put_bool('SentryEnabled', enabled, block=True)
    if not enabled:
      self.params.remove('SentryManualTrigger')
      device.sentry_motion_until = 0.
    self._enable_toggle.action_item.set_state(enabled)

  def _update_state(self):
    self._enable_toggle.action_item.set_state(setting(self.params, 'SentryEnabled'))

  def toggle(self, key, title, description=None):
    return toggle_item(title, description, initial_state=setting(self.params, key),
                       callback=lambda value: self.params.put_bool(key, value), enabled=lambda: not ui_state.ignition)

  def phase_status(self):
    if not setting(self.params, 'SentryEnabled'):
      return 'Off'
    sm = ui_state.sm
    if not sm.valid['sentryState'] or not 0 <= time.monotonic()-sm.recv_time['sentryState'] < 2.:
      return 'Controller unavailable'
    return sm['sentryState'].phase or 'Starting'

  def voltage_status(self):
    sm = ui_state.sm
    if not sm.valid['peripheralState'] or not 0 <= time.monotonic()-sm.recv_time['peripheralState'] < 3.:
      return 'Unavailable'
    return f"{sm['peripheralState'].voltage/1000.:.1f} V"

  def buffer_status(self):
    state = ui_state.sm['sentryRecorderState']
    return f'{state.bufferSeconds:.0f} s / {state.bufferBytes/1048576:.0f} MB' if ui_state.sentry_active else 'Idle'

  def _render(self, rect):
    self.scroller.render(rect)
