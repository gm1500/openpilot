import pyray as rl
import time

from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.sentry.config import setting, RANGES
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


class SentryLayout(Widget):
  def __init__(self):
    super().__init__()
    self.params = Params()
    items = [
      text_item('Sentry', self.phase_status,
                'Parked motion recording for comma 3X. Up to five minutes of compressed video stay in RAM until an event.'),
      self.toggle('SentryEnabled', 'Enable Sentry', 'Arms after ignition turns off. Stops when the vehicle starts.'),
      SentrySlider(self.params, 'SentrySensitivity', 'Motion sensitivity'),
      text_item('Buffered footage', self.buffer_status),
      text_item('Battery voltage', self.voltage_status),
      button_item('Save an event now', 'SAVE', callback=lambda: self.params.put_bool('SentryManualTrigger', True),
                  enabled=lambda: ui_state.sentry_active),
    ]
    for key, title, unit, step in (
      ('SentryArmDelay', 'Arming delay', ' s', 10),
      ('SentryPreMinutes', 'Pre-event buffer', ' min', 1),
      ('SentryPostSeconds', 'Record after motion stops', ' s', 10),
      ('SentryCutoffVoltage', 'Battery cutoff', ' V', .1),
      ('SentryMaxHours', 'Maximum parked time', ' h', 1),
      ('SentryMaxTemp', 'Pause capture above', ' °C', 1),
      ('SentryStorageGB', 'Event storage limit', ' GiB', 1),
    ):
      items.append(SentrySlider(self.params, key, title, unit, step))
    items.extend([
      text_item('Event storage', 'Preserved', 'Saved events remain on the device until deleted. When the limit is reached, new event saving pauses.'),
      text_item('Battery protection', 'Sentry cutoff',
                'While Sentry is active, its voltage and time limits replace the stock offroad software limits. ' +
                'Default cutoff is 11.8 V; adjustable down to 11.0 V. Hardware protections remain active.'),
      ListItem('Cameras', description='Each camera switches independently. Green means on. Cabin infrared adjusts automatically.',
               action_item=SentryCameraButtons(self.params)),
      self.toggle('SentryEye', 'Red eye', 'Appears with motion after the screen times out. Tap to dismiss until the next screen timeout.'),
      self.toggle('SentryWatermark', 'Video watermark',
                  'UTC timestamp, camera name and saved parking coordinates. Shows unavailable when no parking location is known.'),
      text_item('Parking location', self.location_status,
                'Uses the SLC parking checkpoint first. Otherwise requests a GPS fix for up to 90 seconds, then switches GPS off.'),
    ])
    self.scroller = self._child(Scroller(items, spacing=0, line_separator=True))

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

  def location_status(self):
    state = ui_state.sm['sentryState']
    return ('SLC parking position' if state.locationSource == 'slcParked' else 'GPS snapshot') if state.locationValid else 'Unavailable'

  def _render(self, rect):
    self.scroller.render(rect)
