import pyray as rl
import time

from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state, device
from openpilot.system.sentry.config import setting, parking_mode, select_parking_mode, RANGES, DURATION_PRESETS, CAMERA_WARMUP_SECONDS
from openpilot.system.sentry.clock import time_status
from openpilot.system.ui.lib.application import gui_app, FontWeight, TextAlignment
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import ListItem, MultipleButtonAction, ToggleAction, toggle_item, text_item, button_item
from openpilot.system.ui.widgets.label import gui_label
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets.toggle import Toggle, WIDTH as TOGGLE_WIDTH, HEIGHT as TOGGLE_HEIGHT


class SentrySlider(Widget):
  def __init__(self, params, key, title, unit='', step=1, suffix=None, shutdown_key=None, hint=''):
    super().__init__()
    self.params, self.key, self.title, self.unit, self.step = params, key, title, unit, step
    self.low, self.high = RANGES[key]
    self.value = setting(params, key)
    self.dragging = False
    self._rect.height = 190
    self.suffix, self.shutdown_key = suffix, shutdown_key
    self.hint = hint
    self.shutdown_toggle = None
    if shutdown_key:
      self.shutdown_toggle = self._child(Toggle(initial_state=setting(params, shutdown_key),
                                              callback=lambda value: params.put_bool(shutdown_key, value, block=True)))

  def set_touch_valid_callback(self, callback):
    super().set_touch_valid_callback(callback)
    if self.shutdown_toggle:
      self.shutdown_toggle.set_touch_valid_callback(callback)

  def set_parent_rect(self, rect):
    super().set_parent_rect(rect)
    self._rect.width = rect.width

  def _render(self, rect):
    if not self.dragging:
      self.value = setting(self.params, self.key)
    color = rl.WHITE if self.enabled else rl.GRAY
    value = f'{self.value:g}{self.unit}'
    font = gui_app.font(FontWeight.NORMAL)
    suffix = f' ({self.suffix()})' if self.suffix else ''
    rl.draw_text_ex(font, f'{self.title}: {value}{suffix}', rl.Vector2(rect.x+10, rect.y+25), 43, 0, color)
    if self.shutdown_toggle:
      x = rect.x+rect.width-TOGGLE_WIDTH-20
      rl.draw_text_ex(font, 'Shut down', rl.Vector2(x-185, rect.y+32), 32, 0, color)
      self.shutdown_toggle.set_state(setting(self.params, self.shutdown_key))
      self.shutdown_toggle.set_enabled(self.enabled)
      self.shutdown_toggle.set_parent_rect(self._parent_rect or rect)
      self.shutdown_toggle.render(rl.Rectangle(x, rect.y+10, TOGGLE_WIDTH, TOGGLE_HEIGHT))
    self.track = rl.Rectangle(rect.x+35, rect.y+120, rect.width-70, 12)
    x = self.track.x + (self.value-self.low)/(self.high-self.low)*self.track.width
    rl.draw_rectangle_rounded(self.track, 1, 12, rl.Color(80, 80, 80, 255))
    rl.draw_rectangle_rounded(rl.Rectangle(self.track.x, self.track.y, max(1, x-self.track.x), 12), 1, 12, rl.MAROON)
    rl.draw_circle(int(x), int(self.track.y+6), 23, color)
    if self.hint:
      rl.draw_text_ex(font, self.hint, rl.Vector2(rect.x+10, rect.y+158), 24, 0, rl.GRAY)
    if not self._touch_valid() or not self.enabled:
      self.dragging = False

  def _handle_mouse_press(self, pos):
    on_toggle = self.shutdown_toggle and rl.check_collision_point_rec(pos, self.shutdown_toggle.rect)
    self.dragging = not on_toggle and abs(pos.y-self.track.y) < 45

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


class SentryDurationButtons(MultipleButtonAction):
  def __init__(self, params, key):
    super().__init__([f'{seconds}s' for seconds in DURATION_PRESETS], button_width=210)
    self.params, self.key = params, key

  def is_button_selected(self, index):
    return setting(self.params, self.key) == DURATION_PRESETS[index]

  def select_button(self, index):
    if self.enabled:
      self.params.put(self.key, DURATION_PRESETS[index], block=True)


class SentryModeButtons(MultipleButtonAction):
  def __init__(self, params, key=None):
    super().__init__(['Low Risk', 'High Risk', 'Off this parking'], button_width=280)
    self.params, self.key = params, key

  def is_button_selected(self, index):
    if self.key:
      return setting(self.params, self.key) == index
    return parking_mode(self.params) == (index if index < 2 else 'off')

  def select_button(self, index):
    if self.enabled:
      if self.key:
        self.params.put(self.key, index, block=True)
      else:
        select_parking_mode(self.params, index if index < 2 else 'off')

  def _render(self, rect):
    self.button_width = (rect.width-40)/3
    for i, (title, color) in enumerate(zip(self.buttons, ((245, 196, 46), (190, 35, 42), (71, 184, 104)), strict=True)):
      selected = self.is_button_selected(i)
      brightness = 1. if selected else .4
      fill = rl.Color(*(int(c*brightness) for c in color), 255)
      box = rl.Rectangle(rect.x+i*(self.button_width+20), rect.y+(rect.height-100)/2, self.button_width, 100)
      rl.draw_rectangle_rounded(box, 1., 20, fill)
      gui_label(box, title, 30, rl.BLACK if selected and i != 1 else rl.WHITE,
                font_weight=FontWeight.MEDIUM, alignment=TextAlignment.CENTER)


class SentryEnableAction(ToggleAction):
  def __init__(self, params, callback, key='SentryEnabled', mode_key=None):
    super().__init__(initial_state=setting(params, key), width=1080, callback=callback)
    self._child(self.toggle)
    self.modes = self._child(SentryModeButtons(params, mode_key))

  def set_touch_valid_callback(self, callback):
    super().set_touch_valid_callback(callback)
    self.modes.set_touch_valid_callback(callback)

  def _render(self, rect):
    for child in (self.toggle, self.modes):
      child.set_parent_rect(self._parent_rect or rect)
      child.set_enabled(self.enabled)
    self.toggle.render(rl.Rectangle(rect.x, rect.y+(rect.height-TOGGLE_HEIGHT)/2, TOGGLE_WIDTH, TOGGLE_HEIGHT))
    self.modes.render(rl.Rectangle(rect.x+TOGGLE_WIDTH+40, rect.y, rect.width-TOGGLE_WIDTH-40, rect.height))


class SentryLayout(Widget):
  def __init__(self):
    super().__init__()
    self.params = Params()
    self._enable_toggle = ListItem('Enable Sentry',
                                   description='The switch enables automatic arming after ignition turns off. ' +
                                   'Low Risk: sound and vehicle movement. High Risk: also camera motion. ' +
                                   'Off this parking skips only this parking session and keeps the master switch enabled. ' +
                                   'Each mode tap restarts the arming delay. A new drive resets the parking choice. ' +
                                   'Switch off or press STOP to disable Sentry until enabled again.',
                                   action_item=SentryEnableAction(self.params, self._set_sentry_enabled))
    self._stop_button = button_item('Sentry', 'STOP',
                                   'Stops parked capture and the red eye. Saved events remain available for manual upload from Connect.',
                                   callback=lambda: self._set_sentry_enabled(False),
                                   enabled=lambda: setting(self.params, 'SentryEnabled'))
    self._stop_button.action_item.set_value(self.phase_status)
    self._livestream_toggle = ListItem('Livestream trigger',
      description='Each time you open live video in Connect, override the current parking mode with this selection. ' +
                  'Low Risk or High Risk also enables Sentry and overrides the Wi-Fi skip. Off skips this parking. ' +
                  'Choosing a mode here only configures the trigger. ' +
                  'Capture pauses during live viewing and rearms after it closes. Battery, temperature and parking-time limits still apply.',
      action_item=SentryEnableAction(self.params,
        lambda value: self.params.put_bool('SentryLivestreamOverride', value, block=True),
        key='SentryLivestreamOverride', mode_key='SentryLivestreamMode'))
    items = [
      self._stop_button,
      self._enable_toggle,
      self._livestream_toggle,
      self.toggle('SentrySkipOnWifi', 'Skip Sentry on Wi-Fi',
                  'Defaults to Off this parking when parked on Wi-Fi, even with the screen off or no prompt. ' +
                  'Select Low Risk or High Risk above to override it for this parking session.'),
      ListItem('Arming delay', description=f'Includes {CAMERA_WARMUP_SECONDS:g} seconds of camera warmup once video is available. ' +
               'Footage buffers throughout; motion saves and the eye wait until the total delay finishes.',
               action_item=SentryDurationButtons(self.params, 'SentryArmDelay')),
      ListItem('Pre-event buffer', description='How much available footage to keep before motion is confirmed.',
               action_item=SentryDurationButtons(self.params, 'SentryPreSeconds')),
      ListItem('Record after motion stops', description='Keep recording for this long after the last confirmed movement. New motion extends it.',
               action_item=SentryDurationButtons(self.params, 'SentryPostSeconds')),
      SentrySlider(self.params, 'SentrySensitivity', 'Motion sensitivity'),
      SentrySlider(self.params, 'SentryAudioSensitivity', 'Audio sensitivity', '%',
                   hint='Higher reacts to quieter sounds and knocks. Background noise calibrates automatically.'),
      SentrySlider(self.params, 'SentryGyroSensitivity', 'Gyro sensitivity', '%',
                   hint='Higher reacts to lighter shakes. Background vibration calibrates automatically.'),
      SentrySlider(self.params, 'SentryCutoffVoltage', 'Battery cutoff', ' V', .1, suffix=self.voltage_status),
      SentrySlider(self.params, 'SentryMaxHours', 'Maximum parked time', ' h', shutdown_key='SentryShutdownOnTimeLimit',
                   hint='Shutdown off: stop Sentry until the next drive. Normal offroad power rules apply.'),
      SentrySlider(self.params, 'SentryMaxTemp', 'Pause capture above', ' °C', shutdown_key='SentryShutdownOnOverheat',
                   hint='Shutdown off: pause until 8°C cooler. The parked-time limit still applies.'),
      self.toggle('SentryEye', 'Red eye', 'Reacts to camera motion, sound and physical movement. Fades when activity stops while recording finishes. ' +
                  'Gyro activity produces a large, fast red pulse. Tap to dismiss until the next timeout.'),
      self.toggle('SentryWatermark', 'Video watermark',
                  'Local timestamp with UTC offset, camera name and parking coordinates when available. Uses UTC until the timezone is known.'),
      text_item('Buffered footage', self.buffer_status),
      text_item('Current time', self.clock_status,
                'Uses the device clock, synchronized by the system through the network or GPS. ' +
                'Uses the remembered geographic timezone, or the cellular network offset when available. Continuous GPS is not needed.'),
    ]
    self.scroller = self._child(Scroller(items, spacing=0, line_separator=True))

  def _set_sentry_enabled(self, enabled):
    # Settings persist in any vehicle state; sentryd gates parked capture.
    # Persist the command before refreshing the switch or evaluating process gates.
    self.params.put_bool('SentryEnabled', enabled, block=True)
    if not enabled:
      self.params.remove('SentryManualTrigger')
      device.sentry_motion_until = 0.
    self._enable_toggle.action_item.set_state(enabled)

  def _update_state(self):
    self._enable_toggle.action_item.set_state(setting(self.params, 'SentryEnabled'))
    self._livestream_toggle.action_item.set_state(setting(self.params, 'SentryLivestreamOverride'))

  def toggle(self, key, title, description=None):
    return toggle_item(title, description, initial_state=setting(self.params, key),
                       callback=lambda value: self.params.put_bool(key, value))

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
      return 'now unavailable'
    return f"now {sm['peripheralState'].voltage/1000.:.1f} V"

  def clock_status(self):
    sm = ui_state.sm
    fresh = sm.valid['sentryState'] and 0 <= time.monotonic()-sm.recv_time['sentryState'] < 2.
    return time_status(self.params, sm['sentryState'] if fresh else None)

  def buffer_status(self):
    state = ui_state.sm['sentryRecorderState']
    return f'{state.bufferSeconds:.0f} s / {state.bufferBytes/1048576:.0f} MB' if ui_state.sentry_active else 'Idle'

  def _render(self, rect):
    self.scroller.render(rect)
