import math
import time
import pyray as rl

from openpilot.common.params import Params
from openpilot.cereal import log
from openpilot.selfdrive.ui.ui_state import ui_state, device
from openpilot.system.sentry.config import setting, LOW_RISK, HIGH_RISK
from openpilot.system.ui.lib.application import FontWeight, TextAlignment
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.label import gui_label


class SentryPrompt(Widget):
  """One parking choice per drive, sharing the controller's arming countdown."""
  def __init__(self):
    super().__init__()
    self.params = Params()
    self.opened = False
    self.status = ''
    self.pressed = None
    self.buttons = {}
    self.wifi_default = False
    self.close_at = 0.
    self.battery_status = 'Battery unavailable'

  def update(self, eligible):
    if not eligible or not setting(self.params, 'SentryEnabled'):
      self.close()
      return
    sm, now = ui_state.sm, time.monotonic()
    if not self.opened and not self.params.get_bool('SentryModePrompted'):
      # Choose once per parking prompt. Later network changes cannot override
      # an explicit risk choice or silently rearm a skipped parking session.
      self.wifi_default = (setting(self.params, 'SentrySkipOnWifi') and sm.valid['deviceState'] and
                           0 <= now-sm.recv_time['deviceState'] < 5. and
                           sm['deviceState'].networkType == log.DeviceState.NetworkType.wifi)
      if self.wifi_default:
        self.params.put_bool('SentryOffThisDrive', True, block=True)
        self.close_at = now+setting(self.params, 'SentryArmDelay')
      self.opened = True
      device.sentry_prompt_visible = True
      device._reset_interactive_timeout()

    voltage = sm['peripheralState'].voltage/1000.
    voltage_valid = sm.valid['peripheralState'] and 0 <= now-sm.recv_time['peripheralState'] < 3. and 6 < voltage < 20
    self.battery_status = f'Battery {voltage:.1f} V' if voltage_valid else 'Battery unavailable'
    if self.wifi_default:
      self.status = f'Wi-Fi · Sentry off · Closing in {max(0, math.ceil(self.close_at-now))} s'
      if self.opened and now >= self.close_at:
        self.choose(None)
      return
    fresh = sm.valid['sentryState'] and 0 <= now-sm.recv_time['sentryState'] < 2.
    phase = sm['sentryState'].phase if fresh else 'Starting Sentry'
    self.status = phase.replace('Arming:', 'Arming in')
    if self.opened and fresh and phase in ('Watching', 'Recording'):
      self.choose(None)  # Keep the default/current mode when the actual timer expires.

  def close(self):
    if self.opened:
      self.opened = False
      self.pressed = None
      device.sentry_prompt_visible = False
      device._reset_interactive_timeout()

  def choose(self, mode):
    # X retains the parking default, including Off on Wi-Fi.
    if mode == 'off':
      self.params.put_bool('SentryOffThisDrive', True, block=True)
    elif mode is not None:
      self.params.put('SentryRiskMode', mode, block=True)
      self.params.put_bool('SentryOffThisDrive', False, block=True)
    self.params.put_bool('SentryModePrompted', True, block=True)
    self.close()

  def _update_layout_rects(self):
    r = self._rect
    margin, gap, header = 40, 24, 220
    x, y, width = r.x+margin, r.y+margin+header, r.width-2*margin
    height = r.height-2*margin-header
    top = (height-gap)*2/3
    half = (width-gap)/2
    self.buttons = {
      'close': rl.Rectangle(r.x+r.width-margin-90, r.y+margin, 90, 90),
      LOW_RISK: rl.Rectangle(x, y, half, top),
      HIGH_RISK: rl.Rectangle(x+half+gap, y, half, top),
      'off': rl.Rectangle(x, y+top+gap, width, height-top-gap),
    }

  def _render(self, rect):
    rl.draw_rectangle_rec(rect, rl.Color(15, 18, 21, 255))
    gui_label(rl.Rectangle(rect.x+40, rect.y+40, rect.width-200, 90), self.status, 48,
              rl.WHITE, font_weight=FontWeight.MEDIUM)
    labels = (self.battery_status, f'Battery cutoff {setting(self.params, "SentryCutoffVoltage"):.1f} V',
              f'Time cutoff {setting(self.params, "SentryMaxHours")} h')
    width = (rect.width-80)/3
    for i, text in enumerate(labels):
      gui_label(rl.Rectangle(rect.x+40+i*width, rect.y+150, width-24, 60), text, 36, rl.LIGHTGRAY)
    default_mode = 'off' if self.wifi_default else setting(self.params, 'SentryRiskMode')
    for mode, title, detail, fill, ink in (
      (LOW_RISK, 'Low Risk', 'Sound and vehicle movement', rl.Color(245, 196, 46, 255), rl.BLACK),
      (HIGH_RISK, 'High Risk', 'Camera motion, sound and vehicle movement', rl.Color(190, 35, 42, 255), rl.WHITE),
      ('off', 'Sentry off for this drive', 'Wi-Fi connected' if self.wifi_default else 'Choose again after the next drive',
       rl.Color(71, 184, 104, 255), rl.BLACK),
    ):
      box = self.buttons[mode]
      if mode == default_mode:
        detail += ' · Default'
        # A gentle two-second breath identifies the default without changing
        # the meaning of the yellow/red/green mode colours.
        brightness = .85+.15*math.sin(time.monotonic()*math.tau/2.)
        fill = rl.Color(int(fill.r*brightness), int(fill.g*brightness), int(fill.b*brightness), fill.a)
      rl.draw_rectangle_rounded(box, .08, 12, fill)
      gui_label(rl.Rectangle(box.x+20, box.y+box.height/2-70, box.width-40, 90), title, 64,
                ink, font_weight=FontWeight.BOLD, alignment=TextAlignment.CENTER)
      gui_label(rl.Rectangle(box.x+20, box.y+box.height/2+30, box.width-40, 60), detail, 32,
                ink, alignment=TextAlignment.CENTER)
    close = self.buttons['close']
    rl.draw_line_ex(rl.Vector2(close.x+25, close.y+25), rl.Vector2(close.x+65, close.y+65), 6, rl.WHITE)
    rl.draw_line_ex(rl.Vector2(close.x+65, close.y+25), rl.Vector2(close.x+25, close.y+65), 6, rl.WHITE)

  def _hit(self, pos):
    return next((key for key, rect in self.buttons.items() if rl.check_collision_point_rec(pos, rect)), None)

  def _handle_mouse_press(self, pos):
    self.pressed = self._hit(pos)

  def _handle_mouse_release(self, pos):
    target = self._hit(pos)
    if target is not None and target == self.pressed:
      self.choose(None if target == 'close' else target)
    self.pressed = None
