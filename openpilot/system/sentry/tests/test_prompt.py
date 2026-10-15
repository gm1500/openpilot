from types import SimpleNamespace

import pytest
import pyray as rl

from openpilot.common.params import Params, ParamKeyFlag
from openpilot.cereal import log
from openpilot.system.sentry.config import LOW_RISK, HIGH_RISK, setting


@pytest.fixture
def prompt(monkeypatch, tmp_path):
  monkeypatch.setenv('SCALE', '1')
  from openpilot.selfdrive.ui.layouts import sentry_prompt as ui
  params = Params(str(tmp_path/'params'))
  params.put_bool('SentryEnabled', True)
  sm = {'sentryState': SimpleNamespace(phase='Arming: 30 s'),
        'deviceState': SimpleNamespace(networkType=log.DeviceState.NetworkType.none),
        'peripheralState': SimpleNamespace(voltage=12400)}
  sm = type('SM', (dict,), {'valid': dict.fromkeys(sm, True), 'recv_time': dict.fromkeys(sm, 100.)})(sm)
  callbacks = []
  device = SimpleNamespace(sentry_prompt_visible=False, add_interactive_timeout_callback=callbacks.append)
  monkeypatch.setattr(ui, 'Params', lambda: params)
  monkeypatch.setattr(ui, 'ui_state', SimpleNamespace(sm=sm))
  monkeypatch.setattr(ui, 'device', device)
  monkeypatch.setattr(ui.time, 'monotonic', lambda: 100.)
  view = ui.SentryPrompt()
  view.set_rect(rl.Rectangle(0, 0, 2160, 1080))
  return view, params, sm, device


@pytest.mark.parametrize('choice,risk,off', [(LOW_RISK, LOW_RISK, False), (HIGH_RISK, HIGH_RISK, False),
                                           ('off', LOW_RISK, True), ('close', LOW_RISK, False)])
def test_choices_close_and_keep_master_enable(prompt, choice, risk, off):
  view, params, sm, device = prompt
  view.update(True)
  assert view.opened and device.sentry_prompt_visible
  assert view.status == 'Arming in 30 s'
  box = view.buttons[choice]
  pos = rl.Vector2(box.x+box.width/2, box.y+box.height/2)
  view._handle_mouse_press(pos)
  view._handle_mouse_release(pos)
  assert not view.opened and not device.sentry_prompt_visible
  assert setting(params, 'SentryRiskMode') == risk
  assert params.get_bool('SentryOffThisDrive') == off
  assert params.get_bool('SentryEnabled')
  view.update(True)
  assert not view.opened  # Going back into settings does not reopen the prompt.


def test_countdown_waits_for_actual_arming_and_stale_state_cannot_dismiss(prompt):
  view, params, sm, device = prompt
  view.update(True)
  for phase in ('Starting cameras', 'Camera warmup: 10 s', 'Arming: 1 s'):
    sm['sentryState'].phase = phase
    view.update(True)
    assert view.opened
  sm['sentryState'].phase = 'Watching'
  sm.valid['sentryState'] = False
  view.update(True)
  assert view.opened
  sm.valid['sentryState'] = True
  view.update(True)
  assert not view.opened and params.get_bool('SentryModePrompted')
  assert setting(params, 'SentryRiskMode') == LOW_RISK


def test_next_drive_resets_off_override_and_high_risk_and_onroad_closes_prompt(prompt):
  view, params, sm, device = prompt
  view.update(True)
  view.choose('off')
  params.put('SentryRiskMode', HIGH_RISK, block=True)
  params.clear_all(ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION)
  view.update(False)
  assert not view.opened
  assert not params.get_bool('SentryOffThisDrive') and not params.get_bool('SentryModePrompted')
  assert setting(params, 'SentryRiskMode') == LOW_RISK
  view.update(True)
  assert view.opened
  view.update(False)
  assert not view.opened and not device.sentry_prompt_visible


@pytest.mark.parametrize('mode', [None, 'off', LOW_RISK, HIGH_RISK])
def test_wifi_default_off_can_be_overridden_and_network_changes_do_not_reselect(prompt, mode):
  view, params, sm, device = prompt
  sm['deviceState'].networkType = log.DeviceState.NetworkType.wifi
  view.update(True)
  assert view.wifi_default and params.get_bool('SentryOffThisDrive')
  assert view.status == 'Wi-Fi · Sentry off · Closing in 30 s'
  view.choose(mode)
  assert params.get_bool('SentryOffThisDrive') == (mode in (None, 'off'))
  if mode in (LOW_RISK, HIGH_RISK):
    assert setting(params, 'SentryRiskMode') == mode
  sm['deviceState'].networkType = log.DeviceState.NetworkType.none
  view.update(True)
  sm['deviceState'].networkType = log.DeviceState.NetworkType.wifi
  view.update(True)
  assert not view.opened and params.get_bool('SentryOffThisDrive') == (mode in (None, 'off'))


@pytest.mark.parametrize('blocker', ['disabled', 'stale', 'invalid'])
def test_wifi_skip_requires_enabled_option_and_fresh_connection(prompt, blocker):
  view, params, sm, device = prompt
  sm['deviceState'].networkType = log.DeviceState.NetworkType.wifi
  if blocker == 'disabled':
    params.put_bool('SentrySkipOnWifi', False, block=True)
  elif blocker == 'stale':
    sm.recv_time['deviceState'] = 90.
  else:
    sm.valid['deviceState'] = False
  view.update(True)
  assert not view.wifi_default and not params.get_bool('SentryOffThisDrive')


def test_wifi_default_dismisses_without_cameras_and_resets_on_next_drive(prompt, monkeypatch):
  from openpilot.selfdrive.ui.layouts import sentry_prompt as ui
  view, params, sm, device = prompt
  sm['deviceState'].networkType = log.DeviceState.NetworkType.wifi
  view.update(True)
  sm['sentryState'].phase = 'Off for this drive'
  sm.valid['sentryState'] = False
  monkeypatch.setattr(ui.time, 'monotonic', lambda: 130.)
  view.update(True)
  assert not view.opened and params.get_bool('SentryModePrompted') and params.get_bool('SentryOffThisDrive')
  params.clear_all(ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION)
  view.update(False)
  sm['deviceState'].networkType = log.DeviceState.NetworkType.none
  sm.recv_time['deviceState'] = 130.
  view.update(True)
  assert view.opened and not view.wifi_default and not params.get_bool('SentryOffThisDrive')


def test_prompt_battery_updates_and_stale_voltage_is_unavailable(prompt):
  view, params, sm, device = prompt
  view.update(True)
  assert view.battery_status == 'Battery 12.4 V'
  sm['peripheralState'].voltage = 11700
  view.update(True)
  assert view.battery_status == 'Battery 11.7 V'
  sm.recv_time['peripheralState'] = 90.
  view.update(True)
  assert view.battery_status == 'Battery unavailable'


def test_layout_and_drag_between_buttons_does_not_select(prompt):
  view, params, sm, device = prompt
  low, high, off, close = (view.buttons[k] for k in (LOW_RISK, HIGH_RISK, 'off', 'close'))
  assert low.width == high.width and low.height == high.height
  assert low.height == pytest.approx(off.height*2)
  assert off.x == low.x and off.width == high.x+high.width-low.x
  assert close.y < low.y and close.x+close.width <= 2160
  view.update(True)
  view._handle_mouse_press(rl.Vector2(low.x+20, low.y+20))
  view._handle_mouse_release(rl.Vector2(high.x+20, high.y+20))
  assert view.opened and setting(params, 'SentryRiskMode') == LOW_RISK


def test_render_bindings_and_countdown_header_alignment(prompt, monkeypatch):
  from openpilot.selfdrive.ui.layouts import sentry_prompt as ui
  view, params, sm, device = prompt
  view.update(True)
  calls, labels = [], []
  for python_name, native_name in (
    ('draw_rectangle_rec', 'DrawRectangleRec'),
    ('draw_rectangle_rounded', 'DrawRectangleRounded'),
    ('draw_line_ex', 'DrawLineEx'),
  ):
    signature = rl.ffi.typeof(getattr(rl.rl, native_name))
    def capture(*args, name=python_name):
      calls.append(name)
    monkeypatch.setattr(rl, python_name, rl._wrap_function(rl.ffi.callback(signature, capture)))
  monkeypatch.setattr(ui, 'gui_label', lambda rect, text, size, *args, **kwargs: labels.append((rect, text, size)))
  view._render(view.rect)
  assert calls.count('draw_rectangle_rounded') == 4 and calls.count('draw_line_ex') == 2
  header, text, font_size = labels[0]
  close = view.buttons['close']
  assert text == 'Arming in 30 s'
  assert header.x == view.buttons[LOW_RISK].x
  assert header.y+header.height/2 == close.y+close.height/2
  assert header.x+header.width < close.x
  cutoff, text, size = labels[1]
  assert text == 'Time cutoff 4 h' and size == font_size
  assert cutoff.y == header.y and cutoff.height == header.height
  assert header.x+header.width < cutoff.x and cutoff.x+cutoff.width < close.x
  battery, text, _ = labels[2]
  assert text == 'Battery 12.4 V   ·   Cutoff 11.8 V'
  assert battery.y > header.y+header.height and battery.y+battery.height < view.buttons[LOW_RISK].y


@pytest.mark.parametrize('wifi,changing', [(False, 0), (True, 2)])
def test_only_default_tile_background_pulses(prompt, monkeypatch, wifi, changing):
  from openpilot.selfdrive.ui.layouts import sentry_prompt as ui
  view, params, sm, device = prompt
  if wifi:
    sm['deviceState'].networkType = log.DeviceState.NetworkType.wifi
  view.update(True)
  for name in ('draw_rectangle_rec', 'draw_line_ex'):
    monkeypatch.setattr(rl, name, lambda *args: None)
  monkeypatch.setattr(ui, 'gui_label', lambda *args, **kwargs: None)
  fills = []
  def capture(rect, rounding, segments, fill):
    if rect.y >= view.buttons[LOW_RISK].y:
      fills.append((fill.r, fill.g, fill.b))
  monkeypatch.setattr(rl, 'draw_rectangle_rounded', capture)
  for t in (120.3, 120.9, 121.5):
    monkeypatch.setattr(ui.time, 'monotonic', lambda t=t: t)
    view._render(view.rect)
  assert [fills[i] != fills[i+3] for i in range(3)] == [i == changing for i in range(3)]
  assert fills[:3] == fills[6:]
  assert max(fills[changing+3]) == pytest.approx(max(fills[changing])*.5, abs=1)


@pytest.mark.parametrize('wifi', [False, True])
@pytest.mark.parametrize('armed_at', [105., 115.])
def test_prompt_uses_existing_screen_deadline_and_arming_does_not_wake_it(prompt, monkeypatch, wifi, armed_at):
  from openpilot.selfdrive.ui import ui_state as ui
  from openpilot.selfdrive.ui.layouts import sentry_prompt
  _, params, sm, _ = prompt
  clock = [100.]
  sm['deviceState'].networkType = log.DeviceState.NetworkType.wifi if wifi else log.DeviceState.NetworkType.none
  sm['sentryState'] = SimpleNamespace(phase='Camera warmup: 10 s', eyeEnabled=True, motion=False, sensorActive=False)
  context = SimpleNamespace(ignition=False, started=False, sentry_active=not wifi, sm=sm)
  monkeypatch.setattr(ui, 'ui_state', context)
  monkeypatch.setattr(sentry_prompt, 'ui_state', context)
  monkeypatch.setattr(ui, 'gui_app', SimpleNamespace(target_fps=20, mouse_events=[], big_ui=lambda: True))
  monkeypatch.setattr(ui, 'PC', False)
  monkeypatch.setattr(ui.time, 'monotonic', lambda: clock[0])
  device = ui.Device()
  device.set_override_interactive_timeout(10)
  monkeypatch.setattr(device, '_set_awake', lambda awake: setattr(device, '_awake', awake))
  monkeypatch.setattr(sentry_prompt, 'device', device)
  view = sentry_prompt.SentryPrompt()
  params.put('SentryArmDelay', 15, block=True)
  clock[0] = 102.
  view.update(True)
  assert view.opened and device._interaction_time == 110.  # Opening spends the existing budget.
  for t in (104., 105., 109.9, 110.1, 115., 120.):
    clock[0] = t
    sm.recv_time['sentryState'] = t
    sm['sentryState'].phase = 'Watching' if t >= armed_at else 'Arming: 5 s'
    device._update_wakefulness()  # Timer must dismiss the prompt even without a render.
    view.update(True)
    assert device._interaction_time == 110.  # No second menu timeout on dismissal.
    assert device.awake == (t < 110.)
    if t > 110.:
      assert not view.opened and not device.sentry_prompt_visible
      assert params.get_bool('SentryModePrompted')
  assert params.get_bool('SentryOffThisDrive') == wifi


def test_touch_on_prompt_renews_normal_interaction_timer(prompt, monkeypatch):
  from openpilot.selfdrive.ui import ui_state as ui
  from openpilot.selfdrive.ui.layouts import sentry_prompt
  _, params, sm, _ = prompt
  clock = [100.]
  sm['sentryState'] = SimpleNamespace(phase='Arming: 30 s', eyeEnabled=True, motion=False, sensorActive=False)
  context = SimpleNamespace(ignition=False, started=False, sentry_active=True, sm=sm)
  app = SimpleNamespace(target_fps=20, mouse_events=[], big_ui=lambda: True)
  monkeypatch.setattr(ui, 'ui_state', context)
  monkeypatch.setattr(sentry_prompt, 'ui_state', context)
  monkeypatch.setattr(ui, 'gui_app', app)
  monkeypatch.setattr(ui, 'PC', False)
  monkeypatch.setattr(ui.time, 'monotonic', lambda: clock[0])
  device = ui.Device()
  device.set_override_interactive_timeout(10)
  monkeypatch.setattr(device, '_set_awake', lambda awake: setattr(device, '_awake', awake))
  monkeypatch.setattr(sentry_prompt, 'device', device)
  view = sentry_prompt.SentryPrompt()
  view.update(True)
  clock[0] = 105.
  app.mouse_events = [SimpleNamespace(left_down=True)]
  device._update_wakefulness()
  assert device._interaction_time == 115.
  app.mouse_events = []
  clock[0] = 110.1
  device._update_wakefulness()
  assert view.opened and device.awake
  clock[0] = 115.1
  device._update_wakefulness()
  assert not view.opened and not device.awake and params.get_bool('SentryModePrompted')
