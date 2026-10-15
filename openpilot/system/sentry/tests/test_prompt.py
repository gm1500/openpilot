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
  device = SimpleNamespace(sentry_prompt_visible=False, _reset_interactive_timeout=lambda: None)
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
  monkeypatch.setattr(ui, 'gui_label', lambda rect, text, *args, **kwargs: labels.append((rect, text)))
  view._render(view.rect)
  assert calls.count('draw_rectangle_rounded') == 3 and calls.count('draw_line_ex') == 2
  header, text = labels[0]
  close = view.buttons['close']
  assert text == 'Arming in 30 s'
  assert header.x == view.buttons[LOW_RISK].x
  assert header.y+header.height/2 == close.y+close.height/2
  assert header.x+header.width < close.x
  assert [text for _, text in labels[1:4]] == ['Battery 12.4 V', 'Battery cutoff 11.8 V', 'Time cutoff 4 h']
  for rect, _ in labels[1:4]:
    assert rect.y > header.y+header.height and rect.y+rect.height < view.buttons[LOW_RISK].y


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
  monkeypatch.setattr(rl, 'draw_rectangle_rounded', lambda rect, rounding, segments, fill: fills.append((fill.r, fill.g, fill.b)))
  for t in (100.5, 101.5):
    monkeypatch.setattr(ui.time, 'monotonic', lambda t=t: t)
    view._render(view.rect)
  assert [fills[i] != fills[i+3] for i in range(3)] == [i == changing for i in range(3)]
