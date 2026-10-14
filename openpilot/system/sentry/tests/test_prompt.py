from types import SimpleNamespace

import pytest
import pyray as rl

from openpilot.common.params import Params, ParamKeyFlag
from openpilot.system.sentry.config import LOW_RISK, HIGH_RISK, setting


@pytest.fixture
def prompt(monkeypatch, tmp_path):
  monkeypatch.setenv('SCALE', '1')
  from openpilot.selfdrive.ui.layouts import sentry_prompt as ui
  params = Params(str(tmp_path/'params'))
  params.put_bool('SentryEnabled', True)
  sm = {'sentryState': SimpleNamespace(phase='Arming: 30 s')}
  sm = type('SM', (dict,), {'valid': {'sentryState': True}, 'recv_time': {'sentryState': 100.}})(sm)
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
  params.put('SentryRiskMode', HIGH_RISK)
  params.clear_all(ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION)
  view.update(False)
  assert not view.opened
  assert not params.get_bool('SentryOffThisDrive') and not params.get_bool('SentryModePrompted')
  assert setting(params, 'SentryRiskMode') == LOW_RISK
  view.update(True)
  assert view.opened
  view.update(False)
  assert not view.opened and not device.sentry_prompt_visible


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
