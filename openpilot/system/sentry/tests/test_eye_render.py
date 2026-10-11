"""Exercise real raylib argument conversion without requiring an OpenGL window."""
from types import SimpleNamespace

import pytest


@pytest.mark.parametrize('gradient_api', ['installed', 'legacy'])
def test_eye_first_visible_frame_and_hidden_frame(monkeypatch, gradient_api):
  monkeypatch.setenv('SCALE', '1')
  import pyray as rl
  from openpilot.selfdrive.ui.layouts import sentry_eye as ui

  state = SimpleNamespace(motion=True, motionX=.8, motionY=.3, motionCamera=2)
  device = SimpleNamespace(sentry_eye_visible=False, sentry_motion_until=101.)
  monkeypatch.setattr(ui, 'ui_state', SimpleNamespace(sm={'sentryState': state}))
  monkeypatch.setattr(ui, 'device', device)
  monkeypatch.setattr(ui.time, 'monotonic', lambda: 100.)

  # Keep the installed binding's C argument types/arity. Replace only the native
  # drawing destination with callbacks, so API mismatches still raise normally.
  calls = []
  functions = {
    'draw_rectangle_rec': 'DrawRectangleRec',
    'draw_circle_gradient': 'DrawCircleGradient',
    'draw_circle_v': 'DrawCircleV',
    'draw_ring': 'DrawRing',
    'draw_ellipse': 'DrawEllipse',
  }
  for python_name, native_name in functions.items():
    signature = rl.ffi.typeof(getattr(rl.rl, native_name))
    if python_name == 'draw_circle_gradient' and gradient_api == 'legacy':
      signature = rl.ffi.typeof('void(*)(int, int, float, struct Color, struct Color)')
    callback = rl.ffi.callback(signature, lambda *args, name=python_name: calls.append(name))
    monkeypatch.setattr(rl, python_name, rl._wrap_function(callback))

  eye = ui.SentryEye()
  rect = rl.Rectangle(0, 0, 2160, 1080)
  eye._render(rect)
  assert calls == ['draw_rectangle_rec']

  calls.clear()
  device.sentry_eye_visible = True
  eye._render(rect)
  assert calls.count('draw_circle_gradient') == 3
  assert calls.count('draw_ellipse') == 1

  calls.clear()
  device.sentry_eye_visible = False
  eye._render(rect)
  assert calls == ['draw_rectangle_rec']
