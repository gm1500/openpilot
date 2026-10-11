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
  glows = []
  reflections = []
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
    def capture(*args, name=python_name):
      calls.append(name)
      if name == 'draw_circle_gradient':
        if gradient_api == 'legacy':
          glows.append((args[0], args[1], args[2]))
        else:
          glows.append((args[0].x, args[0].y, args[1]))
      if name == 'draw_ellipse':
        reflections.append((args[0], args[1], args[2], args[3]))
    callback = rl.ffi.callback(signature, capture)
    monkeypatch.setattr(rl, python_name, rl._wrap_function(callback))

  eye = ui.SentryEye()
  rect = rl.Rectangle(0, 0, 2160, 1080)
  eye._render(rect)
  assert calls == ['draw_rectangle_rec']

  calls.clear()
  device.sentry_eye_visible = True
  eye._render(rect)
  assert calls.count('draw_circle_gradient') == 3
  assert calls.count('draw_ellipse') == 12  # Six highlights, each with a soft halo.
  assert glows[0][2] == 380  # Original 95 px glow, now four times the diameter.
  # At the center, the three lights on either side have identical mirrored shapes.
  left, right = reflections[1:6:2], reflections[7:12:2]
  for l, r in zip(left, right, strict=True):
    assert l[0]+r[0] == rect.width
    assert l[1:] == r[1:]
  for x, y in ((0., 0.), (0., 1.), (1., 0.), (1., 1.)):
    eye.x, eye.y = x, y
    glows.clear()
    reflections.clear()
    eye._render(rect)
    for cx, cy, radius in glows:
      assert radius <= cx <= rect.width-radius
      assert radius <= cy <= rect.height-radius
    # Reflections follow the display position toward its center, staying inside
    # the lens even at the corners. The red light itself stays at the eye center.
    cx, cy, _ = glows[0]
    reflected_x = sum(r[0] for r in reflections)/len(reflections)
    reflected_y = sum(r[1] for r in reflections)/len(reflections)
    assert (reflected_x-cx)*(rect.width/2-cx) > 0
    assert (reflected_y-cy)*(rect.height/2-cy) > 0
    for rx, ry, width, height in reflections:
      assert ((abs(rx-cx)+width)**2+(abs(ry-cy)+height)**2)**.5 < 44*4

  calls.clear()
  device.sentry_eye_visible = False
  eye._render(rect)
  assert calls == ['draw_rectangle_rec']
