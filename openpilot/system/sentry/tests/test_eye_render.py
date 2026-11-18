"""Exercise real raylib argument conversion without requiring an OpenGL window."""
import math
from types import SimpleNamespace

import pytest


@pytest.mark.parametrize('gradient_api', ['installed', 'legacy'])
def test_eye_first_visible_frame_and_hidden_frame(monkeypatch, gradient_api):
  monkeypatch.setenv('SCALE', '1')
  import pyray as rl
  from openpilot.selfdrive.ui.layouts import sentry_eye as ui

  state = SimpleNamespace(motion=True, motionX=.8, motionY=.3, motionCamera=2, recordUntilMonoTime=0, soundLevel=0., gyroActive=False)
  device = SimpleNamespace(sentry_eye_visible=False, sentry_motion_until=101.)
  monkeypatch.setattr(ui, 'ui_state', SimpleNamespace(sm={'sentryState': state}))
  monkeypatch.setattr(ui, 'device', device)
  monkeypatch.setattr(ui.time, 'monotonic', lambda: 100.)

  # Keep the installed binding's C argument types/arity. Replace only the native
  # drawing destination with callbacks, so API mismatches still raise normally.
  calls = []
  glows = []
  reflections = []
  transforms = [(0., 0., 0.)]
  functions = {
    'draw_rectangle_rec': 'DrawRectangleRec',
    'draw_circle_gradient': 'DrawCircleGradient',
    'draw_circle_v': 'DrawCircleV',
    'draw_ring': 'DrawRing',
    'draw_ellipse': 'DrawEllipse',
    'rl_push_matrix': 'rlPushMatrix',
    'rl_pop_matrix': 'rlPopMatrix',
    'rl_translatef': 'rlTranslatef',
    'rl_rotatef': 'rlRotatef',
  }
  for python_name, native_name in functions.items():
    signature = rl.ffi.typeof(getattr(rl.rl, native_name))
    if python_name == 'draw_circle_gradient' and gradient_api == 'legacy':
      signature = rl.ffi.typeof('void(*)(int, int, float, struct Color, struct Color)')
    def capture(*args, name=python_name):
      calls.append(name)
      tx, ty, angle = transforms[-1]
      cosine, sine = math.cos(math.radians(angle)), math.sin(math.radians(angle))
      if name == 'rl_push_matrix':
        transforms.append(transforms[-1])
      elif name == 'rl_pop_matrix':
        transforms.pop()
      elif name == 'rl_translatef':
        transforms[-1] = (tx+args[0]*cosine-args[1]*sine, ty+args[0]*sine+args[1]*cosine, angle)
      elif name == 'rl_rotatef':
        transforms[-1] = (tx, ty, angle+args[0])
      if name == 'draw_circle_gradient':
        if gradient_api == 'legacy':
          glows.append((args[0], args[1], args[2]))
        else:
          glows.append((args[0].x, args[0].y, args[1]))
      if name == 'draw_ellipse':
        # Capture the transformed center and rotated ellipse's bounding box.
        reflections.append((tx+args[0]*cosine-args[1]*sine, ty+args[0]*sine+args[1]*cosine,
                            math.hypot(args[2]*cosine, args[3]*sine),
                            math.hypot(args[2]*sine, args[3]*cosine), angle))
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
  assert calls.count('draw_ellipse') == 6  # Three highlights, each with a soft halo.
  assert glows[0][2] == 380  # Original 95 px glow, now four times the diameter.
  # All three lights sit above the lens center, with the middle one highest.
  left, middle, right = reflections[1::2]
  assert left[0]+right[0] == pytest.approx(rect.width)
  assert left[1:4] == pytest.approx(right[1:4])
  assert middle[0] == rect.width/2
  assert middle[1] < left[1] < rect.height/2
  assert -45 < left[4] < -30 and 30 < right[4] < 45
  assert left[4] == pytest.approx(-right[4])
  assert middle[4] == 0
  assert transforms == [(0., 0., 0.)]  # No transform leaks into other UI drawing.
  for x, y in ((0., 0.), (0., 1.), (1., 0.), (1., 1.)):
    eye.x, eye.y = x, y
    glows.clear()
    reflections.clear()
    eye._render(rect)
    for cx, cy, radius in glows:
      assert radius <= cx <= rect.width-radius
      assert radius <= cy <= rect.height-radius
    # The full glow reaches the edge exactly, with no extra border/padding.
    cx, cy, radius = glows[0]
    assert min(cx, rect.width-cx) == radius
    assert min(cy, rect.height-cy) == radius
    # The reflections rotate as one arc toward the display center. Their
    # radii/spacing stay fixed instead of translating across the glass.
    cx, cy, _ = glows[0]
    reflected_x = sum(r[0] for r in reflections)/len(reflections)
    assert (reflected_x-cx)*(rect.width/2-cx) > 0
    assert 5 < abs(reflected_x-cx) < 12  # Subtle arc rotation, not a large sweep.
    for rx, ry, width, height, angle in reflections:
      assert ry+height < cy
      assert ((abs(rx-cx)+width)**2+(abs(ry-cy)+height)**2)**.5 < 44*4
      assert math.hypot(rx-cx, ry-cy) == pytest.approx(29*4, abs=.001)
      # Each long axis remains tangent to the circle, including the middle.
      assert (rx-cx)*math.cos(math.radians(angle))+(ry-cy)*math.sin(math.radians(angle)) == pytest.approx(0., abs=.001)
    left, middle, right = reflections[1::2]
    assert left[4] < 0 < right[4] and 2 < abs(middle[4]) <= 6
    assert middle[4]-left[4] == pytest.approx(36.)
    assert right[4]-middle[4] == pytest.approx(36.)
    assert transforms == [(0., 0., 0.)]

  calls.clear()
  device.sentry_eye_visible = False
  eye._render(rect)
  assert calls == ['draw_rectangle_rec']


@pytest.mark.parametrize('hidden_render,wakeup_at', [(False, 100.2), (False, 110.), (True, 110.)])
def test_sensor_wakeup_starts_centered_then_follows_visible_motion(monkeypatch, hidden_render, wakeup_at):
  monkeypatch.setenv('SCALE', '1')
  from openpilot.selfdrive.ui.layouts import sentry_eye as ui
  clock = [100.]
  state = SimpleNamespace(motion=True, motionX=.8, motionY=.3, motionCamera=2, recordUntilMonoTime=0, soundLevel=0., gyroActive=False)
  device = SimpleNamespace(sentry_eye_visible=True, sentry_motion_until=101.)
  monkeypatch.setattr(ui, 'ui_state', SimpleNamespace(sm={'sentryState': state}))
  monkeypatch.setattr(ui, 'device', device)
  monkeypatch.setattr(ui.time, 'monotonic', lambda: clock[0])
  centers = []
  monkeypatch.setattr(ui, 'draw_glow', lambda center, *args: centers.append((center.x, center.y)))
  for name in ('draw_rectangle_rec', 'draw_circle_v', 'draw_ring', 'draw_ellipse',
               'rl_push_matrix', 'rl_pop_matrix', 'rl_translatef', 'rl_rotatef'):
    monkeypatch.setattr(ui.rl, name, lambda *args: None)
  eye = ui.SentryEye()
  rect = ui.rl.Rectangle(0, 0, 2160, 1080)
  eye._render(rect)
  clock[0] = 100.1
  eye._render(rect)
  assert eye.x > .5 and eye.y < .5
  if hidden_render:
    device.sentry_eye_visible = False
    clock[0] = 102.
    eye._render(rect)
  # Sound/IMU starts a new recording while no visual target exists.
  device.sentry_eye_visible, device.sentry_motion_until = True, 111.
  state.motion, state.recordUntilMonoTime = False, int(125e9)
  clock[0] = wakeup_at
  centers.clear()
  eye._render(rect)
  assert eye.x == eye.y == .5
  assert centers[0] == (1080., 540.)
  state.motion = True
  clock[0] = wakeup_at+.1
  eye._render(rect)
  assert eye.x > .5 and eye.y < .5
  # Brief loss of visual motion within this event holds the last known position.
  previous = eye.x, eye.y
  state.motion = False
  clock[0] = wakeup_at+.2
  eye._render(rect)
  assert (eye.x, eye.y) == previous


def test_eye_maps_cabin_windows_to_opposite_display_edges():
  from openpilot.selfdrive.ui.layouts.sentry_eye import eye_target
  assert eye_target(.5, .5, 1) == (.5, .5)
  # Cabin windows sit well inside the distorted image boundaries. They should
  # use the eye's full screen travel, with the inward camera mirrored.
  assert eye_target(.75, .5, 1) == (0., .5)
  assert eye_target(.25, .5, 1) == (1., .5)
  assert eye_target(.72, .5, 1)[0] < .02
  assert eye_target(.28, .5, 1)[0] > .98
  assert eye_target(.25, .5, 2)[0] < .25
  assert eye_target(.75, .5, 2)[0] > .75
  for camera in (1, 2):
    for x in (0., .25, .5, .75, 1.):
      assert eye_target(x, .5, camera)[0]+eye_target(1-x, .5, camera)[0] == pytest.approx(1.)


def test_sound_brightens_red_edges_pulses_and_decays_without_moving_eye(monkeypatch):
  from openpilot.selfdrive.ui.layouts import sentry_eye as ui
  clock = [100.]
  state = SimpleNamespace(motion=False, motionCamera=2, recordUntilMonoTime=int(150e9), soundLevel=0., gyroActive=False)
  device = SimpleNamespace(sentry_eye_visible=True, sentry_motion_until=151.)
  monkeypatch.setattr(ui, 'ui_state', SimpleNamespace(sm={'sentryState': state}))
  monkeypatch.setattr(ui, 'device', device)
  monkeypatch.setattr(ui.time, 'monotonic', lambda: clock[0])
  rings = []
  for name in ('draw_rectangle_rec', 'draw_circle_v', 'draw_ellipse', 'rl_push_matrix', 'rl_pop_matrix', 'rl_translatef', 'rl_rotatef'):
    monkeypatch.setattr(ui.rl, name, lambda *args: None)
  monkeypatch.setattr(ui, 'draw_glow', lambda *args: None)
  monkeypatch.setattr(ui.rl, 'draw_ring', lambda *args: rings.append(args))
  eye = ui.SentryEye()
  rect = ui.rl.Rectangle(0, 0, 2160, 1080)

  def render(level):
    clock[0] += .05
    state.soundLevel = level
    rings.clear()
    eye._render(rect)
    edge = [r for r in rings if r[1] >= 44*4]
    assert all(r[-1].g == r[-1].b == 0 and r[2] <= 95*4 for r in edge)
    assert eye.x == eye.y == .5
    return sum(r[-1].a for r in edge)

  assert render(0.) == 0
  quiet = max(render(.15) for _ in range(20))
  loud = [render(1.) for _ in range(30)]
  assert min(loud[5:]) > quiet
  assert max(loud[5:])-min(loud[5:]) > 100
  for _ in range(40):
    fade = render(0.)
  assert fade == 0
  render(1.)
  device.sentry_eye_visible = False
  eye._render(rect)
  assert eye.sound_level == 0.


def test_gyro_pulses_nearly_full_height_at_four_hz_and_releases(monkeypatch):
  from openpilot.selfdrive.ui.layouts import sentry_eye as ui
  clock = [100.]
  state = SimpleNamespace(motion=False, motionCamera=2, recordUntilMonoTime=int(150e9), soundLevel=0., gyroActive=True)
  device = SimpleNamespace(sentry_eye_visible=True, sentry_motion_until=151.)
  monkeypatch.setattr(ui, 'ui_state', SimpleNamespace(sm={'sentryState': state}))
  monkeypatch.setattr(ui, 'device', device)
  monkeypatch.setattr(ui.time, 'monotonic', lambda: clock[0])
  glows = []
  monkeypatch.setattr(ui, 'draw_glow', lambda center, radius, inner, outer: glows.append((center, radius, inner)))
  for name in ('draw_rectangle_rec', 'draw_circle_v', 'draw_ring', 'draw_ellipse',
               'rl_push_matrix', 'rl_pop_matrix', 'rl_translatef', 'rl_rotatef'):
    monkeypatch.setattr(ui.rl, name, lambda *args: None)
  eye = ui.SentryEye()
  rect = ui.rl.Rectangle(0, 0, 2160, 1080)
  radii, alphas = [], []
  for _ in range(60):
    clock[0] += .025
    glows.clear()
    eye._render(rect)
    center, radius, inner = glows[0]
    assert center.x == rect.width/2 and center.y == rect.height/2
    assert radius <= center.y <= rect.height-radius
    assert inner.r == 255 and inner.g == inner.b == 0
    radii.append(radius)
    alphas.append(inner.a)
  assert .47*rect.height < max(radii) <= .48*rect.height
  assert max(alphas[20:])-min(alphas[20:]) > 140
  # Count the four peaks during the last second, after the attack settles.
  assert sum(alphas[i-1] < alphas[i] >= alphas[i+1] for i in range(21, 59)) == 4
  state.gyroActive = False
  for _ in range(50):
    clock[0] += .025
    glows.clear()
    eye._render(rect)
  assert eye.gyro_level < .005
  assert len(glows) == 3 and glows[0][1] == 380  # Quiet normal eye, despite recording tail.
  state.gyroActive = True
  device.sentry_eye_visible = False
  eye._render(rect)
  assert eye.gyro_level == eye.gyro_phase == 0.
