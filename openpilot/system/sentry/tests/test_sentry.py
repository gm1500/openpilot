import json
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.system.sentry import config
from openpilot.system.sentry.location import parked_location, gps_location
from openpilot.system.sentry.motion import MotionDetector


class FakeParams(dict):
  get_bool = dict.get


def warm(detector, image):
  for t in np.arange(.2, 3., .2):
    assert not detector.update(image, float(t)).active


def test_exposure_and_noise_do_not_trigger():
  d = MotionDetector()
  base = np.full((80, 128), 70, dtype=np.float32)
  warm(d, base)
  rng = np.random.default_rng(2)
  for i in range(30):
    image = base+70+rng.normal(0, 2, base.shape)
    assert not d.update(image, 3.+i*.2, 100).active


def test_local_motion_tracks_position_and_needs_persistence():
  d = MotionDetector()
  base = np.full((80, 128), 70, dtype=np.float32)
  warm(d, base)
  image = base.copy()
  image[20:40, 85:110] += 100
  assert not d.update(image, 3.).active
  assert not d.update(image, 3.2).active
  result = d.update(image, 3.4)
  assert result.active
  assert .7 < result.x < .85 and .3 < result.y < .5
  # Loss of camera continuity re-arms instead of comparing stale pictures.
  assert not d.update(image, 5.).active


def test_sensitivity_detects_smaller_changes():
  base = np.full((80, 128), 70, dtype=np.float32)
  image = base.copy()
  image[20:30, 85:97] += 20
  result = []
  for sensitivity in (1, 100):
    d = MotionDetector()
    warm(d, base)
    for i in range(3):
      m = d.update(image, 3.+i*.2, sensitivity)
    result.append(m.active)
  assert result == [False, True]


def test_parked_location_identity_age_accuracy_and_provenance():
  position = {'version': 1, 'vehicle': 'car-a', 'latitude': 53.5, 'longitude': -113.5, 'accuracy': 7., 'savedAt': 100.}
  result = parked_location(position, 'car-a', 200.)
  assert result['source'] == 'slcParked' and result['savedAt'] == 100.
  assert parked_location(position, 'car-b', 200.) is None
  for changes, now in (({}, 100000.), ({'accuracy': 100}, 200.), ({'latitude': float('nan')}, 200.), ({'savedAt': 201}, 200.)):
    assert parked_location(position | changes, 'car-a', now) is None
  assert parked_location(None, 'car-a', 200.) is None


def test_snapshot_requires_recent_accurate_fix():
  gps = SimpleNamespace(hasFix=True, latitude=53.5, longitude=-113.5, horizontalAccuracy=5.)
  assert gps_location(gps, 10., 12., 100.)['source'] == 'gpsSnapshot'
  assert gps_location(gps, 10., 14., 100.) is None
  gps.hasFix = False
  assert gps_location(gps, 10., 12., 100.) is None


def test_expired_lease_never_keeps_cameras_or_gps_alive(tmp_path, monkeypatch):
  path = tmp_path/'runtime.json'
  monkeypatch.setattr(config, 'RUNTIME_PATH', path)
  config.write_runtime({'monoTime': 10., 'capture': True, 'gps': True})
  assert config.read_runtime(11.)['capture']
  assert config.read_runtime(9.) == {}
  assert config.read_runtime(13.) == {}
  path.write_text('corrupt')
  assert config.read_runtime(11.) == {}
  path.write_text(json.dumps({'monoTime': float('nan'), 'capture': True}))
  assert config.read_runtime(11.) == {}


def test_settings_clamp_and_default():
  assert not config.setting(FakeParams(), 'SentryEnabled')
  assert config.setting(FakeParams(SentrySensitivity=400), 'SentrySensitivity') == 100
  assert config.setting(FakeParams(SentryCutoffVoltage=float('nan')), 'SentryCutoffVoltage') == 11.8
  assert config.setting(FakeParams(SentryCutoffVoltage=12.3), 'SentryCutoffVoltage') == 11.8
  assert config.setting(FakeParams(SentryCutoffVoltage=10.), 'SentryCutoffVoltage') == 11.0
  from openpilot.system.hardware.power_monitoring import VBATT_PAUSE_CHARGING
  assert config.DEFAULTS['SentryCutoffVoltage'] == VBATT_PAUSE_CHARGING


def test_battery_override_and_ignition(monkeypatch, tmp_path):
  from openpilot.system.hardware import power_monitoring as power
  from openpilot.common.params import Params
  params = Params(str(tmp_path/'params'))
  params.put_bool('SentryEnabled', True, block=True)
  params.put_bool('DisablePowerDown', True, block=True)
  p = power.PowerMonitoring()
  p.params = params
  p.car_voltage_mV = 11700
  monkeypatch.setattr(power.time, 'monotonic', lambda: 1000.)
  monkeypatch.setattr(power, 'read_runtime', lambda now: {'session': True})
  assert p.should_shutdown(False, True, 0., True)
  assert not p.should_shutdown(True, True, 0., True)
  p.car_voltage_mV = 11900
  assert not p.should_shutdown(False, True, 0., True)
  p.power_used_uWh = 61e6
  assert not p.should_shutdown(False, True, 0., True)  # no hidden energy budget
  params.put('SentryCutoffVoltage', 11., block=True)
  p.car_voltage_mV = 11100
  assert not p.should_shutdown(False, True, 0., True)
  p.car_voltage_mV = 10900
  assert p.should_shutdown(False, True, 0., True)
  monkeypatch.setattr(power, 'read_runtime', lambda now: {})
  assert not p.should_shutdown(False, True, 0., True)  # stock DisablePowerDown restored


def test_eye_touch_suppression_lasts_until_next_timeout(monkeypatch):
  # SCALE avoids monitor probing when importing the UI in a headless test process.
  monkeypatch.setenv('SCALE', '1')
  from openpilot.selfdrive.ui import ui_state as ui
  clock = [100.]
  state = SimpleNamespace(eyeEnabled=True, motion=True, motionCamera=2)
  monkeypatch.setattr(ui, 'ui_state', SimpleNamespace(ignition=False, started=False, sentry_active=True, sm={'sentryState': state}))
  app = SimpleNamespace(target_fps=20, mouse_events=[], big_ui=lambda: True)
  monkeypatch.setattr(ui, 'gui_app', app)
  monkeypatch.setattr(ui.time, 'monotonic', lambda: clock[0])
  device = ui.Device()
  monkeypatch.setattr(device, '_set_awake', lambda _: None)
  device._interaction_time = 90.
  device._update_wakefulness()
  assert device.sentry_eye_visible
  app.mouse_events = [SimpleNamespace(left_down=True)]
  device._update_wakefulness()
  assert not device.sentry_screen and not device.sentry_eye_visible
  app.mouse_events = []
  for t in (100.1, 110., 129.9):
    clock[0] = t
    device._update_wakefulness()
    assert not device.sentry_eye_visible  # continuous motion must not interrupt settings
  clock[0] = 130.1
  device._update_wakefulness()
  assert device.sentry_eye_visible
  state.motion = False
  clock[0] = 131.2
  device._update_wakefulness()
  assert not device.sentry_eye_visible


@pytest.mark.parametrize('stop_control', ['button', 'toggle'])
@pytest.mark.parametrize('ignition,started', [(False, False), (True, False), (False, True)])
def test_settings_can_stop_sentry_and_only_reenable_offroad(monkeypatch, tmp_path, stop_control, ignition, started):
  monkeypatch.setenv('SCALE', '1')
  import pyray as rl
  from openpilot.common.params import Params
  from openpilot.selfdrive.ui import ui_state
  from openpilot.selfdrive.ui.layouts.settings import sentry as ui
  from openpilot.system.ui.widgets import list_view

  params = Params(str(tmp_path/'params'))
  params.put_bool('SentryEnabled', True, block=True)
  params.put_bool('SentryManualTrigger', True, block=True)

  class SubMaster(dict):
    valid = {'sentryState': True, 'peripheralState': True}
    recv_time = dict.fromkeys(valid, ui.time.monotonic())

  class State(SimpleNamespace):
    sentry_active = ui_state.UIState.sentry_active

  sm = SubMaster(sentryState=SimpleNamespace(capture=True, phase='Recording', locationValid=False),
                 sentryRecorderState=SimpleNamespace(bufferSeconds=120, bufferBytes=1000),
                 peripheralState=SimpleNamespace(voltage=12000))
  state = State(params=params, sm=sm, ignition=ignition, started=started)
  device = SimpleNamespace(sentry_motion_until=100.)
  monkeypatch.setattr(ui, 'Params', lambda: params)
  monkeypatch.setattr(ui, 'ui_state', state)
  monkeypatch.setattr(ui, 'device', device)
  # Construct the real settings rows without an OpenGL context.
  monkeypatch.setattr(ui.gui_app, 'font', lambda *args: rl.Font())
  monkeypatch.setattr(list_view, 'measure_text_cached', lambda font, text, size: rl.Vector2(len(text)*20, 40))
  layout = ui.SentryLayout()
  assert layout._stop_button.action_item.enabled
  assert layout._enable_toggle.action_item.enabled
  assert layout._enable_toggle.action_item.get_state()
  assert state.sentry_active == (not ignition and not started)

  if stop_control == 'button':
    layout._stop_button.callback()
  else:
    layout._enable_toggle.action_item.toggle._handle_mouse_release(rl.Vector2())
  assert not params.get_bool('SentryEnabled')
  assert not params.get_bool('SentryManualTrigger')
  assert not layout._enable_toggle.action_item.get_state()
  assert not layout._stop_button.action_item.enabled
  assert layout._stop_button.action_item.value == 'Off'
  assert device.sentry_motion_until == 0.
  assert not state.sentry_active  # A fresh capture=true message cannot keep the eye alive.

  assert layout._enable_toggle.action_item.enabled == (not ignition and not started)
  layout._set_sentry_enabled(True)
  assert params.get_bool('SentryEnabled') == (not ignition and not started)
  state.ignition = state.started = False
  layout._set_sentry_enabled(True)
  assert params.get_bool('SentryEnabled') and layout._enable_toggle.action_item.get_state()
  # Reflect an external change, e.g. a stop issued over SSH, on the next UI update.
  params.put_bool('SentryEnabled', False, block=True)
  layout._update_state()
  assert not layout._enable_toggle.action_item.get_state()


def test_camera_buttons_toggle_independently_and_keep_personality_exclusive(monkeypatch):
  monkeypatch.setenv('SCALE', '1')
  import pyray as rl
  from openpilot.selfdrive.ui.layouts.settings import sentry as ui
  from openpilot.system.ui.widgets.list_view import MultipleButtonAction

  monkeypatch.setattr(ui.gui_app, 'font', lambda *args: None)
  state = SimpleNamespace(ignition=False)
  monkeypatch.setattr(ui, 'ui_state', state)

  class Params(dict):
    def put_bool(self, key, value):
      self[key] = value

  params = Params()
  buttons = ui.SentryCameraButtons(params)
  buttons.set_rect(rl.Rectangle(0, 0, 670, 170))
  assert all(buttons.is_button_selected(i) for i in range(3))
  buttons._handle_mouse_release(rl.Vector2(335, 85))
  assert [buttons.is_button_selected(i) for i in range(3)] == [True, False, True]
  buttons._handle_mouse_release(rl.Vector2(335, 85))
  assert all(buttons.is_button_selected(i) for i in range(3))
  for x in (105, 335, 565):
    buttons._handle_mouse_release(rl.Vector2(x, 85))
  assert not any(buttons.is_button_selected(i) for i in range(3))
  state.ignition = True
  buttons._handle_mouse_release(rl.Vector2(105, 85))
  assert not any(buttons.is_button_selected(i) for i in range(3))
  selected = []
  personality = MultipleButtonAction(['A', 'B', 'C'], 210, callback=selected.append)
  personality.select_button(2)
  assert selected == [2]
  assert [personality.is_button_selected(i) for i in range(3)] == [False, False, True]


def test_process_gates_do_not_start_driving_controls(monkeypatch):
  from openpilot.system.manager import process_config as processes
  monkeypatch.setattr(config, 'read_runtime', lambda: {'capture': True, 'gps': True})
  params = FakeParams(SentryEnabled=True)
  assert config.sentry_capture_requested(False, params, None)
  assert not config.sentry_capture_requested(True, params, None)
  assert config.sentry_gps_requested(False, params, None)
  params['SentryEnabled'] = False
  # The master switch overrides a capture/GPS lease even before sentryd updates it.
  assert not config.sentry_capture_requested(False, params, None)
  assert not config.sentry_gps_requested(False, params, None)
  assert not processes.sentry_encoder(False, params, None)
  for name in ('modeld', 'controlsd', 'card', 'selfdrived', 'plannerd', 'locationd'):
    assert not processes.managed_processes[name].should_run(False, FakeParams(), SimpleNamespace(notCar=False))
  monkeypatch.setattr(processes.managed_processes['sentry_encoderd'], 'proc', SimpleNamespace(is_alive=lambda: True))
  assert not processes.driving_encoder(True, None, None)
  monkeypatch.setattr(processes.managed_processes['sentry_encoderd'], 'proc', None)
  assert processes.driving_encoder(True, None, None)


def test_uploader_does_not_unlock_active_events(tmp_path, monkeypatch):
  import os
  from openpilot.system.loggerd.uploader import clear_locks
  from openpilot.system.sentry import storage
  route = tmp_path/'route--0'
  route.mkdir()
  (route/'.sentry').write_text('1')
  lock = route/'sentry.lock'
  lock.write_text(str(os.getpid()))
  (route/'fcamera.hevc.lock').touch()
  clear_locks(str(tmp_path))
  assert lock.exists() and (route/'fcamera.hevc.lock').exists()
  monkeypatch.setattr(storage.os, 'kill', lambda *args: (_ for _ in ()).throw(ProcessLookupError()))
  clear_locks(str(tmp_path))
  assert not list(route.glob('*.lock'))


def test_deleter_keeps_all_event_segments(tmp_path, monkeypatch):
  from openpilot.system.loggerd import deleter
  monkeypatch.setattr(deleter.Paths, 'log_root', lambda: str(tmp_path))
  monkeypatch.setattr(deleter, 'get_available_bytes', lambda **kwargs: 0)
  monkeypatch.setattr(deleter, 'get_available_percent', lambda **kwargs: 0)
  for i in range(8):
    route = tmp_path/f'00000000--0123456789--{i}'
    route.mkdir()
    (route/'.sentry').touch()
  ordinary = tmp_path/'00000001--0123456789--0'
  ordinary.mkdir()
  assert deleter.deleter_step() == (True, str(ordinary))
  assert deleter.deleter_step() == (True, None)
  assert len(list(tmp_path.iterdir())) == 8
