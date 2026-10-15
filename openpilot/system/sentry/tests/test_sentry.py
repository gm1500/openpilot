import json
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.system.sentry import config
from openpilot.system.sentry.location import parked_location, gps_location
from openpilot.system.sentry.motion import MotionDetector, camera_area_scale


class FakeParams(dict):
  get_bool = dict.get


def warm(detector, image):
  for t in np.arange(.2, 3., .2):
    assert not detector.update(image, float(t)).active


@pytest.mark.parametrize('stream', [0, 1, 2])
@pytest.mark.parametrize('sensitivity', [50, 100])
def test_exposure_and_noise_do_not_trigger(stream, sensitivity):
  d = MotionDetector.for_camera(stream, 1928, 1208)
  base = np.full((80, 128), 70, dtype=np.float32)
  warm(d, base)
  rng = np.random.default_rng(2)
  for i in range(30):
    image = base+70+rng.normal(0, 2, base.shape)
    assert not d.update(image, 3.+i*.2, sensitivity).active


def test_startup_fade_is_learned_during_warmup():
  d = MotionDetector()
  rows, cols = np.indices((80, 128))
  scene = 30+(cols//16 % 2)*90+rows*.8
  for i in range(150):
    # Spatially varying exposure gain defeats additive brightness compensation.
    # Even a fade lasting almost the entire warmup must not leave stale hits.
    image = scene*min(1., .04+i*.2/29.)
    assert not d.update(image, 1.+i*.2, 75, armed=False).active
  for i in range(15):
    assert not d.update(scene, 31.+i*.2, 75).active
  for i in range(17):
    moving = scene.copy()
    moving[20:40, 70+i:95+i] += 80
    result = d.update(moving, 34.+i*.2, 75)
    assert result.active
    assert result.triggered == (i >= 3)
  assert 86/127 <= result.x < 111/127


def test_local_motion_tracks_position_and_needs_persistence():
  d = MotionDetector()
  base = np.full((80, 128), 70, dtype=np.float32)
  warm(d, base)
  for i in range(17):
    image = base.copy()
    image[20:40, 75+i:100+i] += 100
    result = d.update(image, 3.+i*.2, 75)
    assert result.active  # Eye reacts on the first sample, well before saving.
    assert result.triggered == (i >= 3)
  assert 91/127 <= result.x < 116/127 and .3 < result.y < .5
  # Loss of camera continuity re-arms instead of comparing stale pictures.
  assert not d.update(image, 8.).triggered


def test_sensitivity_detects_smaller_changes():
  base = np.full((80, 128), 70, dtype=np.float32)
  result = []
  for sensitivity in (1, 100):
    d = MotionDetector()
    warm(d, base)
    for i in range(17):
      image = base.copy()
      image[20:40, 70+i:86+i] += 20
      m = d.update(image, 3.+i*.2, sensitivity)
    result.append(m.triggered)
  assert result == [False, True]


@pytest.mark.parametrize('stream,contrast,shape', [(0, 35, (24, 40)), (1, 14, (12, 16)), (2, 35, (12, 16))])
def test_fifty_percent_detects_camera_scaled_motion(stream, contrast, shape):
  base = np.full((80, 128), 70, dtype=np.float32)
  detector = MotionDetector.for_camera(stream, 1928, 1208)
  warm(detector, base)
  results = []
  for i in range(10):
    image = base.copy()
    x = 40+i*3
    image[25:25+shape[0], x:x+shape[1]] += contrast
    results.append(detector.update(image, 3.+i*.2, 50))
  assert all(r.active for r in results)
  assert not any(r.triggered for r in results[:3])
  assert all(r.triggered for r in results[3:])


@pytest.mark.parametrize('stream', [1, 2])
def test_dark_low_contrast_approach_triggers_at_fifty_without_a_flashlight(stream):
  detector = MotionDetector.for_camera(stream, 1928, 1208)
  base = np.full((80, 128), 21, dtype=np.float32)
  warm(detector, base)
  rng = np.random.default_rng(7)
  results = []
  for i in range(12):
    image = base+rng.normal(0, .5, base.shape)
    image[28:44, 20+i*2:34+i*2] -= 4
    results.append(detector.update(image, 3.+i*.2, 50))
  assert results[0].active and not results[0].triggered
  assert any(m.triggered for m in results[3:6])
  assert all(.15 < m.x < .5 and .3 < m.y < .6 for m in results if m.active)


@pytest.mark.parametrize('stream', [1, 2])
@pytest.mark.parametrize('angle', [-12, 0, 12])
def test_bright_sky_does_not_hide_a_dim_crossing_in_the_middle_band(stream, angle):
  detector = MotionDetector.for_camera(stream, 1928, 1208)
  base = np.full((80, 128), 55, dtype=np.float32)
  base[:24] = 180  # Bright sky/window, with a much darker scene below it.
  warm(detector, base)
  rows, cols = np.indices(base.shape)
  radians = np.deg2rad(angle)
  results = []
  for i in range(12):
    dx, dy = cols-(30+i*2), rows-39
    x = dx*np.cos(radians)+dy*np.sin(radians)
    y = -dx*np.sin(radians)+dy*np.cos(radians)
    image = base.copy()
    image[(np.abs(x) < 3.5) & (np.abs(y) < 5.5)] -= 8
    results.append(detector.update(image, 3.+i*.2, 50))
  assert results[0].active
  assert not any(m.triggered for m in results[:3])
  assert any(m.triggered for m in results[3:6])


@pytest.mark.parametrize('stream', [1, 2])
def test_middle_horizontal_third_has_priority_across_the_full_width(stream):
  base = np.full((80, 128), 160, dtype=np.float32)
  results = {}
  for top in (3, 37, 72):
    for left in (10, 85):
      detector = MotionDetector.for_camera(stream, 1928, 1208)
      warm(detector, base)
      samples = []
      for i in range(8):
        image = base.copy()
        image[top:top+5, left+i*2:left+i*2+7] += 60
        samples.append(detector.update(image, 3.+i*.2, 50))
      assert all(m.active for m in samples)  # Outer motion still moves the eye.
      assert samples[-1].triggered == (top == 37)
      results[top, left] = [m.score for m in samples]
    assert results[top, 10] == results[top, 85]  # No left/right preference.
  assert results[37, 10][-1] > 1.5*max(results[3, 10][-1], results[72, 10][-1])


@pytest.mark.parametrize('stream', [1, 2])
@pytest.mark.parametrize('sensitivity', [50, 100])
@pytest.mark.parametrize('noise', [.5, 2., 5.])
def test_dark_noise_and_uniform_exposure_changes_do_not_save(stream, sensitivity, noise):
  detector = MotionDetector.for_camera(stream, 1928, 1208)
  rows, cols = np.indices((80, 128))
  base = 20.+rows/16+cols/32
  rng = np.random.default_rng(5)
  for i in range(180):
    image = base+(3 if i >= 80 else 0)+rng.normal(0, noise, base.shape)
    assert not detector.update(image, 1.+i*.2, sensitivity).triggered


@pytest.mark.parametrize('stream', [1, 2])
def test_dark_scattered_specks_cannot_pool_into_an_event(stream):
  detector = MotionDetector.for_camera(stream, 1928, 1208)
  base = np.full((80, 128), 21, dtype=np.float32)
  warm(detector, base)
  for i in range(30):
    image = base.copy()
    for y in range(8, 70, 14):
      for x in range(8, 110, 16):
        shift = i % 3
        image[y+shift:y+shift+3, x+shift:x+shift+3] += 30
    assert not detector.update(image, 3.+i*.2, 100).triggered


@pytest.mark.parametrize('stream', [1, 2])
@pytest.mark.parametrize('brightness,contrast', [(70, 70), (21, -4)])
def test_large_motion_remains_detectable_across_positions_and_orientations(stream, brightness, contrast):
  for top, left, orientation in ((5, 8, 0), (30, 50, 0), (62, 96, 0), (30, 50, 1), (30, 50, 2)):
    detector = MotionDetector.for_camera(stream, 1928, 1208)
    base = np.full((80, 128), brightness, dtype=np.float32)
    warm(detector, np.rot90(base, orientation))
    results = []
    for i in range(6):
      image = base.copy()
      image[top:top+12, left+i*2:left+15+i*2] += contrast
      results.append(detector.update(np.rot90(image, orientation), 3.+i*.2, 50))
    assert all(m.active for m in results) and results[-1].triggered
    if orientation == 0:
      assert top/79 <= results[-1].y <= (top+12)/79
      assert results[-1].x > results[0].x  # Eye follows the subject in every region.


@pytest.mark.parametrize('stream', [1, 2])
@pytest.mark.parametrize('angle', [-12, 0, 12])
def test_dim_approach_tolerates_mount_tilt_and_scene_position(stream, angle):
  rows, cols = np.indices((80, 128))
  radians = np.deg2rad(angle)
  for center_y in (16, 40, 62):
    detector = MotionDetector.for_camera(stream, 1928, 1208)
    base = np.full((80, 128), 21, dtype=np.float32)
    warm(detector, base)
    results = []
    for i in range(8):
      dx, dy = cols-(30+i*2), rows-center_y
      x = dx*np.cos(radians)+dy*np.sin(radians)
      y = -dx*np.sin(radians)+dy*np.cos(radians)
      image = base.copy()
      image[(np.abs(x) < 8) & (np.abs(y) < 7)] -= 4
      results.append(detector.update(image, 3.+i*.2, 50))
    assert results[0].active and not results[0].triggered
    assert any(m.triggered for m in results[3:6])


@pytest.mark.parametrize('sensitivity', [1, 100])
def test_cabin_calibration_preserves_slider_endpoints(sensitivity):
  original = MotionDetector(area_scale=camera_area_scale(1, 1928, 1208))
  calibrated = MotionDetector.for_camera(1, 1928, 1208)
  base = np.full((80, 128), 70, dtype=np.float32)
  for detector in (original, calibrated):
    warm(detector, base)
  # High-contrast motion remains identical at both ends; sensitivity 50 gets
  # a lower cabin threshold, without raising maximum noise sensitivity.
  for i in range(20):
    image = base.copy()
    image[25:40, 40+i:60+i] += 60
    assert original.update(image, 3.+i*.2, sensitivity) == calibrated.update(image, 3.+i*.2, sensitivity)


def test_eye_follows_fresh_motion_instead_of_large_background_footprint():
  detector = MotionDetector()
  base = np.full((80, 128), 70, dtype=np.float32)
  warm(detector, base)
  scene = base.copy()
  scene[10:45, 10:40] += 100
  assert detector.update(scene, 3., 50).x < .4
  scene[15:19, 15:19] += 30  # A small fluctuation inside the large old footprint.
  scene[30:40, 90:100] += 80  # New movement across the frame.
  result = detector.update(scene, 3.2, 50)
  assert result.active and result.x > .7
  assert not result.triggered  # Eye movement is independent of the save timer.
  # These values go directly into SentryState. NumPy scalar coordinates are
  # rejected by pycapnp and would stop the controller on its first motion hit.
  from openpilot.cereal import log
  event = log.Event.new_message()
  state = event.init('sentryState')
  state.motionX, state.motionY, state.motionScore = result.x, result.y, result.score
  assert event.to_bytes()


@pytest.mark.parametrize('after', ['gone', 'stationary'])
def test_short_motion_and_background_afterimage_do_not_save(after):
  base = np.full((80, 128), 70, dtype=np.float32)
  d = MotionDetector()
  warm(d, base)
  for i in range(2):
    image = base.copy()
    image[20:40, 60+i:85+i] += 100
    result = d.update(image, 3.+i*.2, 75)
    assert result.active and not result.triggered
  resting = base if after == 'gone' else image
  for i in range(40):
    result = d.update(resting, 3.4+i*.2, 75)
    assert not result.triggered
    if i:
      assert not result.active


@pytest.mark.parametrize('stream', [0, 1, 2])
@pytest.mark.parametrize('contrast', [-40, 80])
def test_small_motion_moves_eye_without_saving(stream, contrast):
  base = np.full((80, 128), 70, dtype=np.float32)
  d = MotionDetector.for_camera(stream, 1928, 1208)
  warm(d, base)
  for i in range(40):
    image = base.copy()
    x = 5+(4*i) % 110
    image[30:35, x:x+5] += contrast
    result = d.update(image, 3.+i*.2, 100)
    assert result.active
    assert not result.triggered


@pytest.mark.parametrize('stream', [1, 2])
@pytest.mark.parametrize('noise', [.5, 2.])
def test_dark_sensor_noise_cannot_keep_a_departed_subject_recording(stream, noise):
  detector = MotionDetector.for_camera(stream, 1928, 1208)
  base = np.full((80, 128), 21, dtype=np.float32)
  warm(detector, base)
  occupied = base.copy()
  occupied[20:45, 50:75] += 40
  for i in range(15):
    detector.update(occupied, 3.+i*.2, 50)
  # The departed subject leaves a broad background residual. Scattered fresh
  # pixel changes inside it must not be mistaken for the subject moving again.
  detector.update(base, 6., 50)
  rng = np.random.default_rng(19)
  for i in range(60):
    result = detector.update(base+rng.normal(0, noise, base.shape), 6.2+i*.2, 50)
    assert not result.triggered


@pytest.mark.parametrize('stream', [0, 1, 2])
@pytest.mark.parametrize('sensitivity', [50, 100])
def test_scattered_snow_cannot_combine_area_into_a_trigger(stream, sensitivity):
  base = np.full((80, 128), 70, dtype=np.float32)
  d = MotionDetector.for_camera(stream, 1928, 1208)
  warm(d, base)
  for i in range(60):
    image = base.copy()
    for y in range(8, 70, 14):
      for x in range(8, 110, 16):
        shift = i % 3
        image[y+shift:y+shift+5, x+shift:x+shift+5] += 100
    assert np.count_nonzero(image-base) > .06*image.size
    result = d.update(image, 3.+i*.2, sensitivity)
    assert result.active  # The visual cue is deliberately more sensitive.
    assert not result.triggered


def test_unrelated_regions_cannot_share_the_confirmation_timer():
  base = np.full((80, 128), 70, dtype=np.float32)
  d = MotionDetector()
  warm(d, base)
  for i in range(4):
    image = base.copy()
    x = 8+i if i < 2 else 80+i-2
    image[20:40, x:x+25] += 100
    assert not d.update(image, 3.+i*.2, 75).triggered


@pytest.mark.parametrize('size', [(1928, 1208), (1344, 760)])
def test_wide_lenses_save_smaller_moving_regions_than_narrow(size):
  base = np.full((80, 128), 70, dtype=np.float32)
  outcomes = []
  for stream in (0, 1, 2):
    d = MotionDetector(area_scale=camera_area_scale(stream, *size))
    warm(d, base)
    for i in range(9):
      image = base.copy()
      image[25:35, 65+i:77+i] += 100
      result = d.update(image, 3.+i*.2, 75)
      assert result.active  # Lens area compensation changes saving, not the eye.
      if i < 3:
        assert not result.triggered
    outcomes.append(result.triggered)
  assert outcomes == [False, True, True]

  # The detector's narrow-lens calibration still requires a larger subject.
  d = MotionDetector(area_scale=camera_area_scale(0, *size))
  warm(d, base)
  for i in range(9):
    image = base.copy()
    image[25:49, 55+i:91+i] += 100
    result = d.update(image, 3.+i*.2, 75)
    assert result.triggered == (i >= 3)


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
  assert config.setting(FakeParams(), 'SentryEnabled')
  assert config.setting(FakeParams(), 'SentrySkipOnWifi')
  assert not config.setting(FakeParams(SentrySkipOnWifi=False), 'SentrySkipOnWifi')
  assert config.setting(FakeParams(), 'SentrySensitivity') == 75
  assert config.setting(FakeParams(), 'SentryAudioSensitivity') == 25
  assert config.setting(FakeParams(SentryAudioSensitivity=-10), 'SentryAudioSensitivity') == 0
  assert config.setting(FakeParams(SentryAudioSensitivity=150), 'SentryAudioSensitivity') == 100
  assert config.setting(FakeParams(), 'SentryGyroSensitivity') == 50
  assert config.setting(FakeParams(SentryGyroSensitivity=-10), 'SentryGyroSensitivity') == 0
  assert config.setting(FakeParams(SentryGyroSensitivity=150), 'SentryGyroSensitivity') == 100
  assert config.setting(FakeParams(), 'SentryArmDelay') == 30
  assert config.setting(FakeParams(SentryEnabled=False), 'SentryEnabled') is False
  assert config.setting(FakeParams(SentrySensitivity=42), 'SentrySensitivity') == 42
  for key in config.DURATION_KEYS:
    for previous, expected in ((10, 15), (20, 15), (30, 30), (40, 30), (60, 60), (300, 60)):
      assert config.setting(FakeParams({key: previous}), key) == expected
    assert config.setting(FakeParams({key: float('nan')}), key) == config.DEFAULTS[key]
  # Every former 1–5 minute choice maps to the longest available preset.
  # The obsolete key must never be interpreted as a number of seconds.
  for minutes in range(1, 6):
    assert config.setting(FakeParams(SentryPreMinutes=minutes), 'SentryPreSeconds') == 60
  assert config.setting(FakeParams(SentryPreMinutes=5, SentryPreSeconds=15), 'SentryPreSeconds') == 15
  assert config.setting(FakeParams(SentrySensitivity=400), 'SentrySensitivity') == 100
  assert config.setting(FakeParams(SentryCutoffVoltage=float('nan')), 'SentryCutoffVoltage') == 11.8
  assert config.setting(FakeParams(SentryCutoffVoltage=12.3), 'SentryCutoffVoltage') == 11.8
  assert config.setting(FakeParams(SentryCutoffVoltage=10.), 'SentryCutoffVoltage') == 11.0
  from openpilot.system.hardware.power_monitoring import VBATT_PAUSE_CHARGING
  assert config.DEFAULTS['SentryCutoffVoltage'] == VBATT_PAUSE_CHARGING


def test_manager_first_use_defaults_match_sentry(tmp_path):
  from openpilot.common.params import Params
  params = Params(str(tmp_path/'params'))
  # manager_init only fills absent keys. Its native defaults must agree with the
  # Python fallback, and an explicitly disabled session must stay disabled.
  params.put_bool('SentryEnabled', False, block=True)
  for key, value in config.DEFAULTS.items():
    assert params.get_default_value(key) == value
    if params.get(key) is None:
      params.put(key, params.get_default_value(key), block=True)
  assert not params.get_bool('SentryEnabled')
  assert params.get('SentrySensitivity') == 75
  assert params.get('SentryArmDelay') == 30
  assert params.get('SentryPreSeconds') == 60
  assert params.get('SentryPostSeconds') == 60


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


@pytest.mark.parametrize('phase,key', [('Cooling down', 'SentryShutdownOnOverheat'), ('Parking budget reached', 'SentryShutdownOnTimeLimit')])
@pytest.mark.parametrize('shutdown', [False, True])
@pytest.mark.parametrize('disable_stock', [False, True])
def test_optional_limit_shutdown_and_stock_fallback(monkeypatch, phase, key, shutdown, disable_stock):
  from openpilot.system.hardware import power_monitoring as power
  params = FakeParams(config.DEFAULTS | {key: shutdown, 'SentryCutoffVoltage': 11., 'DisablePowerDown': disable_stock, 'ForcePowerDown': False})
  monitor = SimpleNamespace(params=params, car_voltage_mV=12500, car_battery_capacity_uWh=20e6)
  runtime = {'session': True, 'phase': phase, 'parkingStartedAt': 900.}
  monkeypatch.setattr(power.time, 'monotonic', lambda: 2000.)
  monkeypatch.setattr(power, 'read_runtime', lambda now: runtime)

  def should_shutdown(ignition=False):
    return power.PowerMonitoring.should_shutdown(monitor, ignition, True, 500., True)

  assert should_shutdown() == shutdown
  assert not should_shutdown(True)
  # Returning to ordinary offroad mode must respect stock protections, but must
  # not reinterpret an unchecked switch as a mandatory shutdown.
  monitor.car_voltage_mV = 11500  # Above sentry cutoff, below stock cutoff.
  assert should_shutdown() == (shutdown or not disable_stock)
  monitor.car_voltage_mV = 10900
  assert should_shutdown()  # Battery cutoff remains mandatory in either mode.


def test_parked_time_overrules_thermal_resume_and_uses_controller_clock(monkeypatch):
  from openpilot.system.hardware import power_monitoring as power
  params = FakeParams(config.DEFAULTS | {'SentryMaxHours': 1, 'SentryShutdownOnTimeLimit': False,
                                       'SentryShutdownOnOverheat': True, 'DisablePowerDown': True, 'ForcePowerDown': False})
  monitor = SimpleNamespace(params=params, car_voltage_mV=12500, car_battery_capacity_uWh=20e6)
  runtime = {'session': True, 'phase': 'Cooling down', 'parkingStartedAt': 1000.}
  clock = [4599.]
  monkeypatch.setattr(power.time, 'monotonic', lambda: clock[0])
  monkeypatch.setattr(power, 'read_runtime', lambda now: runtime)

  def should_shutdown():
    return power.PowerMonitoring.should_shutdown(monitor, False, True, 0., True)

  assert should_shutdown()  # Still within Sentry's hour, so thermal shutdown applies.
  runtime['phase'] = 'Watching'
  assert not should_shutdown()  # Earlier stock offroad timestamp cannot expire it early.
  runtime['phase'] = 'Cooling down'
  clock[0] = 4600.
  assert not should_shutdown()  # Expired time budget takes priority; return to stock policy.
  params['SentryShutdownOnTimeLimit'] = True
  assert should_shutdown()
  params['SentryMaxHours'] = 4
  runtime['parkingLimitReached'] = True
  assert should_shutdown()  # A latched expiry stays expired even if the slider increases.
  runtime['parkingLimitReached'] = False
  runtime['phase'] = 'Waiting for temperature reading'
  assert not should_shutdown()  # Missing/NaN readings are not proof of overheating.


@pytest.mark.parametrize('sensor_event', [False, True])
def test_eye_touch_suppression_lasts_until_next_timeout(monkeypatch, sensor_event):
  # SCALE avoids monitor probing when importing the UI in a headless test process.
  monkeypatch.setenv('SCALE', '1')
  from openpilot.selfdrive.ui import ui_state as ui
  clock = [100.]
  state = SimpleNamespace(eyeEnabled=True, motion=not sensor_event, motionCamera=2,
                          sensorActive=sensor_event, recordUntilMonoTime=int(190e9) if sensor_event else 0)
  monkeypatch.setattr(ui, 'ui_state', SimpleNamespace(ignition=False, started=False, sentry_active=True, sm={'sentryState': state}))
  app = SimpleNamespace(target_fps=20, mouse_events=[], big_ui=lambda: True)
  monkeypatch.setattr(ui, 'gui_app', app)
  monkeypatch.setattr(ui.time, 'monotonic', lambda: clock[0])
  device = ui.Device()
  monkeypatch.setattr(device, '_set_awake', lambda _: None)
  device._interaction_time = 90.
  device._update_wakefulness()
  assert device.sentry_eye_visible
  device._update_brightness()
  assert device._brightness_target == 100
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
  state.sensorActive = False
  clock[0] = 131.2
  device._update_wakefulness()
  assert not device.sentry_eye_visible
  device._update_brightness()
  assert device._brightness_target == 0


def test_sensor_eye_fades_before_recording_finishes_and_respects_disable_and_session_end(monkeypatch):
  monkeypatch.setenv('SCALE', '1')
  from openpilot.selfdrive.ui import ui_state as ui
  clock = [100.]
  state = SimpleNamespace(eyeEnabled=True, motion=False, motionCamera=2, sensorActive=True, recordUntilMonoTime=int(175e9))
  context = SimpleNamespace(ignition=False, started=False, sentry_active=True, sm={'sentryState': state})
  monkeypatch.setattr(ui, 'ui_state', context)
  monkeypatch.setattr(ui, 'gui_app', SimpleNamespace(target_fps=20, mouse_events=[], big_ui=lambda: True))
  monkeypatch.setattr(ui.time, 'monotonic', lambda: clock[0])
  device = ui.Device()
  monkeypatch.setattr(device, '_set_awake', lambda _: None)
  device._interaction_time = 90.
  for now in (100., 110., 114.9):
    clock[0] = now
    device._update_wakefulness()
    device._update_brightness()
    assert device.sentry_eye_visible and device._brightness_target == 100
  state.sensorActive = False
  clock[0] = 115.3
  device._update_wakefulness()
  assert device.sentry_eye_visible  # Short fade, independent of the saved tail.
  clock[0] = 115.7
  device._update_wakefulness()
  device._update_brightness()
  assert not device.sentry_eye_visible and device._brightness_target == 0
  state.sensorActive = True
  device._update_wakefulness()
  assert device.sentry_eye_visible
  state.eyeEnabled = False
  device._update_wakefulness()
  device._update_brightness()
  assert not device.sentry_eye_visible and device._brightness_target == 0
  state.eyeEnabled = True
  context.sentry_active = False
  device._update_wakefulness()
  assert not device.sentry_eye_visible and device.sentry_motion_until == 0.


@pytest.mark.parametrize('stop_control', ['button', 'toggle'])
@pytest.mark.parametrize('ignition,started', [(False, False), (True, False), (False, True), (True, True)])
def test_settings_remain_editable_in_all_vehicle_states(monkeypatch, tmp_path, stop_control, ignition, started):
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
  assert [row.title for row in layout.scroller._items] == [
    'Sentry', 'Enable Sentry', 'Skip Sentry on Wi-Fi', 'Arming delay', 'Pre-event buffer', 'Record after motion stops',
    'Motion sensitivity', 'Audio sensitivity', 'Gyro sensitivity', 'Battery cutoff', 'Maximum parked time', 'Pause capture above',
    'Red eye', 'Video watermark', 'Buffered footage', 'Current time',
  ]
  assert layout.voltage_status() == 'now 12.0 V'
  sm.valid['peripheralState'] = False
  assert layout.voltage_status() == 'now unavailable'
  assert layout._stop_button.action_item.enabled
  assert layout._enable_toggle.action_item.enabled
  assert layout._enable_toggle.action_item.get_state()
  assert state.sentry_active == (not ignition and not started)
  for row in layout.scroller._items:
    if isinstance(row, ui.SentrySlider):
      assert row.enabled
    else:
      assert row.action_item.enabled

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

  assert layout._enable_toggle.action_item.enabled
  layout._enable_toggle.action_item.toggle._handle_mouse_release(rl.Vector2())
  assert params.get_bool('SentryEnabled') and layout._enable_toggle.action_item.get_state()
  assert state.sentry_active == (not ignition and not started)
  state.ignition = state.started = False
  assert state.sentry_active
  # Reflect an external change, e.g. a stop issued over SSH, on the next UI update.
  params.put_bool('SentryEnabled', False, block=True)
  layout._update_state()
  assert not layout._enable_toggle.action_item.get_state()
  modes = layout._enable_toggle.action_item.modes
  modes.select_button(2)
  assert params.get_bool('SentryOffThisDrive')
  layout._set_sentry_enabled(True)
  assert params.get_bool('SentryOffThisDrive')  # Master enable does not cancel this parking's skip.
  assert not state.sentry_active
  modes.select_button(1)
  assert params.get('SentryRiskMode') == 1 and not params.get_bool('SentryOffThisDrive')
  assert state.sentry_active and params.get('SentryParkingModeSource') == 'manual'
  assert [modes.is_button_selected(i) for i in range(3)] == [False, True, False]
  layout._enable_toggle.action_item.set_touch_valid_callback(lambda: False)
  assert not modes._touch_valid() and not layout._enable_toggle.action_item.toggle._touch_valid()


def test_enable_switch_and_three_coloured_parking_modes_fit_one_row(monkeypatch, tmp_path):
  import pyray as rl
  from openpilot.common.params import Params
  from openpilot.selfdrive.ui.layouts.settings import sentry as ui
  from openpilot.system.ui.widgets import list_view
  params = Params(str(tmp_path/'params'))
  params.put_bool('SentryEnabled', True, block=True)
  monkeypatch.setattr(ui.gui_app, 'font', lambda *args: rl.Font())
  monkeypatch.setattr(list_view, 'measure_text_cached', lambda font, text, size: rl.Vector2(len(text)*30, 60))
  monkeypatch.setattr(ui.Toggle, '_render', lambda *args: None)
  labels, fills = [], []
  monkeypatch.setattr(ui, 'gui_label', lambda rect, text, size, *args, **kwargs: labels.append((rect, text)))
  monkeypatch.setattr(rl, 'draw_rectangle_rounded', lambda rect, rounding, segments, color: fills.append((color.r, color.g, color.b)))
  action = ui.SentryEnableAction(params, lambda enabled: params.put_bool('SentryEnabled', enabled, block=True))
  row = ui.ListItem('Enable Sentry', action_item=action)
  # 2160px display, 500px sidebar and 50px panel margins on both sides.
  rect = row.get_right_item_rect(rl.Rectangle(550, 25, 1560, 170))
  assert rect.x >= 550+20+len('Enable Sentry')*30
  action._render(rect)
  toggle = action.toggle.rect
  assert len(labels) == 3 and toggle.x+toggle.width < labels[0][0].x
  assert labels[-1][0].x+labels[-1][0].width <= 2110
  assert all(box.y+box.height/2 == toggle.y+toggle.height/2 for box, _ in labels)
  for i, color in enumerate(((245, 196, 46), (190, 35, 42), (71, 184, 104))):
    box, _ = labels[i]
    action.modes._handle_mouse_release(rl.Vector2(box.x+box.width/2, box.y+box.height/2))
    fills.clear()
    action._render(rect)
    assert fills[i] == color
    assert [action.modes.is_button_selected(n) for n in range(3)] == [n == i for n in range(3)]
    assert params.get_bool('SentryEnabled')
  assert params.get_bool('SentryOffThisDrive')


@pytest.mark.parametrize('key,shutdown_key', [('SentryMaxHours', 'SentryShutdownOnTimeLimit'), ('SentryMaxTemp', 'SentryShutdownOnOverheat')])
def test_shutdown_switch_is_independent_of_limit_slider_and_respects_scrolling(monkeypatch, key, shutdown_key):
  monkeypatch.setenv('SCALE', '1')
  import pyray as rl
  from openpilot.selfdrive.ui.layouts.settings import sentry as ui
  monkeypatch.setattr(ui.gui_app, 'font', lambda *args: rl.Font())
  monkeypatch.setattr(ui.Toggle, '_render', lambda *args: None)
  for name in ('draw_text_ex', 'draw_rectangle_rounded', 'draw_circle'):
    monkeypatch.setattr(rl, name, lambda *args: None)

  class Params(dict):
    def put_bool(self, key, value, block=False):
      self[key] = value
    put = put_bool

  params = Params(config.DEFAULTS)
  slider = ui.SentrySlider(params, key, 'Limit', shutdown_key=shutdown_key)
  slider._render(rl.Rectangle(0, 0, 1400, 190))
  toggle = slider.shutdown_toggle
  assert 0 <= toggle.rect.x < toggle.rect.x+toggle.rect.width <= 1400
  pos = rl.Vector2(toggle.rect.x+40, toggle.rect.y+toggle.rect.height-5)
  slider._handle_mouse_press(pos)
  assert not slider.dragging  # Tapping the switch must not change the duration/temperature.
  toggle._handle_mouse_release(pos)
  assert params[shutdown_key] == (not config.DEFAULTS[shutdown_key])
  assert params[key] == config.DEFAULTS[key]
  pos = rl.Vector2(slider.track.x, slider.track.y)
  slider._handle_mouse_press(pos)
  slider._handle_mouse_event(SimpleNamespace(pos=pos, left_released=True))
  assert params[key] == config.RANGES[key][0]
  assert params[shutdown_key] == (not config.DEFAULTS[shutdown_key])
  slider.set_touch_valid_callback(lambda: False)
  assert not toggle._touch_valid()


@pytest.mark.parametrize('key', config.DURATION_KEYS)
def test_duration_buttons_select_one_preset(monkeypatch, tmp_path, key):
  monkeypatch.setenv('SCALE', '1')
  import pyray as rl
  from openpilot.common.params import Params
  from openpilot.selfdrive.ui.layouts.settings import sentry as ui
  monkeypatch.setattr(ui.gui_app, 'font', lambda *args: None)
  state = SimpleNamespace(ignition=False)
  monkeypatch.setattr(ui, 'ui_state', state)
  params = Params(str(tmp_path/'params'))
  buttons = ui.SentryDurationButtons(params, key)
  buttons.set_rect(rl.Rectangle(0, 0, 670, 170))
  assert [buttons.is_button_selected(i) for i in range(3)] == [n == config.DEFAULTS[key] for n in config.DURATION_PRESETS]
  for index, x in enumerate((105, 335, 565)):
    buttons._handle_mouse_release(rl.Vector2(x, 85))
    assert params.get(key) == config.DURATION_PRESETS[index]
    assert [buttons.is_button_selected(i) for i in range(3)] == [i == index for i in range(3)]
  state.ignition = True
  assert buttons.enabled
  buttons._handle_mouse_release(rl.Vector2(105, 85))
  assert params.get(key) == 15


def test_process_gates_do_not_start_driving_controls(monkeypatch):
  from openpilot.system.manager import process_config as processes
  monkeypatch.setattr(config, 'read_runtime', lambda: {'capture': True, 'gps': True})
  params = FakeParams(SentryEnabled=True)
  assert config.sentry_capture_requested(False, params, None)
  assert not config.sentry_capture_requested(True, params, None)
  assert config.sentry_gps_requested(False, params, None)
  for name in ('micd', 'sensord'):
    assert processes.managed_processes[name].should_run(False, params, SimpleNamespace(notCar=False))
  params['SentryOffThisDrive'] = True
  assert not config.sentry_capture_requested(False, params, None)
  assert not config.sentry_gps_requested(False, params, None)
  for name in ('micd', 'sensord'):
    assert not processes.managed_processes[name].should_run(False, params, SimpleNamespace(notCar=False))
  params['SentryOffThisDrive'] = False
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


def test_deleter_uses_normal_cleanup_for_events_but_never_active_segments(tmp_path, monkeypatch):
  import os
  from openpilot.system.loggerd import deleter
  monkeypatch.setattr(deleter.Paths, 'log_root', lambda: str(tmp_path))
  monkeypatch.setattr(deleter, 'get_available_bytes', lambda **kwargs: 0)
  monkeypatch.setattr(deleter, 'get_available_percent', lambda **kwargs: 0)
  for i in range(8):
    route = tmp_path/f'00000000--0123456789--{i}'
    route.mkdir()
    (route/'.sentry').touch()
    os.setxattr(route, 'user.preserve', b'1')  # Retention from an older Sentry build.
    if i == 0:
      (route/'sentry.lock').touch()
  ordinary = tmp_path/'00000001--0123456789--0'
  ordinary.mkdir()
  for i in range(1, 8):
    assert deleter.deleter_step() == (True, str(tmp_path/f'00000000--0123456789--{i}'))
  assert deleter.deleter_step() == (True, str(ordinary))
  assert deleter.deleter_step() == (True, None)
  assert len(list(tmp_path.iterdir())) == 1


@pytest.mark.parametrize('free_gib,free_percent', [(10, 11), (5.5, 20)])
def test_sentry_cleanup_makes_headroom_by_deleting_segments_inside_long_active_route(tmp_path, monkeypatch, free_gib, free_percent):
  from openpilot.system.loggerd import deleter
  monkeypatch.setattr(deleter.Paths, 'log_root', lambda: str(tmp_path))
  monkeypatch.setattr(deleter, 'get_available_bytes', lambda **kwargs: free_gib*1024**3)
  monkeypatch.setattr(deleter, 'get_available_percent', lambda **kwargs: free_percent)
  segments = []
  for n in (0, 598, 599, 600):
    segment = tmp_path/f'00000000--0123456789--{n}'
    segment.mkdir()
    (segment/'.sentry').touch()
    (segment/'dcamera.hevc').write_bytes(b'footage')
    if n >= 599:
      (segment/'sentry.lock').touch()
    segments.append(segment)
  # The driving policy keeps its existing limits. Sentry makes room before its
  # hard stop, even when the oldest and newest footage share one 10-hour route.
  assert deleter.deleter_step() == (False, None)
  assert deleter.deleter_step(sentry_capture=True) == (True, str(segments[0]))
  assert deleter.deleter_step(sentry_capture=True) == (True, str(segments[1]))
  assert deleter.deleter_step(sentry_capture=True) == (True, None)
  assert all((segment/'dcamera.hevc').exists() for segment in segments[2:])


def test_cleanup_polls_faster_only_during_sentry_capture(monkeypatch):
  from openpilot.system.loggerd import deleter
  waits, modes = [], []
  runtime = iter([{'capture': True}, {'capture': True}, {}])
  monkeypatch.setattr(deleter, 'read_runtime', lambda: next(runtime))
  def step(capture):
    modes.append(capture)
    return len(modes) == 2, None
  monkeypatch.setattr(deleter, 'deleter_step', step)
  event = SimpleNamespace(is_set=lambda: len(waits) == 3, wait=waits.append)
  deleter.deleter_thread(event)
  assert modes == [True, True, False]
  assert waits == [1, .1, 30]
