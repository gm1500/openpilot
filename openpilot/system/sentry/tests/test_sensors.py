import math
import pytest

from openpilot.system.sentry.sensors import ActivityDetector, SENSOR_SERVICES, audio_threshold


def settle(detector, values=None):
  values = values or {'soundPressure': 40., 'accelerometer': (0., 0., 9.81), 'gyroscope': (0., 0., 0.)}
  for i in range(401):
    for service, value in values.items():
      assert not detector.update(service, value, i*.01, armed=False)
  return values


@pytest.mark.parametrize('service,value', [
  ('soundPressure', 70.), ('accelerometer', (2., 0., 9.81)), ('gyroscope', (0., .3, 0.)),
])
def test_each_sensor_can_trigger_and_duplicates_do_not(service, value):
  detector = ActivityDetector()
  settle(detector)
  assert detector.ready(4.)
  assert detector.update(service, value, 4.01, armed=True) == (service != 'gyroscope')
  assert not detector.update(service, value, 4.01, armed=True)
  assert detector.update(service, value, 4.02, armed=True) == (service == 'gyroscope')
  assert not detector.update(service, value, 4.03, armed=True)  # burst debounce


@pytest.mark.parametrize('axis', range(3))
def test_mount_orientation_does_not_change_acceleration_detection(axis):
  detector = ActivityDetector()
  gravity = [0., 0., 0.]
  gravity[axis] = 9.81
  settle(detector, {'accelerometer': gravity})
  bump = gravity.copy()
  bump[(axis+1) % 3] = 1.5
  assert detector.update('accelerometer', bump, 4.01, True)


def test_ambient_noise_warmup_small_vibration_and_stale_restart():
  detector = ActivityDetector()
  values = settle(detector)
  for i in range(401, 1000):
    t = i*.01
    assert not detector.update('soundPressure', 40.+2*math.sin(t), t, True)
    assert not detector.update('accelerometer', (.05*math.sin(t), 0., 9.81), t, True)
    assert not detector.update('gyroscope', (0., .001*math.sin(t), 0.), t, True)
  assert not detector.ready(11.)
  assert not detector.update('soundPressure', 80., 11., True)  # Relearn after gap.
  assert not detector.update('accelerometer', (math.nan, 0., 0.), 11., True)
  assert not detector.update('gyroscope', (math.inf, 0., 0.), 11., True)
  assert not detector.update('accelerometer', (1., 2.), 11., True)
  for service in SENSOR_SERVICES:
    assert not detector.update(service, values[service], 12., False)


def test_single_microphone_block_knock_triggers_but_quiet_low_amplitude_is_ignored():
  detector = ActivityDetector()
  settle(detector)
  assert detector.update('soundPressure', 48., 4.1, True, 25)
  assert not detector.update('soundPressure', 40., 4.2, True, 25)
  assert detector.active(4.2, 'soundPressure')
  detector = ActivityDetector()
  settle(detector, {'soundPressure': 0.})
  assert not detector.update('soundPressure', 25., 4.1, True)


def test_gradual_sound_level_changes_do_not_trigger():
  detector = ActivityDetector()
  settle(detector)
  for i in range(1, 601):
    assert not detector.update('soundPressure', 40.+i*.02, 4.+i*.1, True)


def test_sound_envelope_tracks_loudness_keeps_short_peaks_and_clears_stale_audio():
  levels = []
  for db in (40., 46., 52., 64.):
    detector = ActivityDetector()
    settle(detector)
    detector.update('soundPressure', db, 4.01, True)
    levels.append(detector.audio_level(4.01))
  assert levels == sorted(levels)
  assert levels[0] == 0. and levels[-1] == 1.
  detector.update('soundPressure', 40., 4.02, True)
  assert .4 < detector.audio_level(4.2) < 1.  # Peak survives a quiet sample in the same drain.
  assert detector.audio_level(4.53) == 0.
  assert detector.audio_level(3.) == 0.
  detector.update('soundPressure', 70., 5., True)  # Relearn after a message gap.
  assert detector.audio_level(5.) == 0.


def test_sound_envelope_waits_for_arming_and_does_not_change_trigger_threshold():
  detector = ActivityDetector()
  settle(detector)
  assert not detector.update('soundPressure', 45., 4.01, True)
  assert 0. < detector.audio_level(4.01) < .5
  assert not detector.update('soundPressure', 80., 4.02, False)
  assert detector.audio_level(4.02) == 0.


@pytest.mark.parametrize('sensitivity,expected', [(0, False), (25, True), (50, True), (75, True), (100, True)])
def test_audio_sensitivity_detects_quieter_sounds_at_existing_slider_setting(sensitivity, expected):
  assert audio_threshold(50) == 9.
  assert audio_threshold(25) == pytest.approx(12.73, abs=.01)
  detector = ActivityDetector()
  for i in range(41):
    assert not detector.update('soundPressure', 40., i*.1, False, sensitivity)
  # Gentle onset exercises the sustained-sound path at the default setting.
  hits = [detector.update('soundPressure', level, 4.+i*.1, True, sensitivity)
          for i, level in enumerate((43., 46., 49., 52., 55., 55., 55.), 1)]
  assert any(hits) == expected


def test_fluctuating_background_audio_stays_quiet_but_a_small_knock_can_trigger():
  detector = ActivityDetector()
  for i in range(101):
    assert not detector.update('soundPressure', 60.+2*math.sin(i*.5), i*.1, i > 30, 25)
  assert detector.update('soundPressure', 69., 10.1, True, 25)
  assert not detector.update('soundPressure', 60., 10.2, True, 25)
  assert not detector.active(10.5, 'soundPressure')


@pytest.mark.parametrize('ambient', [55., 70., 95.])
def test_new_constant_noise_settles_without_repeated_events_and_still_detects_bang(ambient):
  detector = ActivityDetector()
  settle(detector)
  hits = [4.+i*.01 for i in range(1, 801) if detector.update('soundPressure', ambient, 4.+i*.01, True, 25)]
  assert len(hits) <= 1
  assert not detector.active(12.) and detector.audio_level(12.) < .01
  assert detector.baseline['soundPressure'] == (ambient,)
  assert detector.update('soundPressure', ambient+40., 12.01, True, 25)
  # A single sharp sound must not immediately teach a new, louder ambient level.
  assert detector.baseline['soundPressure'] == (ambient,)


@pytest.mark.parametrize('frequency', [7., 20.])
@pytest.mark.parametrize('amplitude', [.006, .02])
def test_continuous_gyro_vibration_calibrates_without_losing_short_impacts(frequency, amplitude):
  detector = ActivityDetector()
  bias = (-.035, -.017, -.028)
  for i in range(1, 1041):
    t = i/104.
    value = (bias[0], bias[1]+amplitude*math.sin(math.tau*frequency*t), bias[2])
    assert not detector.update('gyroscope', value, t, t > 4.)
  assert detector.ambient['gyroscope'].spread > .002
  impact = (bias[0], bias[1]+6*amplitude, bias[2])
  assert not detector.update('gyroscope', impact, 10.01, True)
  assert detector.update('gyroscope', impact, 10.02, True)
  assert detector.active(10.2, 'gyroscope')
  assert not detector.active(10.4, 'gyroscope')


def test_sustained_rotation_does_not_become_bias_or_cause_false_activity_when_it_stops():
  detector = ActivityDetector()
  settle(detector)
  hits = [detector.update('gyroscope', (.03, 0., 0.), 4.+i*.01, True) for i in range(1, 501)]
  assert any(hits[:10]) and any(hits[-100:])
  assert detector.baseline['gyroscope'] == (0., 0., 0.)
  for i in range(1, 401):
    assert not detector.update('gyroscope', (0., 0., 0.), 9.+i*.01, True)
  assert not detector.active(13.)


def test_confirmed_source_stays_visible_inside_recording_debounce_then_releases():
  detector = ActivityDetector()
  settle(detector)
  assert detector.update('soundPressure', 80., 4.01, True)
  assert not detector.update('gyroscope', (.05, 0., 0.), 4.02, True)
  assert not detector.update('gyroscope', (.05, 0., 0.), 4.03, True)  # Confirmed, inside audio's debounce.
  assert detector.active(4.2, 'soundPressure') and detector.active(4.2, 'gyroscope')
  assert not detector.active(4.2, 'accelerometer')
  assert not detector.update('soundPressure', 40., 4.03, True)
  assert not detector.update('gyroscope', (0., 0., 0.), 4.04, True)
  assert not detector.active(4.4)


@pytest.mark.parametrize('rotate', [False, True])
@pytest.mark.parametrize('scale,expected', [(0.4, False), (0.6, True), (0.9, True), (1., True)])
def test_bookmarked_gyro_amplitude_when_sustained(rotate, scale, expected):
  # Bias and bookmarked sample from stationary 000003ef--91bc370306/5.
  # qlog is ~1 Hz: repeat the measured amplitude for a synthetic 125 ms pulse,
  # not a claim that the original peak lasted this long at the native 104 Hz.
  baseline = (-.035048, -.016875, -.027565)
  peak = (-.036346, -.012370, -.028100)
  if rotate:
    baseline = (-baseline[1], baseline[0], baseline[2])
    peak = (-peak[1], peak[0], peak[2])
  detector = ActivityDetector()
  settle(detector, {'gyroscope': baseline})
  value = tuple(b+scale*(p-b) for b, p in zip(baseline, peak, strict=True))
  hits = [detector.update('gyroscope', value, 4.+i/104., True) for i in range(1, 14)]
  assert not any(hits[:3])  # Moderate movement still needs 20 ms confirmation.
  assert any(hits) == expected


@pytest.mark.parametrize('sensitivity,expected', [(0, False), (25, False), (50, False), (75, True), (100, True)])
def test_gyro_slider_detects_lighter_shakes_without_changing_audio_or_acceleration(sensitivity, expected):
  detector = ActivityDetector()
  settle(detector)
  hits = [detector.update('gyroscope', (.002, 0., 0.), 4.+i/104., True, gyro_sensitivity=sensitivity) for i in range(1, 14)]
  assert any(hits) == expected
  for service, value in [('soundPressure', 48.), ('accelerometer', (.3, 0., 9.81))]:
    detector = ActivityDetector()
    settle(detector)
    assert detector.update(service, value, 4.1, True, audio_sensitivity=50, gyro_sensitivity=sensitivity) == (service == 'soundPressure')


def test_short_first_gyro_jolt_triggers_before_slower_rebound_and_then_releases():
  detector = ActivityDetector()
  bias = (-.035, -.017, -.028)
  for i in range(417):
    assert not detector.update('gyroscope', bias, i/104., False)
  hits = []
  for i in range(1, 105):
    offset = .004 if i <= 4 else -.003 if 22 <= i <= 35 else 0.
    t = 4.+i/104.
    if detector.update('gyroscope', (bias[0], bias[1]+offset, bias[2]), t, True):
      hits.append(t)
    if i == 4:
      assert detector.active(t, 'gyroscope')
  assert len(hits) == 1 and hits[0]-4. < .04
  assert not detector.active(5., 'gyroscope')
  assert detector.baseline['gyroscope'] == pytest.approx(bias, abs=1e-6)


def test_isolated_small_gyro_spike_and_slow_bias_drift_do_not_trigger():
  detector = ActivityDetector()
  bias = (-.035, -.017, -.028)
  settle(detector, {'gyroscope': bias})
  assert not detector.update('gyroscope', (bias[0], bias[1]+.004, bias[2]), 4.01, True)
  assert not detector.update('gyroscope', bias, 4.02, True)
  for i in range(1, 2001):
    value = (bias[0]+i*.000001, bias[1], bias[2])
    assert not detector.update('gyroscope', value, 4.02+i*.01, True)


@pytest.mark.parametrize('peak', [.004, .006, .03])
def test_repeated_isolated_gyro_peaks_do_not_retrigger_after_a_real_shake(peak):
  detector = ActivityDetector()
  settle(detector)
  assert not detector.update('gyroscope', (.02, 0., 0.), 4.01, True)
  assert detector.update('gyroscope', (.02, 0., 0.), 4.02, True)
  for i in range(1, 1041):
    t = 4.02+i/104.
    value = (peak if i % 104 == 0 else 0., 0., 0.)
    assert not detector.update('gyroscope', value, t, True)
    if t > 4.4:
      assert not detector.active(t, 'gyroscope')


@pytest.mark.parametrize('service,value', [
  ('soundPressure', 70.), ('accelerometer', (2., 0., 9.81)), ('gyroscope', (0., .3, 0.)),
])
def test_native_messages_and_brief_impact_between_controller_ticks(monkeypatch, service, value):
  from openpilot.cereal import messaging
  from openpilot.system.sentry.sensors import SensorActivity

  queued = {s: [] for s in SENSOR_SERVICES}
  def subscribe(service, conflate):
    assert not conflate  # A later quiet sample must not hide a short impact.
    return service
  def drain(service, wait_for_one):
    assert not wait_for_one
    events, queued[service] = queued[service], []
    return events
  def message(service, value, timestamp, valid=True, sensor_delay=0.):
    msg = messaging.new_message(service, valid=valid)
    msg.logMonoTime = int(timestamp*1e9)
    data = getattr(msg, service)
    if service == 'soundPressure':
      data.soundPressureWeightedDb = value
    else:
      # sensord's LSM6DS3 driver emits gyroUncalibrated, not gyro.
      field = 'acceleration' if service == 'accelerometer' else 'gyroUncalibrated'
      data.init(field).v = value
      data.timestamp = int((timestamp-sensor_delay)*1e9)
    queued[service].append(msg.as_reader())

  monkeypatch.setattr(messaging, 'sub_sock', subscribe)
  monkeypatch.setattr(messaging, 'drain_sock', drain)
  activity = SensorActivity()
  ambient = {'soundPressure': 40., 'accelerometer': (0., 0., 9.81), 'gyroscope': (0., 0., 0.)}
  for i in range(401):
    for source, baseline in ambient.items():
      message(source, baseline, i*.01)
    assert not activity.sample(i*.01, armed=False)
  assert activity.ready(4.)
  assert activity.gyro.angle(3.8, 4.) == pytest.approx(0.)
  message(service, value, 4.01, sensor_delay=.005)
  if service == 'gyroscope':
    message(service, value, 4.02, sensor_delay=.005)
  message(service, ambient[service], 4.03, sensor_delay=.005)
  assert activity.sample(4.2, armed=True)
  assert activity.active(4.2, service)  # Later quiet sample did not hide the impact.
  assert not activity.active(4.4, service)
  if service == 'gyroscope':
    assert activity.gyro.samples[-1][0] == pytest.approx(4.025)
    assert activity.gyro.angle(4., 4.015) > .002
    assert activity.gyro_diagnostics['gyroMagnitude'] == pytest.approx(.3)
    assert activity.gyro_diagnostics['gyroThreshold'] == pytest.approx(.0025)
    assert activity.gyro_diagnostics['gyroNoise'] == 0.
    assert activity.gyro_diagnostics['gyroBaselineOffset'] == 0.
  assert not activity.sample(4.2, armed=True)
  assert activity.gyro_diagnostics == {}
  message(service, value, 5.0)  # stale by the next controller tick
  message(service, value, 6.99, valid=False)
  assert not activity.sample(7., armed=True)
