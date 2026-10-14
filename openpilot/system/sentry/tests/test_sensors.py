import math
import pytest

from openpilot.system.sentry.sensors import ActivityDetector, SENSOR_SERVICES


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
  assert detector.update(service, value, 4.01, armed=True)
  assert not detector.update(service, value, 4.01, armed=True)
  assert not detector.update(service, value, 4.02, armed=True)  # burst debounce


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


def test_moderate_noise_needs_multiple_samples_and_quiet_low_amplitude_is_ignored():
  detector = ActivityDetector()
  settle(detector)
  assert not detector.update('soundPressure', 55., 4.1, True)
  assert detector.update('soundPressure', 55., 4.2, True)
  detector = ActivityDetector()
  settle(detector, {'soundPressure': 0.})
  assert not detector.update('soundPressure', 25., 4.1, True)


def test_gradual_sound_level_changes_do_not_trigger():
  detector = ActivityDetector()
  settle(detector)
  for i in range(1, 601):
    assert not detector.update('soundPressure', 40.+i*.02, 4.+i*.1, True)


@pytest.mark.parametrize('rotate', [False, True])
@pytest.mark.parametrize('scale,expected', [(0.6, False), (0.9, True), (1., True)])
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
  assert not any(hits[:8])  # Moderate movement still needs 80 ms confirmation.
  assert any(hits) == expected


def test_isolated_small_gyro_spike_and_slow_bias_drift_do_not_trigger():
  detector = ActivityDetector()
  bias = (-.035, -.017, -.028)
  settle(detector, {'gyroscope': bias})
  assert not detector.update('gyroscope', (bias[0], bias[1]+.006, bias[2]), 4.01, True)
  assert not detector.update('gyroscope', bias, 4.02, True)
  for i in range(1, 2001):
    value = (bias[0]+i*.000001, bias[1], bias[2])
    assert not detector.update('gyroscope', value, 4.02+i*.01, True)


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
  def message(service, value, timestamp, valid=True):
    msg = messaging.new_message(service, valid=valid)
    msg.logMonoTime = int(timestamp*1e9)
    data = getattr(msg, service)
    if service == 'soundPressure':
      data.soundPressureWeightedDb = value
    else:
      # sensord's LSM6DS3 driver emits gyroUncalibrated, not gyro.
      field = 'acceleration' if service == 'accelerometer' else 'gyroUncalibrated'
      data.init(field).v = value
    queued[service].append(msg.as_reader())

  monkeypatch.setattr(messaging, 'sub_sock', subscribe)
  monkeypatch.setattr(messaging, 'drain_sock', drain)
  activity = SensorActivity()
  ambient = {'soundPressure': 40., 'accelerometer': (0., 0., 9.81), 'gyroscope': (0., 0., 0.)}
  for i in range(41):
    for source, baseline in ambient.items():
      message(source, baseline, i*.1)
    assert not activity.sample(i*.1, armed=False)
  assert activity.ready(4.)
  message(service, value, 4.01)
  message(service, ambient[service], 4.02)
  assert activity.sample(4.2, armed=True)
  assert not activity.sample(4.2, armed=True)
  message(service, value, 5.0)  # stale by the next controller tick
  message(service, value, 6.99, valid=False)
  assert not activity.sample(7., armed=True)
