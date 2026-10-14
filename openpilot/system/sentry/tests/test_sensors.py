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
    assert not detector.update('gyroscope', (0., .005*math.sin(t), 0.), t, True)
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
