"""Parked sound/IMU triggers, independent of mounting orientation and camera motion."""
import math

SENSOR_SERVICES = ('soundPressure', 'accelerometer', 'gyroscope')
SETTLE_SECONDS = 3.
FRESH_SECONDS = .5


class ActivityDetector:
  def __init__(self):
    self.baseline = {}
    self.first = {}
    self.last = {}
    self.above_since = {}
    self.last_trigger = -math.inf

  def update(self, service, value, timestamp, armed):
    values = (float(value),) if service == 'soundPressure' else tuple(value)
    if len(values) != (1 if service == 'soundPressure' else 3) or not all(math.isfinite(v) for v in values):
      return False
    previous = self.last.get(service)
    if previous is not None and timestamp <= previous:
      return False
    self.last[service] = timestamp
    if previous is None or timestamp-previous > FRESH_SECONDS:
      self.first[service] = timestamp
      self.baseline[service] = values
      self.above_since.pop(service, None)
      return False

    dt = timestamp-previous
    baseline = self.baseline[service]
    if service == 'soundPressure':
      # micd's dB readings are uncalibrated. Use change above the ambient
      # floor, with a modest floor to reject numerical noise in near silence.
      strength = (values[0]-baseline[0])/12. if values[0] >= 30. else 0.
      confirm = .08
    else:
      # Subtract gravity/bias as a vector; no fixed mount axes or car model.
      threshold = .6 if service == 'accelerometer' else .08  # m/s², rad/s
      strength = math.dist(values, baseline)/threshold
      confirm = .04 if service == 'accelerometer' else .08

    # Learn quickly while settling, slowly while watching. Loud sustained
    # background noise eventually becomes ambient rather than an endless event.
    settling = timestamp-self.first[service] < SETTLE_SECONDS
    alpha = 1-math.exp(-dt/(.5 if settling else 10.))
    self.baseline[service] = tuple(b+alpha*(v-b) for b, v in zip(baseline, values, strict=True))
    if not armed or settling or strength < 1.:
      self.above_since.pop(service, None)
      return False
    since = self.above_since.setdefault(service, timestamp)
    if (strength >= 2. or timestamp-since >= confirm) and timestamp-self.last_trigger >= 2.:
      self.last_trigger = timestamp
      return True
    return False

  def ready(self, now):
    return all(service in self.first and now-self.first[service] >= SETTLE_SECONDS and
               0 <= now-self.last[service] <= FRESH_SECONDS for service in SENSOR_SERVICES)


class SensorActivity:
  def __init__(self):
    from openpilot.cereal import messaging
    self.messaging = messaging
    # Drain every sample so short impacts are not lost between sentryd's 5 Hz
    # ticks. Nothing is written to disk and raw microphone audio is not consumed.
    self.sockets = {s: messaging.sub_sock(s, conflate=False) for s in SENSOR_SERVICES}
    self.detector = ActivityDetector()

  def sample(self, now, armed):
    triggered = False
    for service, sock in self.sockets.items():
      for event in self.messaging.drain_sock(sock, wait_for_one=False):
        timestamp = event.logMonoTime/1e9
        if not event.valid or not 0 <= now-timestamp <= FRESH_SECONDS:
          continue
        data = getattr(event, service)
        if service == 'soundPressure':
          value = data.soundPressureWeightedDb
        else:
          field = data.which()
          expected = ('acceleration',) if service == 'accelerometer' else ('gyro', 'gyroUncalibrated')
          if field not in expected:
            continue
          value = getattr(data, field).v
        triggered |= self.detector.update(service, value, timestamp, armed)
    return triggered

  def ready(self, now):
    return self.detector.ready(now)
