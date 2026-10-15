"""Parked sound/IMU triggers, independent of mounting orientation and camera motion."""
import math
from collections import deque
from statistics import median

from openpilot.system.sentry.stabilization import GyroHistory

SENSOR_SERVICES = ('soundPressure', 'accelerometer', 'gyroscope')
SETTLE_SECONDS = 3.
FRESH_SECONDS = .5
GYRO_THRESHOLD_RAD_S = .0025
ACTIVITY_HOLD_SECONDS = .35
AMBIENT_SECONDS = 2.


def audio_threshold(sensitivity):
  """Rise above ambient: 0/50/100 percent corresponds to 18/9/4.5 dB."""
  sensitivity = min(100., max(0., float(sensitivity)))
  return 9.*2.**((50.-sensitivity)/50.)


class AmbientCalibration:
  """Robust recent ambient level and vibration spread; brief spikes are outliers."""
  def __init__(self):
    self.samples = deque(maxlen=512)
    self.center = ()
    self.spread = 0.
    self.updated = -math.inf

  def update(self, values, timestamp):
    # Keep native samples to avoid aliasing periodic vibration. Recompute the
    # small window at 5 Hz; detection still sees every sample immediately.
    self.samples.append((timestamp, values))
    while timestamp-self.samples[0][0] > AMBIENT_SECONDS:
      self.samples.popleft()
    if timestamp-self.updated < .2:
      return
    self.updated = timestamp
    axes = [sorted(axis) for axis in zip(*(v for _, v in self.samples), strict=True)]
    if len(values) == 1:
      # Lower quartile keeps a brief bang out of the noise floor; a newly steady
      # background becomes ambient in <2 seconds instead of extending forever.
      axis = axes[0]
      self.center = (axis[(len(axis)-1)//4],)
      self.spread = axis[3*(len(axis)-1)//4]-self.center[0]
    else:
      self.center = tuple(median(axis) for axis in axes)
      self.spread = math.sqrt(sum(median(abs(v-c) for v in axis)**2
                                 for axis, c in zip(axes, self.center, strict=True)))


class ActivityDetector:
  def __init__(self):
    self.baseline = {}
    self.first = {}
    self.last = {}
    self.above_since = {}
    self.last_trigger = -math.inf
    self.sound_envelope = 0.
    self.sound_value = 0.
    self.last_active = {}
    self.ambient = {}
    self.gyro_sample = None

  def update(self, service, value, timestamp, armed, audio_sensitivity=50, gyro_sensitivity=50):
    if service == 'gyroscope':
      self.gyro_sample = None
    values = (float(value),) if service == 'soundPressure' else tuple(value)
    if len(values) != (1 if service == 'soundPressure' else 3) or not all(math.isfinite(v) for v in values):
      return False
    previous = self.last.get(service)
    if previous is not None and timestamp <= previous:
      return False
    self.last[service] = timestamp
    sound_rise = values[0]-self.sound_value if service == 'soundPressure' else 0.
    if service == 'soundPressure':
      self.sound_value = values[0]
    if previous is None or timestamp-previous > FRESH_SECONDS:
      self.first[service] = timestamp
      self.baseline[service] = values
      self.above_since.pop(service, None)
      self.last_active.pop(service, None)
      if service in ('soundPressure', 'gyroscope'):
        self.ambient[service] = AmbientCalibration()
        self.ambient[service].update(values, timestamp)
      if service == 'soundPressure':
        self.sound_envelope = 0.
      return False

    dt = timestamp-previous
    baseline = self.baseline[service]
    # Shift the existing curve by 25 slider points: new 50% is old 25%.
    # Scale both the quiet floor and learned vibration threshold together.
    gyro_scale = 2.**((75.-min(100., max(0., float(gyro_sensitivity))))/50.)
    impulse = False
    if service == 'soundPressure':
      # micd's dB readings are uncalibrated. Use change above the ambient
      # floor, with a modest floor to reject numerical noise in near silence.
      threshold = max(audio_threshold(audio_sensitivity), 3*self.ambient[service].spread)
      strength = (values[0]-baseline[0])/threshold if values[0] >= 30. else 0.
      # micd reports one 100 ms RMS block at a time. A window knock can occupy
      # only one block, so waiting for a second loud sample misses it. Require
      # both a sharp rise and a meaningful excess over learned ambient noise.
      impulse = strength >= .6 and sound_rise >= .5*threshold
      confirm = .08
    else:
      # Subtract gravity/bias as a vector; no fixed mount axes or car model.
      # Learn background vibration as well as bias. Do not cap its noise floor:
      # a cap below actual vibration turns ordinary peaks into repeated hits.
      threshold = (.6 if service == 'accelerometer' else
                   gyro_scale*max(GYRO_THRESHOLD_RAD_S, 4*self.ambient[service].spread))  # m/s², rad/s
      strength = math.dist(values, baseline)/threshold
      # A brief initial jolt can end before 40 ms while its slower rebound
      # passes that gate. Strong noise spikes must meet the same 20 ms gate:
      # a short pair of large samples is not evidence of a physical shake.
      confirm = .02 if service == 'gyroscope' else .04
      if service == 'gyroscope':
        self.gyro_sample = {'gyroMagnitude': strength*threshold, 'gyroThreshold': threshold,
                            'gyroNoise': self.ambient[service].spread,
                            'gyroBaselineOffset': math.dist(self.ambient[service].center, baseline)}

    settling = timestamp-self.first[service] < SETTLE_SECONDS
    if service == 'soundPressure':
      # Preserve a short sound peak between 5 Hz UI publications, then decay.
      self.sound_envelope = (max(min(1., max(0., strength/2.)), self.sound_envelope*math.exp(-dt/.25))
                             if armed and not settling else 0.)
    if service in self.ambient:
      self.ambient[service].update(values, timestamp)
    if service == 'soundPressure':
      self.baseline[service] = self.ambient[service].center
    else:
      # Gyro bias follows the robust center slowly: a short rotation remains
      # motion. Acceleration retains the existing gravity-vector adaptation.
      target = self.ambient[service].center if service == 'gyroscope' else values
      if service == 'gyroscope' and not settling and math.dist(target, baseline) > gyro_scale*GYRO_THRESHOLD_RAD_S:
        target = baseline  # Sustained rotation is not a new stationary bias.
      alpha = 1-math.exp(-dt/(.5 if settling else 10.))
      self.baseline[service] = tuple(b+alpha*(v-b) for b, v in zip(baseline, target, strict=True))
    if not armed or settling:
      self.above_since.pop(service, None)
      self.last_active.pop(service, None)
      return False
    if strength < 1. and not impulse:
      self.above_since.pop(service, None)
      return False
    since = self.above_since.setdefault(service, timestamp)
    if impulse or service != 'gyroscope' and strength >= 2. or timestamp-since >= confirm:
      # Display current confirmed activity even inside the recording debounce.
      # A sound followed by a gyro hit must still show the larger gyro pulse.
      self.last_active[service] = timestamp
      if timestamp-self.last_trigger >= 2.:
        self.last_trigger = timestamp
        return True
    return False

  def ready(self, now):
    return all(service in self.first and now-self.first[service] >= SETTLE_SECONDS and
               0 <= now-self.last[service] <= FRESH_SECONDS for service in SENSOR_SERVICES)

  def audio_level(self, now):
    age = now-self.last.get('soundPressure', -math.inf)
    return self.sound_envelope*math.exp(-age/.25) if 0 <= age <= FRESH_SECONDS else 0.

  def active(self, now, service=None):
    return any(0 <= now-timestamp <= ACTIVITY_HOLD_SECONDS for source, timestamp in self.last_active.items()
               if service is None or source == service)


class SensorActivity:
  def __init__(self):
    from openpilot.cereal import messaging
    self.messaging = messaging
    # Drain every sample so short impacts are not lost between sentryd's 5 Hz
    # ticks. Nothing is written to disk and raw microphone audio is not consumed.
    self.sockets = {s: messaging.sub_sock(s, conflate=False) for s in SENSOR_SERVICES}
    self.detector = ActivityDetector()
    self.gyro = GyroHistory()
    self.gyro_diagnostics = {}

  def sample(self, now, armed, audio_sensitivity=50, gyro_sensitivity=50):
    triggered = False
    self.gyro_diagnostics = {}
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
          if service == 'gyroscope':
            baseline = self.detector.baseline.get(service)
            last = self.detector.last.get(service, -math.inf)
            if (baseline is not None and len(value) == 3 and 0 < timestamp-last <= FRESH_SECONDS and
                timestamp-self.detector.first[service] >= SETTLE_SECONDS):
              # The IRQ timestamp shares the camera monotonic clock; logMonoTime
              # includes publication latency. Synthetic/older events may omit it.
              sensor_time = data.timestamp/1e9 if data.timestamp else timestamp
              if 0 <= now-sensor_time <= FRESH_SECONDS:
                self.gyro.add(sensor_time, tuple(v-b for v, b in zip(value, baseline, strict=True)))
        triggered |= self.detector.update(service, value, timestamp, armed, audio_sensitivity, gyro_sensitivity)
        if service == 'gyroscope' and (sample := self.detector.gyro_sample) is not None:
          # Preserve the strongest sample relative to its threshold in this
          # controller tick, before later quiet samples overwrite the evidence.
          peak = self.gyro_diagnostics
          if not peak or sample['gyroMagnitude']/sample['gyroThreshold'] > peak['gyroMagnitude']/peak['gyroThreshold']:
            self.gyro_diagnostics = sample
    return triggered

  def ready(self, now):
    return self.detector.ready(now)

  def audio_level(self, now):
    return self.detector.audio_level(now)

  def active(self, now, service=None):
    return self.detector.active(now, service)
