from types import SimpleNamespace

import numpy as np
import pytest

from opendbc.car.structs import car
from openpilot.cereal import log
from openpilot.system.sentry import sentryd
from openpilot.system.sentry.config import DEFAULTS, CAMERA_WARMUP_SECONDS
from openpilot.system.sentry.location import vehicle_identity
from openpilot.system.sentry.motion import Motion


@pytest.mark.parametrize('first,second', [(1, 2), (2, 1)])
def test_preview_follows_strongest_current_wide_motion(first, second):
  results = {stream: Motion() for stream in range(3)}
  results[first] = Motion(active=True, score=.1, triggered=True)
  camera = sentryd.trigger_source(results, 2)
  assert camera == first
  # Once recording, stronger motion takes over on its very first sample,
  # even while the original lens still has confirmed motion.
  results[second] = Motion(active=True, score=.8, triggered=False)
  camera = sentryd.trigger_source(results, camera, recording=True)
  assert camera == second
  results[0] = Motion(active=True, score=1., triggered=True)  # Narrow cannot take preview priority.
  assert sentryd.trigger_source(results, camera, recording=True) == second
  # Equal scores retain the current view; a larger score switches back.
  results[first] = Motion(active=True, score=.8, triggered=True)
  assert sentryd.trigger_source(results, camera, recording=True) == second
  results[first] = Motion(active=True, score=.9, triggered=False)
  assert sentryd.trigger_source(results, camera, recording=True) == first
  results[first] = results[second] = Motion()
  assert sentryd.trigger_source(results, camera, recording=True) == second


def test_new_event_preview_uses_confirmed_source_even_with_larger_eye_only_motion():
  results = {1: Motion(active=True, score=.8), 2: Motion(active=True, score=.1, triggered=True)}
  assert sentryd.trigger_source(results, 1) == 2
  assert sentryd.trigger_source(results, 2, recording=True) == 1


@pytest.mark.parametrize('enabled,expected', [((1, 2), 2), ((1,), 1), ((2,), 2)])
def test_manual_preview_uses_a_wide_camera_and_eye_only_motion_waits_for_recording(enabled, expected):
  results = {stream: Motion() for stream in enabled}
  assert sentryd.trigger_source(results, 2, manual=True) == expected
  results[enabled[0]] = Motion(active=True, score=.5)
  assert sentryd.trigger_source(results, expected) == expected
  assert sentryd.trigger_source(results, expected, manual=True) == enabled[0]


@pytest.mark.parametrize('stream', [0, 1, 2])
@pytest.mark.parametrize('size', [(1928, 1208), (1344, 760)])
def test_live_camera_sampling_applies_lens_area_and_short_confirmation(monkeypatch, stream, size):
  width, height = size
  image = np.full((height, width), 70, dtype=np.uint8)
  client = SimpleNamespace(width=width, height=height, stride=width, frame_id=0, timestamp_eof=0,
                           is_connected=lambda: True, recv=lambda timeout: SimpleNamespace(data=image.tobytes()))
  monkeypatch.setattr(sentryd, 'VisionIpcClient', lambda *args: client)
  camera = sentryd.CameraMotion(stream)
  for i in range(74):
    now = 100.+i*.2
    client.frame_id, client.timestamp_eof = i, int(now*1e9)
    image.fill(70)
    if i >= 65:
      x = 65+i-65
      image[25*height//80:35*height//80, x*width//128:(x+12)*width//128] = 170
    result = camera.sample(now, 75, armed=i >= 65)
    assert result.active == (i >= 65)
    assert result.triggered == (stream in (1, 2) and i >= 68)


@pytest.mark.parametrize('restart', ['reconnect', 'frame_counter', 'size', 'frame_gap'])
def test_camera_recovery_learns_exposure_without_replaying_arm_delay(monkeypatch, restart):
  client = SimpleNamespace(width=640, height=400, stride=640, frame_id=1000, timestamp_eof=0, connected=True)
  image = [None]
  client.is_connected = lambda: client.connected
  client.connect = lambda blocking: setattr(client, 'connected', True) or True
  client.recv = lambda timeout: SimpleNamespace(data=image[0].tobytes())
  monkeypatch.setattr(sentryd, 'VisionIpcClient', lambda *args: client)
  camera = sentryd.CameraMotion(1)

  def sample(now, small):
    client.frame_id += 1
    client.timestamp_eof = int(now*1e9)
    image[0] = np.repeat(np.repeat(small, client.height//80, axis=0), client.width//128, axis=1).astype(np.uint8)
    return camera.sample(now, 75, armed=True)

  base = np.full((80, 128), 70, dtype=np.uint8)
  for i in range(60):
    sample(round(100+i*.2, 1), base)
  assert camera.warm_until == 110.
  if restart == 'reconnect':
    client.connected = False
  elif restart == 'frame_counter':
    client.frame_id = -1
  elif restart == 'size':
    client.width, client.stride = 1280, 1280

  for i in range(50 if restart != 'frame_gap' else 15):
    frame = base.copy()
    if restart != 'frame_gap':
      # A spatial IR/exposure ramp must remain suppressed through recovery.
      frame[20:50, 30:90] += i*2
    result = sample(round(120+i*.2, 1), frame)
    assert not result.active and not result.triggered
  assert camera.warm_until == (110. if restart == 'frame_gap' else 130.)
  start = 123. if restart == 'frame_gap' else 130.
  for i in range(8):
    frame = base.copy()
    frame[20:50, 30:90] += 98 if restart != 'frame_gap' else 0
    frame[25:40, 50+i*3:65+i*3] = 250
    result = sample(round(start+i*.2, 1), frame)
  assert result.active and result.triggered  # No extra 15/30/60-second user delay.


@pytest.mark.parametrize('post_seconds', [15, 30, 60])
def test_quiet_eye_and_later_lens_extend_event_without_rearming(monkeypatch, tmp_path, post_seconds):
  class Params(dict):
    get_bool = dict.get
    def remove(self, key):
      self.pop(key, None)

  params = Params(DEFAULTS | {'IsOffroad': True, 'SentryArmDelay': 30, 'SentryPostSeconds': post_seconds})
  clock, sent, samples = [100.], {}, []
  first, second = 145., 145.+post_seconds-3.
  end = second+1.+post_seconds
  third = end+3.

  class SubMaster(dict):
    def __init__(self, services):
      super().__init__({
        'deviceState': SimpleNamespace(started=False, maxTempC=50.),
        'pandaStates': [SimpleNamespace(pandaType=log.PandaState.PandaType.tres, ignitionLine=False, ignitionCan=False)],
        'peripheralState': SimpleNamespace(voltage=12000),
        'sentryRecorderState': SimpleNamespace(recording=False, error='', bufferBytes=1000),
        'gpsLocation': SimpleNamespace(hasFix=False), 'gpsLocationExternal': SimpleNamespace(hasFix=False),
      })
      self.valid = {s: not s.startswith('gps') for s in services}
      self.recv_time, self.logMonoTime = {}, {}

    def update(self, timeout):
      now = clock[0]
      self.recv_time = dict.fromkeys(self, now)
      self.logMonoTime = dict.fromkeys(self, int(now*1e9))
      previous = sent.get(round(now-.2, 1))
      self['sentryRecorderState'].recording = bool(previous and previous['recordUntilMonoTime'] > now*1e9)
      # Changing the arming preset applies at the next capture start, not after
      # the eye disappears or while an event is still open.
      if now == first+2.:
        params['SentryArmDelay'] = 60

  class Camera:
    warm_until = 0.
    def __init__(self, stream):
      self.stream, self.last_received = stream, 0.
    def sample(self, now, sensitivity, armed):
      self.last_received = now
      samples.append((now, armed))
      burst = any(t <= now <= t+1 for t in ((first, third) if self.stream == 2 else (second,)))
      # Also show a brief eye-only hit: it must not extend the route.
      eye_only = self.stream == 1 and first+5 <= now <= first+5.4
      return Motion(burst or eye_only, .3 if eye_only else .2, .8, .4, triggered=burst)

  class Finished(Exception):
    pass

  class Ratekeeper:
    def __init__(self, *args, **kwargs):
      pass
    def keep_time(self):
      clock[0] = round(clock[0]+.2, 1)
      if clock[0] > third+1.:
        raise Finished

  monkeypatch.setattr(sentryd.time, 'monotonic', lambda: clock[0])
  monkeypatch.setattr(sentryd, 'Params', lambda: params)
  monkeypatch.setattr(sentryd, 'SentryClock', lambda params: SimpleNamespace(update=lambda *args: None, snapshot=lambda wall: ('', 0)))
  monkeypatch.setattr(sentryd, 'HARDWARE', SimpleNamespace(get_device_type=lambda: 'tizi'))
  monkeypatch.setattr(sentryd.messaging, 'SubMaster', SubMaster)
  monkeypatch.setattr(sentryd.messaging, 'PubMaster',
                      lambda services: SimpleNamespace(send=lambda service, msg: sent.update({clock[0]: msg.to_dict()['sentryState']})))
  monkeypatch.setattr(sentryd, 'CameraMotion', Camera)
  monkeypatch.setattr(sentryd, 'Ratekeeper', Ratekeeper)
  monkeypatch.setattr(sentryd, 'write_runtime', lambda state: None)
  monkeypatch.setattr(sentryd, 'RUNTIME_PATH', tmp_path/'runtime.json')
  with pytest.raises(Finished):
    sentryd.main()

  assert all(armed for now, armed in samples if now >= 142.)
  assert not any(s['phase'].startswith(('Arming:', 'Camera warmup:')) for now, s in sent.items() if now >= 142.)
  for now in (first+2., first+6., second-.2):
    assert not sent[round(now, 1)]['motion']
    assert sent[round(now, 1)]['recordUntilMonoTime'] == int((first+1.+post_seconds)*1e9)
  assert sent[first+5.]['motion']
  assert sent[first+5.]['triggerCamera'] == 1  # The first unconfirmed sample hands off an open route.
  assert sent[first]['triggerCamera'] == 2 and sent[second]['triggerCamera'] == 1
  assert sent[second+1.]['recordUntilMonoTime'] == int(end*1e9)
  assert sent[end-.2]['phase'] == 'Recording'
  assert sent[end]['phase'] == 'Watching'
  assert sent[third]['recordUntilMonoTime'] == int((third+post_seconds)*1e9)


@pytest.mark.parametrize('cached,fix_after,video_ready', [(True, None, True), (False, 20, True), (False, None, True), (True, None, False)])
@pytest.mark.parametrize('stop_reason', ['ignition', 'disabled'])
def test_controller_requests_gps_only_until_snapshot_or_timeout(monkeypatch, tmp_path, cached, fix_after, video_ready, stop_reason):
  clock = [100.]
  wall = 10000.
  CP = car.CarParams.new_message()
  CP.carFingerprint, CP.carVin = 'test-car', 'test-vin'
  checkpoint = {'version': 1, 'vehicle': vehicle_identity(CP), 'latitude': 53.5, 'longitude': -113.5,
                'accuracy': 6., 'savedAt': wall-60}

  class Params(dict):
    get_bool = dict.get
    def remove(self, key):
      self.pop(key, None)

  params = Params(DEFAULTS | {'SentryEnabled': False, 'IsOffroad': True, 'CarParamsPersistent': CP.to_bytes(),
                            'MapParkedPosition': checkpoint if cached else None})
  runtime = []
  sent = []

  class SubMaster(dict):
    def __init__(self, services):
      super().__init__({
        'deviceState': SimpleNamespace(started=False, maxTempC=50., offroadPowerUsageUwh=0),
        'pandaStates': [SimpleNamespace(pandaType=log.PandaState.PandaType.tres, ignitionLine=False, ignitionCan=False)],
        'peripheralState': SimpleNamespace(voltage=12000),
        'sentryRecorderState': SimpleNamespace(recording=False, error='', bufferBytes=1000 if video_ready else 0),
        'gpsLocation': SimpleNamespace(hasFix=True, latitude=53.51, longitude=-113.51, horizontalAccuracy=8.),
        'gpsLocationExternal': SimpleNamespace(hasFix=False),
      })
      self.valid = dict.fromkeys(services, True)
      self.valid['gpsLocationExternal'] = False
      self.recv_time, self.logMonoTime = {}, {}

    def update(self, timeout):
      self.recv_time = dict.fromkeys(self, clock[0])
      self.logMonoTime = dict.fromkeys(self, int(clock[0]*1e9))
      self.valid['gpsLocation'] = fix_after is not None and clock[0]-100 >= fix_after
      # Mimic enabling the existing UI while already parked, then a manual save
      # before the arming delay, and finally disabling Sentry or ignition returning.
      params['SentryEnabled'] = clock[0] >= 101. and not (stop_reason == 'disabled' and clock[0] >= 199.)
      if clock[0] == 104.:
        params['SentryManualTrigger'] = True
      self['pandaStates'][0].ignitionLine = stop_reason == 'ignition' and clock[0] >= 199.

  class Finished(Exception):
    pass

  class Ratekeeper:
    def __init__(self, *args, **kwargs):
      pass
    def keep_time(self):
      clock[0] += 1.
      if clock[0] >= 200.:
        raise Finished

  monkeypatch.setattr(sentryd.time, 'monotonic', lambda: clock[0])
  monkeypatch.setattr(sentryd.time, 'time', lambda: wall+clock[0]-100)
  monkeypatch.setattr(sentryd, 'Params', lambda: params)
  monkeypatch.setattr(sentryd, 'SentryClock', lambda params: SimpleNamespace(update=lambda *args: None, snapshot=lambda wall: ('', 0)))
  monkeypatch.setattr(sentryd, 'HARDWARE', SimpleNamespace(get_device_type=lambda: 'tizi'))
  monkeypatch.setattr(sentryd.messaging, 'SubMaster', SubMaster)
  monkeypatch.setattr(sentryd.messaging, 'PubMaster', lambda services: SimpleNamespace(send=lambda service, msg: sent.append(msg.to_dict())))
  monkeypatch.setattr(sentryd, 'Ratekeeper', Ratekeeper)
  class Camera:
    warm_until = 0.
    last_received = 0.
    def sample(self, now, sensitivity, armed):
      self.last_received = now
      return Motion()

  monkeypatch.setattr(sentryd, 'CameraMotion', lambda stream: Camera())
  monkeypatch.setattr(sentryd, 'write_runtime', lambda state: runtime.append(state))
  monkeypatch.setattr(sentryd, 'RUNTIME_PATH', tmp_path/'runtime.json')
  with pytest.raises(Finished):
    sentryd.main()
  assert runtime[0]['phase'] == 'Off' and not runtime[0]['capture']
  assert runtime[1]['session'] and not runtime[1]['capture']
  assert runtime[3]['capture']
  assert sent[4]['sentryState']['recordUntilMonoTime'] > 104e9
  assert runtime[-2]['phase'] == ('Watching' if video_ready else 'Waiting for encoded video')
  assert runtime[-1]['phase'] == ('Off' if stop_reason == 'disabled' else 'Waiting for ignition off')
  assert not runtime[-1]['capture'] and not runtime[-1]['gps'] and not runtime[-1]['session']
  assert sent[-1]['sentryState']['recordUntilMonoTime'] == 0
  # Discard disabled/onroad samples for the parking-location assertions.
  runtime = runtime[1:-1]
  if cached:
    assert not any(state['gps'] for state in runtime)
    assert runtime[-1]['location']['source'] == 'slcParked'
    assert runtime[-1]['location']['savedAt'] == checkpoint['savedAt']
  elif fix_after is not None:
    assert all(state['gps'] for state in runtime[:fix_after-1])
    assert not any(state['gps'] for state in runtime[fix_after-1:])
    assert runtime[-1]['location']['source'] == 'gpsSnapshot'
    assert runtime[-1]['location']['savedAt'] == wall+fix_after
  else:
    assert all(state['gps'] for state in runtime[:90])
    assert not any(state['gps'] for state in runtime[90:])
    assert runtime[-1]['location'] is None


@pytest.mark.parametrize('arm_delay', [15, 30, 60])
@pytest.mark.parametrize('eye_only', [False, True])
@pytest.mark.parametrize('recorder_gap', [None, 'heartbeat', 'buffer'])
def test_arming_survives_frame_and_recorder_gaps(monkeypatch, tmp_path, arm_delay, eye_only, recorder_gap):
  class Params(dict):
    get_bool = dict.get
    def remove(self, key):
      self.pop(key, None)

  params = Params(DEFAULTS | {'IsOffroad': True, 'SentryArmDelay': arm_delay, 'SentryPostSeconds': 15})
  clock = [100.]
  ready_at = 145.
  arm_at = ready_at+CAMERA_WARMUP_SECONDS
  saving_at = arm_at+arm_delay+1.2
  saved_at = arm_at+arm_delay+4.
  drop_at = arm_at+arm_delay+5.
  restart_at = drop_at+3.
  rearm_at = restart_at+CAMERA_WARMUP_SECONDS
  stop_at = rearm_at+arm_delay+5.
  sent = {}
  samples = []

  class SubMaster(dict):
    def __init__(self, services):
      super().__init__({
        'deviceState': SimpleNamespace(started=False, maxTempC=50.),
        'pandaStates': [SimpleNamespace(pandaType=log.PandaState.PandaType.tres, ignitionLine=False, ignitionCan=False)],
        'peripheralState': SimpleNamespace(voltage=12000),
        'sentryRecorderState': SimpleNamespace(recording=False, error='', bufferBytes=0),
        'gpsLocation': SimpleNamespace(hasFix=False),
        'gpsLocationExternal': SimpleNamespace(hasFix=False),
      })
      self.valid = {s: not s.startswith('gps') for s in services}
      self.recv_time, self.logMonoTime = {}, {}

    def update(self, timeout):
      now = clock[0]
      self.recv_time = dict.fromkeys(self, now)
      self.logMonoTime = dict.fromkeys(self, int(now*1e9))
      self['sentryRecorderState'].bufferBytes = 1000 if now >= 130. else 0
      previous = sent.get(round(now-.2, 1))
      self['sentryRecorderState'].recording = bool(previous and previous['recordUntilMonoTime'] > now*1e9)
      # Persisting pre-roll can delay recorder status without restarting any
      # cameras. Also cover recorder-only recovery with an empty RAM buffer.
      if recorder_gap and saving_at <= now < saved_at:
        self['sentryRecorderState'].recording = False
        if recorder_gap == 'heartbeat':
          self.recv_time['sentryRecorderState'] = saving_at-.2
        else:
          self['sentryRecorderState'].bufferBytes = 0
      if now == ready_at+2.:
        params['SentryManualTrigger'] = True
      if now >= stop_at:
        params['SentryEnabled'] = False

  class Camera:
    warm_until = 0.
    def __init__(self, stream):
      self.stream, self.last_received = stream, 0.
    def sample(self, now, sensitivity, armed):
      samples.append((now, armed))
      if now < {0: 135., 1: ready_at, 2: 140.}[self.stream] or (self.stream == 1 and drop_at <= now < restart_at):
        return Motion()
      self.last_received = now
      # Deliberately return motion even while unarmed: the controller must gate
      # both automatic requests and the eye independently of the detector.
      confirmed = not eye_only and (not armed or now >= arm_at+arm_delay+1.)
      return Motion(True, .2, .8, .4, triggered=confirmed)

  class Finished(Exception):
    pass

  class Ratekeeper:
    def __init__(self, *args, **kwargs):
      pass
    def keep_time(self):
      clock[0] = round(clock[0]+.2, 1)
      if clock[0] > stop_at:
        raise Finished

  monkeypatch.setattr(sentryd.time, 'monotonic', lambda: clock[0])
  monkeypatch.setattr(sentryd, 'Params', lambda: params)
  monkeypatch.setattr(sentryd, 'SentryClock', lambda params: SimpleNamespace(update=lambda *args: None, snapshot=lambda wall: ('', 0)))
  monkeypatch.setattr(sentryd, 'HARDWARE', SimpleNamespace(get_device_type=lambda: 'tizi'))
  monkeypatch.setattr(sentryd.messaging, 'SubMaster', SubMaster)
  monkeypatch.setattr(sentryd.messaging, 'PubMaster',
                      lambda services: SimpleNamespace(send=lambda service, msg: sent.update({clock[0]: msg.to_dict()['sentryState']})))
  monkeypatch.setattr(sentryd, 'Ratekeeper', Ratekeeper)
  monkeypatch.setattr(sentryd, 'CameraMotion', Camera)
  monkeypatch.setattr(sentryd, 'write_runtime', lambda state: None)
  monkeypatch.setattr(sentryd, 'RUNTIME_PATH', tmp_path/'runtime.json')
  with pytest.raises(Finished):
    sentryd.main()

  assert sent[ready_at]['phase'] == f'Camera warmup: {CAMERA_WARMUP_SECONDS:g} s'
  # Manual saves bypass both delays without letting exposure changes extend them.
  # The manual tail finishes at arm_at+7; the arming clock must still be running.
  assert sent[arm_at+8.]['phase'] == f'Arming: {arm_delay-8} s'
  assert all(not s['motion'] for now, s in sent.items() if now < arm_at+arm_delay)
  assert all(s['recordUntilMonoTime'] == 0 for now, s in sent.items() if now < ready_at+2.)
  manual_until = int((ready_at+17.)*1e9)
  assert all(s['recordUntilMonoTime'] == manual_until for now, s in sent.items() if ready_at+2. <= now < arm_at+arm_delay)
  assert min(now for now, armed in samples if armed) == arm_at+arm_delay
  assert sent[arm_at+arm_delay+.4]['motion']
  assert sent[arm_at+arm_delay+.4]['recordUntilMonoTime'] == manual_until
  assert (sent[arm_at+arm_delay+1.2]['recordUntilMonoTime'] > manual_until) == (not eye_only)
  assert sent[saved_at]['motion']  # Recorder recovery must not replay arming.
  assert sent[saved_at]['phase'] == ('Watching' if eye_only else 'Recording')
  assert all(armed for now, armed in samples if saving_at <= now < drop_at)
  if recorder_gap:
    assert all(s['capture'] and s['motion'] for now, s in sent.items() if saving_at <= now < saved_at)
    last_gap = sent[round(saved_at-.2, 1)]
    assert last_gap['phase'] == ('Starting recorder' if recorder_gap == 'heartbeat' else 'Waiting for encoded video')
    assert (last_gap['recordUntilMonoTime'] > sent[round(saved_at-.4, 1)]['recordUntilMonoTime']) == (not eye_only)
    assert (sent[saved_at]['recordUntilMonoTime'] > last_gap['recordUntilMonoTime']) == (not eye_only)
  # A missing cabin sample must not disarm the healthy wide camera, replay the
  # user's delay, or stop confirmed motion from extending the open event.
  assert all(s['motion'] for now, s in sent.items() if drop_at <= now < stop_at)
  assert all(armed for now, armed in samples if arm_at+arm_delay <= now < stop_at)
  assert all(not s['phase'].startswith(('Arming:', 'Camera warmup:')) for now, s in sent.items()
             if arm_at+arm_delay <= now < stop_at)
  assert sent[restart_at]['phase'] == ('Watching' if eye_only else 'Recording')
  assert not sent[stop_at]['capture'] and not sent[stop_at]['motion']
  assert sent[stop_at]['recordUntilMonoTime'] == 0


@pytest.mark.parametrize('blocker,expected', [
  ('disabled', 'Off'), ('hardware', 'Requires comma 3X'), ('shutdown', 'Shutting down'),
  ('panda', 'Waiting for panda'), ('unknown', 'Waiting for panda'),
  ('ignition', 'Waiting for ignition off'), ('device', 'Waiting for device state'),
  ('started', 'Waiting for offroad'), ('offroad', 'Waiting for offroad'), ('none', ''),
])
def test_activation_blockers_are_visible(blocker, expected):
  class State(dict):
    valid = {'pandaStates': True, 'deviceState': True}
    recv_time = {'pandaStates': 100., 'deviceState': 100.}

  class Params(dict):
    get_bool = dict.get

  params = Params(SentryEnabled=blocker != 'disabled', IsOffroad=blocker != 'offroad', DoShutdown=blocker == 'shutdown')
  sm = State(deviceState=SimpleNamespace(started=blocker == 'started'),
             pandaStates=[SimpleNamespace(pandaType=log.PandaState.PandaType.unknown if blocker == 'unknown' else log.PandaState.PandaType.tres,
                                          ignitionLine=blocker == 'ignition', ignitionCan=False)])
  if blocker in ('panda', 'device'):
    sm.recv_time['pandaStates' if blocker == 'panda' else 'deviceState'] = 95.
  assert sentryd.inactive_reason(params, sm, 100., 'pc' if blocker == 'hardware' else 'tizi') == expected


@pytest.mark.parametrize('shutdown', [False, True])
def test_cooling_resumes_only_before_parking_expiry_and_detection_uses_wide_cameras(monkeypatch, tmp_path, shutdown):
  class Params(dict):
    get_bool = dict.get
    def remove(self, key):
      self.pop(key, None)

  params = Params(DEFAULTS | {'IsOffroad': True, 'SentryMaxHours': 1,
                            'SentryShutdownOnTimeLimit': shutdown, 'SentryShutdownOnOverheat': shutdown})
  # Capture, heat pause, a bad reading, recovery, then another pause extending
  # through the parking limit. Neither cooling nor a health/enable gap renews it.
  temperatures = {100: 50., 102: 50., 143: 50., 150: 70., 160: 66., 170: float('nan'),
                  180: 63., 190: 62., 231: 50., 250: 72., 3700: 72., 3701: 40.,
                  3702: 40., 3703: 40., 3704: 40., 3705: 40., 3710: 40., 3711: 40., 3713: 40.}
  times = list(temperatures)
  index = [0]
  def clock():
    return float(times[index[0]])
  runtime, sent, opened = {}, {}, []

  class SubMaster(dict):
    def __init__(self, services):
      super().__init__({
        'deviceState': SimpleNamespace(started=False, maxTempC=50.),
        'pandaStates': [SimpleNamespace(pandaType=log.PandaState.PandaType.tres, ignitionLine=False, ignitionCan=False)],
        'peripheralState': SimpleNamespace(voltage=12500),
        'sentryRecorderState': SimpleNamespace(recording=False, error='', bufferBytes=1000),
        'gpsLocation': SimpleNamespace(hasFix=False), 'gpsLocationExternal': SimpleNamespace(hasFix=False),
      })
      self.valid = {s: not s.startswith('gps') for s in services}
      self.recv_time, self.logMonoTime = {}, {}

    def update(self, timeout):
      now = clock()
      self.recv_time = dict.fromkeys(self, now)
      self.logMonoTime = dict.fromkeys(self, int(now*1e9))
      self['deviceState'].maxTempC = temperatures[now]
      self['pandaStates'][0].ignitionLine = now == 3710.
      self.valid['deviceState'] = now != 3704.
      params['SentryEnabled'] = now != 3702.

  class Camera:
    warm_until = 0.
    def __init__(self, stream):
      opened.append(stream)
      self.last_received = 0.
    def sample(self, now, sensitivity, armed):
      self.last_received = now
      return Motion()

  class Finished(Exception):
    pass

  class Ratekeeper:
    def __init__(self, *args, **kwargs):
      pass
    def keep_time(self):
      index[0] += 1
      if index[0] == len(times):
        raise Finished

  monkeypatch.setattr(sentryd.time, 'monotonic', clock)
  monkeypatch.setattr(sentryd, 'Params', lambda: params)
  monkeypatch.setattr(sentryd, 'SentryClock', lambda params: SimpleNamespace(update=lambda *args: None, snapshot=lambda wall: ('', 0)))
  monkeypatch.setattr(sentryd, 'HARDWARE', SimpleNamespace(get_device_type=lambda: 'tizi'))
  monkeypatch.setattr(sentryd.messaging, 'SubMaster', SubMaster)
  monkeypatch.setattr(sentryd.messaging, 'PubMaster',
                      lambda services: SimpleNamespace(send=lambda service, msg: sent.update({clock(): msg.to_dict()['sentryState']})))
  monkeypatch.setattr(sentryd, 'CameraMotion', Camera)
  monkeypatch.setattr(sentryd, 'Ratekeeper', Ratekeeper)
  monkeypatch.setattr(sentryd, 'write_runtime', lambda state: runtime.update({clock(): state}))
  monkeypatch.setattr(sentryd, 'RUNTIME_PATH', tmp_path/'runtime.json')
  with pytest.raises(Finished):
    sentryd.main()

  assert runtime[143]['phase'] == runtime[231]['phase'] == 'Watching'
  for now in (150, 160, 180, 250):
    assert runtime[now]['phase'] == 'Cooling down' and not runtime[now]['capture']
  assert runtime[170]['phase'] == 'Waiting for temperature reading' and not runtime[170]['capture']
  assert runtime[190]['capture'] and runtime[190]['phase'] == 'Camera warmup: 10 s'
  for now in (3700, 3701, 3703, 3705):
    assert runtime[now]['phase'] == 'Parking budget reached' and not runtime[now]['capture']
    assert runtime[now]['parkingStartedAt'] == 100.
    assert runtime[now]['parkingLimitReached']
  assert runtime[3702]['phase'] == 'Off' and runtime[3704]['phase'] == 'Waiting for device state'
  assert runtime[3710]['phase'] == 'Waiting for ignition off'
  assert runtime[3711]['parkingStartedAt'] == 3711. and not runtime[3711]['parkingLimitReached']
  assert runtime[3713]['capture']
  assert set(opened) == {1, 2}  # Narrow is recorded, never opened by the detector.
  assert all(state['cameraMask'] == 0b111 for state in sent.values())
