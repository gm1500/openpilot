from types import SimpleNamespace

import numpy as np
import pytest

from opendbc.car.structs import car
from openpilot.cereal import log
from openpilot.system.sentry import sentryd
from openpilot.system.sentry.config import DEFAULTS, CAMERA_WARMUP_SECONDS
from openpilot.system.sentry.location import vehicle_identity
from openpilot.system.sentry.motion import Motion


@pytest.mark.parametrize('first,second,third', [(0, 1, 2), (1, 2, 0), (2, 0, 1)])
def test_preview_follows_confirmed_motion_in_first_detection_order(first, second, third):
  since = {}
  results = {stream: Motion() for stream in range(3)}
  results[first] = Motion(active=True, score=.1, triggered=True)
  camera = sentryd.trigger_source(results, 2, since, 100.)
  assert camera == first
  # Stronger motion in an overlapping camera must not steal the preview.
  results[second] = Motion(active=True, score=.8, triggered=True)
  assert sentryd.trigger_source(results, camera, since, 101.) == first
  results[third] = Motion(active=True, score=1., triggered=True)
  assert sentryd.trigger_source(results, camera, since, 102.) == first
  # As a person leaves one view, the next already-confirmed lens takes over.
  results[first] = Motion(active=True, score=.9, triggered=False)
  camera = sentryd.trigger_source(results, camera, since, 103.)
  assert camera == second
  results[second] = Motion()
  assert sentryd.trigger_source(results, camera, since, 104.) == third


@pytest.mark.parametrize('enabled,expected', [((0, 1, 2), 2), ((0, 1), 0), ((1,), 1), ((2,), 2)])
def test_manual_preview_uses_an_enabled_camera_and_eye_only_motion_does_not_switch(enabled, expected):
  results = {stream: Motion() for stream in enabled}
  assert sentryd.trigger_source(results, 2, {}, 100., manual=True) == expected
  results[enabled[0]] = Motion(active=True, score=.5)
  assert sentryd.trigger_source(results, expected, {}, 101.) == expected
  assert sentryd.trigger_source(results, expected, {}, 101., manual=True) == enabled[0]


@pytest.mark.parametrize('stream', [0, 1, 2])
@pytest.mark.parametrize('size', [(1928, 1208), (1344, 760)])
def test_live_camera_sampling_applies_lens_area_and_one_second_confirmation(monkeypatch, stream, size):
  width, height = size
  image = np.full((height, width), 70, dtype=np.uint8)
  client = SimpleNamespace(width=width, height=height, stride=width, frame_id=0, timestamp_eof=0,
                           is_connected=lambda: True, recv=lambda timeout: SimpleNamespace(data=image.tobytes()))
  monkeypatch.setattr(sentryd, 'VisionIpcClient', lambda *args: client)
  camera = sentryd.CameraMotion(stream)
  for i in range(24):
    now = 100.+i*.2
    client.frame_id, client.timestamp_eof = i, int(now*1e9)
    image.fill(70)
    if i >= 15:
      x = 65+i-15
      image[25*height//80:35*height//80, x*width//128:(x+12)*width//128] = 170
    result = camera.sample(now, 75, armed=i >= 15)
    assert result.active == (i >= 15)
    assert result.triggered == (stream in (1, 2) and i >= 20)


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
def test_warmup_restarts_for_camera_loss_but_not_recorder_gaps(monkeypatch, tmp_path, arm_delay, eye_only, recorder_gap):
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
    def __init__(self, stream):
      self.stream, self.last_received = stream, 0.
    def sample(self, now, sensitivity, armed):
      samples.append((now, armed))
      if now < {0: 135., 1: ready_at, 2: 140.}[self.stream] or (self.stream == 1 and drop_at <= now < restart_at):
        return Motion()
      self.last_received = now
      # Deliberately return motion even while unarmed: the controller must gate
      # both automatic requests and the eye independently of the detector.
      ready = arm_at if now < drop_at else rearm_at
      confirmed = not eye_only and (not armed or now >= ready+arm_delay+1.)
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
    assert last_gap['recordUntilMonoTime'] == sent[round(saved_at-.4, 1)]['recordUntilMonoTime']
    assert (sent[saved_at]['recordUntilMonoTime'] > last_gap['recordUntilMonoTime']) == (not eye_only)
  assert all(not s['motion'] for now, s in sent.items() if drop_at+1. <= now < rearm_at+arm_delay)
  # The 15-second recording tail can outlast the restarted camera warmup.
  assert sent[rearm_at+3.]['phase'] == f'Arming: {arm_delay-3} s'
  assert sent[rearm_at+arm_delay-1.]['phase'] == 'Arming: 1 s'
  assert sent[rearm_at+arm_delay+1.]['motion']
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
