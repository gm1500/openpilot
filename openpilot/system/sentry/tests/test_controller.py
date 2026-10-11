from types import SimpleNamespace

import pytest

from opendbc.car.structs import car
from openpilot.cereal import log
from openpilot.system.sentry import sentryd
from openpilot.system.sentry.config import DEFAULTS
from openpilot.system.sentry.location import vehicle_identity
from openpilot.system.sentry.motion import Motion


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
    def sample(self, now, sensitivity):
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
