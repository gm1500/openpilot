from types import SimpleNamespace

import pytest

from opendbc.car.structs import car
from openpilot.cereal import log
from openpilot.system.sentry import sentryd
from openpilot.system.sentry.config import DEFAULTS
from openpilot.system.sentry.location import vehicle_identity
from openpilot.system.sentry.motion import Motion


@pytest.mark.parametrize('cached,fix_after', [(True, None), (False, 20), (False, None)])
def test_controller_requests_gps_only_until_snapshot_or_timeout(monkeypatch, tmp_path, cached, fix_after):
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

  params = Params(DEFAULTS | {'SentryEnabled': True, 'IsOffroad': True, 'CarParamsPersistent': CP.to_bytes(),
                            'MapParkedPosition': checkpoint if cached else None})
  runtime = []

  class SubMaster(dict):
    def __init__(self, services):
      super().__init__({
        'deviceState': SimpleNamespace(started=False, maxTempC=50., offroadPowerUsageUwh=0),
        'pandaStates': [SimpleNamespace(pandaType=log.PandaState.PandaType.tres, ignitionLine=False, ignitionCan=False)],
        'peripheralState': SimpleNamespace(voltage=12600),
        'sentryRecorderState': SimpleNamespace(recording=False, error=''),
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
  monkeypatch.setattr(sentryd, 'HARDWARE', SimpleNamespace(get_device_type=lambda: 'tizi'))
  monkeypatch.setattr(sentryd.messaging, 'SubMaster', SubMaster)
  monkeypatch.setattr(sentryd.messaging, 'PubMaster', lambda services: SimpleNamespace(send=lambda *args: None))
  monkeypatch.setattr(sentryd, 'Ratekeeper', Ratekeeper)
  monkeypatch.setattr(sentryd, 'CameraMotion', lambda stream: SimpleNamespace(sample=lambda *args: Motion()))
  monkeypatch.setattr(sentryd, 'write_runtime', lambda state: runtime.append(state))
  monkeypatch.setattr(sentryd, 'RUNTIME_PATH', tmp_path/'runtime.json')
  with pytest.raises(Finished):
    sentryd.main()
  assert runtime[-1]['capture']
  if cached:
    assert not any(state['gps'] for state in runtime)
    assert runtime[-1]['location']['source'] == 'slcParked'
    assert runtime[-1]['location']['savedAt'] == checkpoint['savedAt']
  elif fix_after is not None:
    assert all(state['gps'] for state in runtime[:fix_after])
    assert not any(state['gps'] for state in runtime[fix_after:])
    assert runtime[-1]['location']['source'] == 'gpsSnapshot'
    assert runtime[-1]['location']['savedAt'] == wall+fix_after
  else:
    assert all(state['gps'] for state in runtime[:90])
    assert not any(state['gps'] for state in runtime[90:])
    assert runtime[-1]['location'] is None
