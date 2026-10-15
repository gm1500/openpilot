from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.common.params import Params, ParamKeyFlag
from openpilot.system.sentry.config import apply_parking_default, parking_mode, select_parking_mode, LOW_RISK, HIGH_RISK


@pytest.fixture
def parking(tmp_path):
  params = Params(str(tmp_path/'params'))
  params.put_bool('SentryEnabled', True, block=True)
  params.put_bool('IsOffroad', True, block=True)
  sm = {'deviceState': SimpleNamespace(started=False, networkType=log.DeviceState.NetworkType.wifi),
        'pandaStates': [SimpleNamespace(ignitionLine=False, ignitionCan=False)]}
  sm = type('SM', (dict,), {'valid': dict.fromkeys(sm, True), 'recv_time': dict.fromkeys(sm, 100.)})(sm)
  return params, sm


@pytest.mark.parametrize('enabled', [False, True])
@pytest.mark.parametrize('skip', [False, True])
@pytest.mark.parametrize('wifi', [False, True])
def test_parking_default_without_prompt_and_master_switch_is_independent(parking, enabled, skip, wifi):
  params, sm = parking
  params.put_bool('SentryEnabled', enabled, block=True)
  params.put_bool('SentrySkipOnWifi', skip, block=True)
  sm['deviceState'].networkType = log.DeviceState.NetworkType.wifi if wifi else log.DeviceState.NetworkType.none
  apply_parking_default(params, sm, 100.)
  assert parking_mode(params) == ('off' if wifi and skip else LOW_RISK)
  assert params.get('SentryParkingModeSource') == ('wifi' if wifi and skip else 'default')
  assert params.get_bool('SentryEnabled') == enabled
  assert not params.get_bool('SentryModePrompted')  # The controller never consumes the prompt.
  sm['deviceState'].networkType = log.DeviceState.NetworkType.none if wifi else log.DeviceState.NetworkType.wifi
  apply_parking_default(params, sm, 100.)
  assert parking_mode(params) == ('off' if wifi and skip else LOW_RISK)  # No silent rearming or reselection.


def test_prompt_dismissal_does_not_skip_the_controller_wifi_default(parking):
  params, sm = parking
  params.put_bool('SentryModePrompted', True, block=True)
  apply_parking_default(params, sm, 100.)
  assert parking_mode(params) == 'off' and params.get('SentryParkingModeSource') == 'wifi'


@pytest.mark.parametrize('blocker', ['onroad', 'started', 'ignition_line', 'ignition_can', 'no_panda',
                                  'device_invalid', 'device_stale', 'panda_invalid', 'panda_stale'])
def test_default_waits_for_fresh_parked_state(parking, blocker):
  params, sm = parking
  if blocker == 'onroad':
    params.put_bool('IsOffroad', False, block=True)
  elif blocker == 'started':
    sm['deviceState'].started = True
  elif blocker.startswith('ignition'):
    setattr(sm['pandaStates'][0], 'ignitionLine' if blocker.endswith('line') else 'ignitionCan', True)
  elif blocker == 'no_panda':
    sm['pandaStates'] = []
  else:
    service = 'deviceState' if blocker.startswith('device') else 'pandaStates'
    if blocker.endswith('invalid'):
      sm.valid[service] = False
    else:
      sm.recv_time[service] = 90.
  apply_parking_default(params, sm, 100.)
  assert params.get('SentryParkingModeSource') is None
  assert not params.get_bool('SentryOffThisDrive')


@pytest.mark.parametrize('choice', [LOW_RISK, HIGH_RISK, 'off'])
@pytest.mark.parametrize('default_first', [False, True])
def test_explicit_choice_wins_and_resets_only_for_next_drive(parking, choice, default_first):
  params, sm = parking
  if default_first:
    apply_parking_default(params, sm, 100.)
  select_parking_mode(params, choice)
  apply_parking_default(params, sm, 100.)
  assert parking_mode(params) == choice
  assert params.get('SentryParkingModeSource') == 'manual'
  assert params.get('SentryArmResetAt') is not None
  assert params.get_bool('SentryModePrompted') and params.get_bool('SentryEnabled')
  # A choice made while driving can apply to the upcoming parking session.
  params.clear_all(ParamKeyFlag.CLEAR_ON_OFFROAD_TRANSITION)
  assert parking_mode(params) == choice
  assert params.get('SentryParkingModeSource') == 'manual'
  params.clear_all(ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION)
  assert parking_mode(params) == LOW_RISK
  assert not params.get_bool('SentryModePrompted')
  assert params.get('SentryParkingModeSource') is None
  assert params.get('SentryArmResetAt') is None
  assert params.get_bool('SentryEnabled')
  apply_parking_default(params, sm, 100.)
  assert parking_mode(params) == 'off' and params.get('SentryParkingModeSource') == 'wifi'
