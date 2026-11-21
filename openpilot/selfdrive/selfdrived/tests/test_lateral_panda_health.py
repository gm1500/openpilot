import pytest

from opendbc.car.gm.values import CAR
from opendbc.car.structs import car
from openpilot.cereal import log
from openpilot.selfdrive.selfdrived.always_on_lateral import AlwaysOnLateralState
from openpilot.selfdrive.selfdrived.lateral_panda_health import GM_AOL_PARAM, LateralPandaHealth


@pytest.fixture
def inputs():
  cp = car.CarParams.new_message(brand='gm', carFingerprint=CAR.CHEVROLET_SILVERADO,
                                 openpilotLongitudinalControl=True, networkLocation='fwdCamera',
                                 safetyConfigs=[{'safetyModel': 'gm', 'safetyParam': GM_AOL_PARAM}])
  ps = log.PandaState.new_message(pandaType='tres', safetyModel='gm', safetyParam=GM_AOL_PARAM,
                                  lateralSupported=True, harnessStatus='normal', uptime=100,
                                  faults=['interruptRateCan2'], faultStatus='faultTemp')
  for bus in (ps.canState0, ps.canState2):
    bus.totalRxCnt = 1000
    bus.irq0CallRate = 1700
    bus.irq1CallRate = 180
  # These diagnostic-bus counters remain latched after the observed startup
  # burst. Requiring them to become zero would recreate the original failure.
  d = ps.canState1
  d.totalErrorCnt, d.totalTxCnt, d.totalTxLostCnt, d.canCoreResetCnt = 74251, 2, 1, 3
  d.receiveErrorCnt, d.transmitErrorCnt = 127, 8
  d.errorWarning = d.errorPassive = True
  d.lastError = 'noChange'
  d.lastStoredError = 'stuffError'
  return cp, ps


def qualify(health, cp, ps, start=100.):
  results = [health.check(cp, [ps], start + i / 10, True) for i in range(21)]
  assert results[:-1] == [False] * 20
  assert results[-1]


def test_recorded_startup_diagnostic_requires_two_seconds_of_distinct_quiet_samples(inputs):
  cp, ps = inputs
  health = LateralPandaHealth()
  qualify(health, cp, ps)
  assert health.check(cp, [ps], 102., True)  # the same qualified sample is safe to reuse
  assert health.check(cp, [ps], 102.1, True)


def test_repeated_packet_cannot_qualify(inputs):
  cp, ps = inputs
  health = LateralPandaHealth()
  for _ in range(1000):
    assert not health.check(cp, [ps], 100., True)
  assert not health.check(cp, [ps], 102., True)  # a sample gap starts a fresh window
  qualify(health, cp, ps, start=102.)


@pytest.mark.parametrize('gap', [0., -1., float('nan'), float('inf'), 101., 102.31])
def test_bad_timestamp_or_gap_invalidates_qualification(inputs, gap):
  cp, ps = inputs
  health = LateralPandaHealth()
  qualify(health, cp, ps)
  assert not health.check(cp, [ps], gap, True)


def test_stale_packet_resets_quiet_period(inputs):
  cp, ps = inputs
  health = LateralPandaHealth()
  qualify(health, cp, ps)
  assert not health.check(cp, [ps], 102.1, False)
  qualify(health, cp, ps, start=102.2)


@pytest.mark.parametrize('field', ['irq0CallRate', 'irq1CallRate', 'irq2CallRate', 'totalRxCnt', 'totalFwdCnt', 'busOff'])
def test_diagnostic_bus_activity_always_blocks(inputs, field):
  cp, ps = inputs
  health = LateralPandaHealth()
  qualify(health, cp, ps)
  setattr(ps.canState1, field, True if field == 'busOff' else 1)
  for i in range(25):
    assert not health.check(cp, [ps], 102.1 + i / 10, True)


@pytest.mark.parametrize('field', ['totalErrorCnt', 'totalTxLostCnt', 'totalRxLostCnt', 'totalTxCnt',
                                  'canCoreResetCnt', 'busOffCnt', 'receiveErrorCnt', 'transmitErrorCnt'])
def test_changed_diagnostic_counter_discards_qualification(inputs, field):
  cp, ps = inputs
  health = LateralPandaHealth()
  qualify(health, cp, ps)
  setattr(ps.canState1, field, getattr(ps.canState1, field) + 1)
  assert not health.check(cp, [ps], 102.1, True)
  assert not health.check(cp, [ps], 102.2, True)


@pytest.mark.parametrize('field,value', [('errorWarning', False), ('errorPassive', False), ('lastError', 'stuffError'),
                                        ('lastStoredError', 'ackError'), ('lastDataError', 'stuffError'),
                                        ('lastDataStoredError', 'stuffError'), ('canSpeed', 250),
                                        ('canDataSpeed', 1000), ('canfdEnabled', True), ('brsEnabled', True),
                                        ('canfdNonIso', True)])
def test_diagnostic_status_changes_also_restart_observation(inputs, field, value):
  cp, ps = inputs
  health = LateralPandaHealth()
  qualify(health, cp, ps)
  setattr(ps.canState1, field, value)
  assert not health.check(cp, [ps], 102.1, True)


@pytest.mark.parametrize('bus_index', [0, 2])
@pytest.mark.parametrize('field,value', [('busOff', True), ('errorWarning', True), ('errorPassive', True),
                                        ('receiveErrorCnt', 1), ('transmitErrorCnt', 1), ('totalErrorCnt', 1),
                                        ('totalTxLostCnt', 1), ('totalRxLostCnt', 1), ('canCoreResetCnt', 1),
                                        ('busOffCnt', 1), ('irq0CallRate', 16000), ('irq1CallRate', 16000),
                                        ('irq2CallRate', 16000), ('totalRxCnt', 0), ('lastError', 'stuffError'),
                                        ('lastDataError', 'ackError'), ('lastStoredError', 'bit0Error'),
                                        ('lastDataStoredError', 'crcError')])
def test_driving_bus_errors_or_missing_health_cannot_use_exception(inputs, bus_index, field, value):
  cp, ps = inputs
  health = LateralPandaHealth()
  qualify(health, cp, ps)
  setattr(getattr(ps, f'canState{bus_index}'), field, value)
  assert not health.check(cp, [ps], 102.1, True)


@pytest.mark.parametrize('field', ['safetyRxChecksInvalid', 'heartbeatLost', 'nmiReset', 'hardfaultReset'])
@pytest.mark.parametrize('with_fault', [False, True])
def test_integrity_gates_apply_even_without_fault_bits(inputs, field, with_fault):
  cp, ps = inputs
  if not with_fault:
    ps.faults = []
    ps.faultStatus = 'none'
  setattr(ps, field, True)
  assert not LateralPandaHealth().check(cp, [ps], 100., True)


@pytest.mark.parametrize('field', ['safetyRxInvalid', 'rxBufferOverflow', 'txBufferOverflow', 'spiErrorCount'])
def test_transport_integrity_counters_deny_exception(inputs, field):
  cp, ps = inputs
  setattr(ps, field, 1)
  health = LateralPandaHealth()
  for i in range(21):
    assert not health.check(cp, [ps], 100. + i / 10, True)


@pytest.mark.parametrize('faults', [['interruptRateCan1'], ['interruptRateCan3'], ['interruptRateSpi'],
                                   ['safetyRedundancy'], ['interruptRateCan2', 'relayMalfunction']])
def test_no_other_fault_or_combination_is_exempt(inputs, faults):
  cp, ps = inputs
  ps.faults = faults
  health = LateralPandaHealth()
  for i in range(21):
    assert not health.check(cp, [ps], 100. + i / 10, True)


@pytest.mark.parametrize('field,value', [('pandaType', 'dos'), ('pandaType', 'redPanda'), ('pandaType', 'cuatro'),
                                        ('harnessStatus', 'notConnected'), ('safetyParam', 11),
                                        ('lateralSupported', False), ('alternativeExperience', 1),
                                        ('faultStatus', 'faultPerm')])
def test_exception_is_confined_to_matching_verified_hardware_and_config(inputs, field, value):
  cp, ps = inputs
  setattr(ps, field, value)
  health = LateralPandaHealth()
  for i in range(21):
    assert not health.check(cp, [ps], 100. + i / 10, True)


@pytest.mark.parametrize('field,value', [('carFingerprint', 'OTHER'), ('brand', 'toyota'), ('passive', True),
                                        ('notCar', True), ('pcmCruise', True), ('openpilotLongitudinalControl', False),
                                        ('networkLocation', 'gateway')])
def test_other_vehicle_configurations_remain_blocked(inputs, field, value):
  cp, ps = inputs
  setattr(cp, field, value)
  health = LateralPandaHealth()
  for i in range(21):
    assert not health.check(cp, [ps], 100. + i / 10, True)


def test_multiple_or_absent_pandas_do_not_use_exception(inputs):
  cp, ps = inputs
  health = LateralPandaHealth()
  assert not health.check(cp, [], 100., True)
  assert not health.check(cp, [ps, ps], 100., True)


@pytest.mark.parametrize('change', ['uptime', 'harness', 'alternative_experience'])
def test_configuration_or_board_restart_discards_previous_quiet_window(inputs, change):
  cp, ps = inputs
  health = LateralPandaHealth()
  qualify(health, cp, ps)
  if change == 'uptime':
    ps.uptime = 1
  elif change == 'harness':
    ps.harnessStatus = 'flipped'
  else:
    cp.alternativeExperience = ps.alternativeExperience = 1
  assert not health.check(cp, [ps], 102.1, True)


def test_no_fault_path_does_not_need_exception_or_quiet_window(inputs):
  cp, ps = inputs
  ps.faults = []
  ps.faultStatus = 'none'
  ps.pandaType = 'dos'
  assert LateralPandaHealth().check(cp, [ps], 100., True)


def test_health_requalification_does_not_rearm_aol_after_new_fault(inputs):
  cp, ps = inputs
  health = LateralPandaHealth()
  qualify(health, cp, ps)
  state = AlwaysOnLateralState()
  args = {'enabled': True, 'onroad': True, 'supported': True, 'firmware_allowed': True, 'firmware_suspended': False,
          'rearm': False}
  state.update(**args, ready=health.check(cp, [ps], 102.1, True), now=102.1)
  assert state.active
  ps.canState1.totalErrorCnt += 1
  state.update(**args, ready=health.check(cp, [ps], 102.2, True), now=102.2)
  assert state.suspended and not state.active
  qualify(health, cp, ps, start=102.2)
  state.update(**args, ready=health.check(cp, [ps], 104.4, True), now=104.4)
  assert state.suspended and not state.active
  state.update(**(args | {'rearm': True}), ready=True, now=104.5)
  assert state.active
