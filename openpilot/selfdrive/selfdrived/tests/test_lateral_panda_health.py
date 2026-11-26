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


def test_recorded_single_can0_receive_loss_qualifies_only_after_stable_observation(inputs):
  cp, ps = inputs
  ps.canState0.totalErrorCnt = ps.canState0.totalRxLostCnt = 1
  health = LateralPandaHealth()
  # selfdrived's first observed sample can already have the startup loss.
  for i in range(21):
    ps.canState0.totalRxCnt += 100
    ps.canState2.totalRxCnt += 100
    assert health.check(cp, [ps], 100. + i / 10, True) == (i == 20)
  assert health.check(cp, [ps], 102.1, True)


@pytest.mark.parametrize('bus_index,total_errors,rx_lost', [(0, 1, 0), (0, 0, 1), (0, 2, 1),
                                                          (0, 1, 2), (0, 2, 2), (2, 1, 1)])
def test_receive_loss_allowance_is_confined_to_recorded_can0_counters(inputs, bus_index, total_errors, rx_lost):
  cp, ps = inputs
  bus = getattr(ps, f'canState{bus_index}')
  bus.totalErrorCnt, bus.totalRxLostCnt = total_errors, rx_lost
  health = LateralPandaHealth()
  for i in range(25):
    assert not health.check(cp, [ps], 100. + i / 10, True)


@pytest.mark.parametrize('field,value', [('busOff', True), ('errorWarning', True), ('errorPassive', True),
                                        ('receiveErrorCnt', 1), ('transmitErrorCnt', 1), ('totalTxLostCnt', 1),
                                        ('canCoreResetCnt', 1), ('busOffCnt', 1), ('lastError', 'stuffError'),
                                        ('lastStoredError', 'bit0Error'), ('lastDataError', 'ackError'),
                                        ('lastDataStoredError', 'crcError'), ('irq0CallRate', 16000),
                                        ('irq1CallRate', 16000), ('irq2CallRate', 16000), ('totalRxCnt', 0)])
def test_single_receive_loss_never_exempts_other_driving_bus_health(inputs, field, value):
  cp, ps = inputs
  ps.canState0.totalErrorCnt = ps.canState0.totalRxLostCnt = 1
  setattr(ps.canState0, field, value)
  health = LateralPandaHealth()
  for i in range(25):
    assert not health.check(cp, [ps], 100. + i / 10, True)


def test_single_receive_loss_requires_distinct_fresh_samples(inputs):
  cp, ps = inputs
  ps.canState0.totalErrorCnt = ps.canState0.totalRxLostCnt = 1
  health = LateralPandaHealth()
  for _ in range(100):
    assert not health.check(cp, [ps], 100., True)
  qualify(health, cp, ps)
  assert not health.check(cp, [ps], 102.1, False)
  qualify(health, cp, ps, start=102.2)


def test_further_receive_loss_invalidates_and_cannot_requalify(inputs):
  cp, ps = inputs
  ps.canState0.totalErrorCnt = ps.canState0.totalRxLostCnt = 1
  health = LateralPandaHealth()
  qualify(health, cp, ps)
  ps.canState0.totalErrorCnt = ps.canState0.totalRxLostCnt = 2
  for i in range(25):
    assert not health.check(cp, [ps], 102.1 + i / 10, True)


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


@pytest.mark.parametrize('fault', ['diagnostic_error', 'can0_receive_loss'])
def test_health_requalification_does_not_rearm_aol_after_new_fault(inputs, fault):
  cp, ps = inputs
  health = LateralPandaHealth()
  qualify(health, cp, ps)
  state = AlwaysOnLateralState()
  args = {'enabled': True, 'onroad': True, 'supported': True, 'firmware_allowed': True, 'firmware_suspended': False,
          'rearm': False}
  state.update(**args, ready=health.check(cp, [ps], 102.1, True), now=102.1)
  assert state.active
  if fault == 'diagnostic_error':
    ps.canState1.totalErrorCnt += 1
  else:
    ps.canState0.totalErrorCnt = ps.canState0.totalRxLostCnt = 1
  state.update(**args, ready=health.check(cp, [ps], 102.2, True), now=102.2)
  assert state.suspended and not state.active
  qualify(health, cp, ps, start=102.2)
  state.update(**args, ready=health.check(cp, [ps], 104.4, True), now=104.4)
  assert state.suspended and not state.active
  state.update(**(args | {'rearm': True}), ready=True, now=104.5)
  assert state.active


def observe_elm(health, cp, ps, loss, *, parked=True, can0_loss=0):
  ps.safetyModel, ps.safetyParam = 'elm327', 1
  ps.ignitionCan = True
  ps.canState2.totalRxLostCnt = loss
  ps.canState0.totalErrorCnt = ps.canState0.totalRxLostCnt = can0_loss
  assert not health.check(cp, [ps], 99.8, True)
  assert not health.check(cp, [ps], 99.9, True, parked=parked)
  ps.safetyModel, ps.safetyParam = 'gm', GM_AOL_PARAM


def check_with_rx(health, cp, ps, start=100., *, progressing=(True, True)):
  results = []
  for i in range(22):
    for bus, progress in zip((ps.canState0, ps.canState2), progressing, strict=True):
      bus.totalRxCnt += 100 if progress else 0
    results.append(health.check(cp, [ps], start + i / 10, True))
  return results


@pytest.mark.parametrize('loss', [1, 2, 3, 4, 5, 6, 65535])
@pytest.mark.parametrize('can0_loss', [0, 1])
def test_can2_fifo_loss_from_parked_startup_qualifies_with_live_traffic(inputs, loss, can0_loss):
  cp, ps = inputs
  health = LateralPandaHealth()
  observe_elm(health, cp, ps, loss, can0_loss=can0_loss)
  assert check_with_rx(health, cp, ps) == [False] * 20 + [True, True]


@pytest.mark.parametrize('can0_loss', [0, 1])
def test_zero_startup_fifo_loss_does_not_allow_a_later_loss(inputs, can0_loss):
  cp, ps = inputs
  health = LateralPandaHealth()
  observe_elm(health, cp, ps, 0, can0_loss=can0_loss)
  assert check_with_rx(health, cp, ps) == [False] * 20 + [True, True]
  ps.canState2.totalRxLostCnt = 5
  assert not any(check_with_rx(health, cp, ps, start=102.2))


@pytest.mark.parametrize('start_loss,later_loss', [(0, 1), (0, 5), (1, 2), (2, 1), (2, 3), (4, 5), (5, 4), (5, 6)])
@pytest.mark.parametrize('can0_loss', [0, 1])
def test_changed_fifo_loss_never_qualifies(inputs, start_loss, later_loss, can0_loss):
  cp, ps = inputs
  health = LateralPandaHealth()
  observe_elm(health, cp, ps, start_loss, can0_loss=can0_loss)
  ps.canState2.totalRxLostCnt = later_loss
  assert not any(check_with_rx(health, cp, ps))


def test_new_can2_loss_stops_aol_and_driver_rearm_cannot_bypass_health(inputs):
  cp, ps = inputs
  health = LateralPandaHealth()
  observe_elm(health, cp, ps, 5)
  assert check_with_rx(health, cp, ps)[-1]
  state = AlwaysOnLateralState()
  args = {'enabled': True, 'onroad': True, 'supported': True, 'firmware_allowed': True,
          'firmware_suspended': False, 'rearm': False}
  state.update(**args, ready=health.check(cp, [ps], 102.1, True), now=102.1)
  assert state.requested and state.active

  for count, start in ((6, 102.2), (5, 104.4)):
    ps.canState2.totalRxLostCnt = count
    for i, ready in enumerate(check_with_rx(health, cp, ps, start=start)):
      state.update(**(args | {'rearm': i == 21}), ready=ready, now=start + i / 10)
      assert state.suspended and not state.requested and not state.active


@pytest.mark.parametrize('change', ['fresh_host', 'missed_park', 'stale', 'reboot', 'harness', 'gap', 'elm_again',
                                    'can0_loss', 'can2_error', 'another_loss'])
@pytest.mark.parametrize('can0_loss', [0, 1])
def test_can2_qualification_requires_continuous_startup_history(inputs, change, can0_loss):
  cp, ps = inputs
  health = LateralPandaHealth()
  observe_elm(health, cp, ps, 5, parked=change != 'missed_park', can0_loss=can0_loss)
  if change == 'fresh_host':
    health = LateralPandaHealth()
  elif change == 'stale':
    assert not health.check(cp, [ps], 100., False)
  elif change == 'reboot':
    ps.uptime = 1
  elif change == 'harness':
    ps.harnessStatus = 'flipped'
  elif change == 'elm_again':
    assert not health.check(cp, [ps], 99.95, True)
    ps.safetyModel, ps.safetyParam = 'elm327', 1
    assert not health.check(cp, [ps], 99.98, True, parked=True)
    ps.safetyModel, ps.safetyParam = 'gm', GM_AOL_PARAM
  elif change == 'can0_loss':
    ps.canState0.totalErrorCnt = ps.canState0.totalRxLostCnt = can0_loss + 1
  elif change == 'can2_error':
    ps.canState2.totalErrorCnt = 1
  elif change == 'another_loss':
    assert check_with_rx(health, cp, ps)[-1]
    ps.canState2.totalRxLostCnt = 6
    assert not any(check_with_rx(health, cp, ps, start=102.2))
    ps.canState2.totalRxLostCnt = 5
    assert not any(check_with_rx(health, cp, ps, start=104.4))
    return
  assert not any(check_with_rx(health, cp, ps, start=100.5 if change == 'gap' else 100.))


@pytest.mark.parametrize('progressing', [(False, True), (True, False)])
@pytest.mark.parametrize('can0_loss', [0, 1])
def test_can2_historical_loss_requires_new_rx_on_both_driving_buses(inputs, progressing, can0_loss):
  cp, ps = inputs
  health = LateralPandaHealth()
  observe_elm(health, cp, ps, 5, can0_loss=can0_loss)
  assert check_with_rx(health, cp, ps)[-1]
  assert not any(check_with_rx(health, cp, ps, start=102.2, progressing=progressing))


@pytest.mark.parametrize('change', ['no_initial_fault', 'extra_fault', 'transport_error', 'disappears', 'rx_rewind'])
@pytest.mark.parametrize('can0_loss', [0, 1])
def test_startup_witness_requires_same_fault_and_continuous_panda(inputs, change, can0_loss):
  cp, ps = inputs
  health = LateralPandaHealth()
  ps.ignitionCan = True
  ps.safetyModel, ps.safetyParam = 'elm327', 1
  ps.canState2.totalRxLostCnt = 5
  ps.canState0.totalErrorCnt = ps.canState0.totalRxLostCnt = can0_loss
  if change == 'no_initial_fault':
    ps.faults = []
  elif change == 'extra_fault':
    ps.faults = ['interruptRateCan2', 'relayMalfunction']
  elif change == 'transport_error':
    ps.rxBufferOverflow = 1
  assert health.check(cp, [ps], 99.8, True, parked=True) == (change == 'no_initial_fault')
  ps.faults = ['interruptRateCan2']
  ps.rxBufferOverflow = 0
  if change == 'disappears':
    ps.faults = []
    assert health.check(cp, [ps], 99.9, True)
    ps.faults = ['interruptRateCan2']
  elif change == 'rx_rewind':
    ps.canState0.totalRxCnt -= 500
  ps.safetyModel, ps.safetyParam = 'gm', GM_AOL_PARAM
  assert not any(check_with_rx(health, cp, ps))


@pytest.mark.parametrize('qualified', [False, True])
@pytest.mark.parametrize('total_errors,rx_lost', [(0, 0), (1, 0), (0, 1), (2, 1), (1, 2), (2, 2)])
def test_mixed_startup_losses_require_unchanged_can0_pair(inputs, qualified, total_errors, rx_lost):
  cp, ps = inputs
  health = LateralPandaHealth()
  observe_elm(health, cp, ps, 5, can0_loss=1)
  if qualified:
    assert check_with_rx(health, cp, ps)[-1]
  ps.canState0.totalErrorCnt, ps.canState0.totalRxLostCnt = total_errors, rx_lost
  start = 102.2 if qualified else 100.
  assert not any(check_with_rx(health, cp, ps, start=start))
  # Restoring an old count cannot recreate the discarded startup evidence.
  ps.canState0.totalErrorCnt = ps.canState0.totalRxLostCnt = 1
  assert not any(check_with_rx(health, cp, ps, start=start + 2.2))


@pytest.mark.parametrize('total_errors,rx_lost', [(0, 0), (1, 0), (0, 1), (2, 1), (1, 2), (2, 2)])
def test_can0_pair_must_be_present_in_first_can2_startup_witness(inputs, total_errors, rx_lost):
  cp, ps = inputs
  health = LateralPandaHealth()
  ps.safetyModel, ps.safetyParam = 'elm327', 1
  ps.ignitionCan = True
  ps.canState2.totalRxLostCnt = 5
  ps.canState0.totalErrorCnt, ps.canState0.totalRxLostCnt = total_errors, rx_lost
  assert not health.check(cp, [ps], 99.8, True)
  ps.canState0.totalErrorCnt = ps.canState0.totalRxLostCnt = 1
  assert not health.check(cp, [ps], 99.9, True, parked=True)
  ps.safetyModel, ps.safetyParam = 'gm', GM_AOL_PARAM
  assert not any(check_with_rx(health, cp, ps))


@pytest.mark.parametrize('field,value', [('busOff', True), ('errorWarning', True), ('errorPassive', True),
                                        ('receiveErrorCnt', 1), ('transmitErrorCnt', 1), ('totalTxLostCnt', 1),
                                        ('canCoreResetCnt', 1), ('busOffCnt', 1), ('lastError', 'stuffError'),
                                        ('lastStoredError', 'bit0Error'), ('lastDataError', 'ackError'),
                                        ('lastDataStoredError', 'crcError'), ('irq0CallRate', 16000),
                                        ('irq1CallRate', 16000), ('irq2CallRate', 16000)])
@pytest.mark.parametrize('bus_index', [0, 2])
def test_mixed_startup_losses_do_not_exempt_live_driving_bus_errors(inputs, field, value, bus_index):
  cp, ps = inputs
  health = LateralPandaHealth()
  observe_elm(health, cp, ps, 5, can0_loss=1)
  assert check_with_rx(health, cp, ps)[-1]
  setattr(getattr(ps, f'canState{bus_index}'), field, value)
  assert not any(check_with_rx(health, cp, ps, start=102.2))


@pytest.fixture
def retained_history(inputs):
  cp, ps = inputs
  # Route BD: these totals and stored codes were present in ELM, before the
  # first valid Park observation, and unchanged during the recorded drive.
  ps.canState0.totalErrorCnt, ps.canState0.totalRxLostCnt = 2, 1
  ps.canState0.lastStoredError = 'formError'
  ps.canState2.totalErrorCnt, ps.canState2.totalRxLostCnt = 1, 2
  return cp, ps


def observe_retained_history(health, cp, ps, *, parked=True):
  ps.safetyModel, ps.safetyParam = 'elm327', 1
  ps.ignitionCan = True
  assert not health.check(cp, [ps], 99.8, True)
  assert not health.check(cp, [ps], 99.9, True, parked=parked)
  ps.safetyModel, ps.safetyParam = 'gm', GM_AOL_PARAM


def test_bd_retained_error_history_qualifies_with_clean_current_status(retained_history):
  cp, ps = retained_history
  health = LateralPandaHealth()
  observe_retained_history(health, cp, ps)
  assert check_with_rx(health, cp, ps) == [False] * 20 + [True, True]
  # The history is preserved; qualification does not clear or rewrite it.
  assert (ps.canState0.totalErrorCnt, ps.canState0.totalRxLostCnt) == (2, 1)
  assert ps.canState0.lastStoredError == 'formError'
  assert (ps.canState2.totalErrorCnt, ps.canState2.totalRxLostCnt) == (1, 2)


@pytest.mark.parametrize('bus_index', [0, 2])
@pytest.mark.parametrize('field,value', [('lastError', 'formError'), ('lastDataError', 'crcError'),
                                        ('receiveErrorCnt', 1), ('transmitErrorCnt', 1),
                                        ('busOff', True), ('errorWarning', True), ('errorPassive', True),
                                        ('totalTxLostCnt', 1), ('busOffCnt', 1), ('canCoreResetCnt', 1),
                                        ('irq0CallRate', 16000), ('irq1CallRate', 16000), ('irq2CallRate', 16000)])
def test_retained_history_never_exempts_live_error_at_startup(retained_history, bus_index, field, value):
  cp, ps = retained_history
  health = LateralPandaHealth()
  setattr(getattr(ps, f'canState{bus_index}'), field, value)
  observe_retained_history(health, cp, ps)
  assert not any(check_with_rx(health, cp, ps))


@pytest.mark.parametrize('qualified', [False, True])
@pytest.mark.parametrize('bus_index', [0, 2])
@pytest.mark.parametrize('field,value', [('totalErrorCnt', 9), ('totalRxLostCnt', 9),
                                        ('lastStoredError', 'ackError'), ('lastDataStoredError', 'crcError')])
def test_retained_history_change_is_not_rebaselined(retained_history, qualified, bus_index, field, value):
  cp, ps = retained_history
  health = LateralPandaHealth()
  observe_retained_history(health, cp, ps)
  if qualified:
    assert check_with_rx(health, cp, ps)[-1]
  bus = getattr(ps, f'canState{bus_index}')
  previous = getattr(bus, field)
  setattr(bus, field, value)
  start = 102.2 if qualified else 100.
  assert not any(check_with_rx(health, cp, ps, start=start))
  setattr(bus, field, previous)
  assert not any(check_with_rx(health, cp, ps, start=start + 2.2))


@pytest.mark.parametrize('change', ['direct_gm', 'missed_park', 'host_restart', 'stale', 'gap', 'panda_restart',
                                    'extra_fault', 'transport_error', 'history_reset'])
def test_retained_history_needs_continuous_parked_startup_witness(retained_history, change):
  cp, ps = retained_history
  health = LateralPandaHealth()
  if change == 'direct_gm':
    assert not any(check_with_rx(health, cp, ps))
    return
  observe_retained_history(health, cp, ps, parked=change != 'missed_park')
  if change == 'host_restart':
    health = LateralPandaHealth()
  elif change == 'stale':
    assert not health.check(cp, [ps], 100., False)
  elif change == 'panda_restart':
    ps.uptime = 1
  elif change == 'extra_fault':
    ps.faults = ['interruptRateCan2', 'relayMalfunction']
  elif change == 'transport_error':
    ps.spiErrorCount = 1
  elif change == 'history_reset':
    assert check_with_rx(health, cp, ps)[-1]
    # A reset to canonical legacy values must not bypass a lost witness.
    for bus in (ps.canState0, ps.canState2):
      bus.totalErrorCnt = bus.totalRxLostCnt = 0
      bus.lastStoredError = bus.lastDataStoredError = 'noError'
    assert not any(check_with_rx(health, cp, ps, start=102.2))
    return
  assert not any(check_with_rx(health, cp, ps, start=100.5 if change == 'gap' else 100.))


@pytest.mark.parametrize('bus_index', [0, 2])
@pytest.mark.parametrize('errors,lost,stored', [(1, 0, 'formError'), (8, 13, 'noError'), (100, 55, 'crcError')])
def test_retained_history_uses_witnessed_values_not_exact_count_allowlist(inputs, bus_index, errors, lost, stored):
  cp, ps = inputs
  bus = getattr(ps, f'canState{bus_index}')
  bus.totalErrorCnt, bus.totalRxLostCnt, bus.lastStoredError = errors, lost, stored
  health = LateralPandaHealth()
  observe_retained_history(health, cp, ps)
  assert check_with_rx(health, cp, ps) == [False] * 20 + [True, True]


@pytest.mark.parametrize('progressing', [(False, True), (True, False)])
def test_bd_retained_history_requires_live_rx_on_both_buses(retained_history, progressing):
  cp, ps = retained_history
  health = LateralPandaHealth()
  observe_retained_history(health, cp, ps)
  assert not any(check_with_rx(health, cp, ps, progressing=progressing))


def test_unknown_extended_history_cannot_reset_into_legacy_allowance(retained_history):
  cp, ps = retained_history
  health = LateralPandaHealth()
  assert not any(check_with_rx(health, cp, ps))  # first observation already in GM
  for bus in (ps.canState0, ps.canState2):
    bus.totalErrorCnt = bus.totalRxLostCnt = 0
    bus.lastStoredError = bus.lastDataStoredError = 'noError'
  assert not any(check_with_rx(health, cp, ps, start=102.2))


def test_changed_retained_error_history_cannot_be_bypassed_by_driver_rearm(retained_history):
  cp, ps = retained_history
  health = LateralPandaHealth()
  observe_retained_history(health, cp, ps)
  assert check_with_rx(health, cp, ps)[-1]
  state = AlwaysOnLateralState()
  args = {'enabled': True, 'onroad': True, 'supported': True, 'firmware_allowed': True,
          'firmware_suspended': False, 'rearm': False}
  state.update(**args, ready=health.check(cp, [ps], 102.1, True), now=102.1)
  assert state.active
  ps.canState0.lastStoredError = 'ackError'
  for i, ready in enumerate(check_with_rx(health, cp, ps, start=102.2)):
    state.update(**(args | {'rearm': i == 21}), ready=ready, now=102.2 + i / 10)
    assert state.suspended and not state.requested and not state.active

