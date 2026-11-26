from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from openpilot.cereal import log
from openpilot.common.constants import CV
from opendbc.car.structs import car
from opendbc.car.gm.values import CAR, GMSafetyFlags
from openpilot.selfdrive.selfdrived.always_on_lateral import (
  AlwaysOnLateralState, lateral_events_clear, lateral_safety_supported,
)
from openpilot.selfdrive.selfdrived.events import Events, ET, EVENTS, EVENT_NAME
from openpilot.selfdrive.selfdrived.lateral_panda_health import LateralPandaHealth
from openpilot.selfdrive.monitoring.policy import DriverMonitoring

EventName = log.OnroadEvent.EventName


def step(state, **kwargs):
  inputs = {'enabled': True, 'onroad': True, 'supported': True, 'ready': True, 'firmware_allowed': True,
            'firmware_suspended': False, 'rearm': False, 'now': 10.}
  state.update(**(inputs | kwargs))
  return state


def test_boot_arms_only_after_readiness_and_firmware_acknowledgement():
  state = AlwaysOnLateralState()
  step(state, ready=False, firmware_allowed=False)
  assert not state.requested and not state.active and not state.suspended
  step(state, firmware_allowed=False)
  assert state.requested and not state.active
  step(state)
  assert state.requested and state.active


@pytest.mark.parametrize('gate', ['enabled', 'onroad', 'supported', 'ready', 'firmware_allowed'])
def test_each_startup_gate_prevents_steering(gate):
  state = step(AlwaysOnLateralState(), **{gate: False})
  assert not state.active


def test_fault_latches_across_readiness_changes_until_explicit_rearm():
  state = step(AlwaysOnLateralState())
  step(state, ready=False)
  assert state.suspended and not state.active and not state.requested and state.just_stopped
  step(state, ready=False)
  step(state, now=20.)
  assert state.suspended and not state.active
  step(state, rearm=True, now=20., firmware_suspended=True, firmware_allowed=False)
  assert state.requested and not state.active
  step(state, now=20.1)
  assert state.active and not state.suspended


def test_brake_or_non_drive_pauses_without_cancelling_saved_intent():
  state = step(AlwaysOnLateralState())
  step(state, paused=True)
  assert state.paused and not state.active and not state.requested and not state.suspended
  step(state, paused=True, now=20.)
  assert not state.suspended
  step(state, now=21.)
  assert state.active


def test_fault_during_temporary_pause_remains_latched_after_release():
  state = step(AlwaysOnLateralState())
  step(state, paused=True)
  step(state, paused=True, ready=False)
  assert state.suspended
  step(state, paused=True, ready=True)
  step(state, now=20.)
  assert state.suspended and not state.requested
  step(state, now=21., rearm=True)
  assert state.active


@pytest.mark.parametrize('event', [EventName.wrongGear, EventName.reverseGear, EventName.parkBrake])
def test_temporary_gear_and_brake_pause_is_separate_from_fault_latch(event):
  events = Events()
  events.add(event)
  assert not lateral_events_clear(events)
  assert lateral_events_clear(events, paused=True)
  events.add(EventName.commIssue)
  assert not lateral_events_clear(events, paused=True)


@pytest.mark.parametrize('gate', ['ready', 'supported'])
def test_fault_loss_does_not_silently_resume(gate):
  state = step(AlwaysOnLateralState())
  step(state, **{gate: False})
  assert state.suspended and not state.active
  step(state, now=20.)
  assert not state.requested
  step(state, enabled=False)
  step(state)
  assert state.active


def test_firmware_fault_requires_driver_action():
  state = step(AlwaysOnLateralState())
  step(state, firmware_suspended=True, firmware_allowed=False, now=11.)
  assert state.suspended and not state.requested
  step(state, firmware_suspended=False, now=12.)
  assert state.suspended and not state.requested


def test_rearm_grace_cannot_ignore_a_firmware_latch_indefinitely():
  state = step(AlwaysOnLateralState(), rearm=True, firmware_suspended=True, firmware_allowed=False)
  assert state.requested and not state.active
  step(state, now=10.5, firmware_suspended=True, firmware_allowed=False)
  assert state.suspended and not state.requested


def test_invalid_clock_never_requests_permission():
  state = step(AlwaysOnLateralState(), now=float('nan'))
  assert not state.requested and not state.active


@pytest.mark.parametrize('event', [EventName.buttonCancel, EventName.pedalPressed, EventName.preEnableStandstill, EventName.pcmDisable,
                                   EventName.resumeBlocked, EventName.belowEngageSpeed])
def test_longitudinal_disengagement_does_not_block_lateral(event):
  events = Events()
  events.add(event)
  assert lateral_events_clear(events)


@pytest.mark.parametrize('event', [EventName.wrongGear, EventName.reverseGear,
                                   EventName.parkBrake, EventName.steerUnavailable, EventName.calibrationIncomplete,
                                   EventName.commIssue, EventName.controlsMismatch, EventName.tooDistracted,
                                   EventName.doorOpen, EventName.seatbeltNotLatched, EventName.wrongCarMode])
def test_existing_entry_and_fault_guards_are_retained(event):
  events = Events()
  events.add(event)
  assert not lateral_events_clear(events)


def safety_messages():
  CP = car.CarParams.new_message()
  CP.brand = 'gm'
  CP.carFingerprint = CAR.CHEVROLET_SILVERADO
  CP.minSteerSpeed = -1
  CP.openpilotLongitudinalControl = True
  CP.safetyConfigs = [{'safetyModel': 'gm', 'safetyParam': int(GMSafetyFlags.LATERAL_ONLY | GMSafetyFlags.HW_CAM |
                                                            GMSafetyFlags.HW_CAM_LONG | GMSafetyFlags.SILVERADO_TORQUE)}]
  ps = log.PandaState.new_message()
  ps.safetyModel = 'gm'
  ps.safetyParam = CP.safetyConfigs[0].safetyParam
  ps.lateralSupported = True
  return CP, [ps]


def test_legacy_firmware_and_safety_mismatch_are_unsupported():
  CP, pandas = safety_messages()
  assert lateral_safety_supported(CP, pandas)
  pandas[0].lateralSupported = False
  assert not lateral_safety_supported(CP, pandas)
  pandas[0].lateralSupported = True
  pandas[0].safetyParam = 0
  assert not lateral_safety_supported(CP, pandas)
  assert not lateral_safety_supported(CP, [])


@pytest.mark.parametrize('field', ['passive', 'notCar', 'pcmCruise'])
def test_unsupported_control_modes_never_request_lateral_permission(field):
  CP, pandas = safety_messages()
  setattr(CP, field, True)
  assert not lateral_safety_supported(CP, pandas)


@pytest.mark.parametrize('requested,active,expected', [(False, False, False), (True, False, True), (True, True, True)])
def test_lateral_only_uses_full_driver_monitoring(requested, active, expected):
  sm = {'carState': SimpleNamespace(vEgo=25., gearShifter=car.CarState.GearShifter.drive,
                                   steeringPressed=False, gasPressed=False, steeringAngleDeg=0.),
        'selfdriveState': SimpleNamespace(enabled=False, alwaysOnLateralRequested=requested, alwaysOnLateralActive=active),
        'modelV2': SimpleNamespace(meta=SimpleNamespace(disengagePredictions=SimpleNamespace(brakeDisengageProbs=[0.]))),
        'extrinsicsCalibration': SimpleNamespace(rpyCalib=[0., 0., 0.]), 'driverStateV2': SimpleNamespace()}
  dm = DriverMonitoring.__new__(DriverMonitoring)
  dm.settings = SimpleNamespace(_ALERT_MIN_SPEED=5.)
  dm._set_pose_strictness = Mock()
  dm._update_states = Mock()
  dm._update_events = Mock()
  dm.run_step(sm)
  assert dm._update_states.call_args.kwargs['op_engaged'] is expected
  assert dm._update_events.call_args.kwargs['op_engaged'] is expected


def ready_selfdrived(monkeypatch):
  from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
  from openpilot.selfdrive.selfdrived.state import StateMachine

  class SubMaster(dict):
    def all_checks(self):
      return all(self.valid.values())

  monkeypatch.setattr('openpilot.selfdrive.selfdrived.selfdrived.time.monotonic', lambda: 100.)
  monkeypatch.setattr('openpilot.selfdrive.selfdrived.selfdrived.REPLAY', False)
  CP, pandas = safety_messages()
  pandas[0].ignitionLine = True
  pandas[0].lateralAllowed = True
  sm = SubMaster(pandaStates=pandas, deviceState=log.DeviceState.new_message(started=True),
                 modelV2=log.ModelDataV2.new_message(), driverMonitoringState=log.DriverMonitoringState.new_message(),
                 extrinsicsCalibration=log.ExtrinsicsCalibration.new_message(calStatus='calibrated'),
                 deviceMotion=SimpleNamespace(), vehicleParameters=SimpleNamespace())
  sm.valid = dict.fromkeys(sm, True)
  sm.logMonoTime = {service: int(100e9) for service in sm}
  sm.recv_time = dict.fromkeys(sm, 100.0)
  sd = SelfdriveD.__new__(SelfdriveD)
  sd.CP, sd.sm = CP, sm
  sd.initialized, sd.enabled, sd.active = True, False, False
  sd.events = Events()
  sd.state_machine = StateMachine()
  sd.always_on_lateral = AlwaysOnLateralState()
  sd._retained_aol_fault_warnings = set()
  sd.lateral_panda_health = LateralPandaHealth()
  sd.always_on_lateral_enabled = True
  sd.car_state_mono_time, sd.car_state_received, sd.car_state_valid = 100., True, True
  CS = car.CarState.new_message(canValid=True, gearShifter='drive', vEgo=20.)
  CS.cruiseState.available = True
  return sd, CS


def test_selfdrived_steering_only_does_not_engage_cruise(monkeypatch):
  sd, CS = ready_selfdrived(monkeypatch)
  sd.update_always_on_lateral(CS)
  assert sd.always_on_lateral.active and sd.always_on_lateral.requested
  assert not sd.enabled and not sd.active


@pytest.mark.parametrize('service', ['carState', 'modelV2', 'driverMonitoringState', 'pandaStates', 'extrinsicsCalibration',
                                    'deviceMotion', 'vehicleParameters', 'deviceState'])
def test_selfdrived_stale_inputs_stop_and_latch(monkeypatch, service):
  sd, CS = ready_selfdrived(monkeypatch)
  sd.update_always_on_lateral(CS)
  if service == 'carState':
    sd.car_state_mono_time = 98.
  else:
    sd.sm.logMonoTime[service] = int(98e9)
  sd.update_always_on_lateral(CS)
  assert not sd.always_on_lateral.requested and sd.always_on_lateral.suspended
  sd.car_state_mono_time = 100.
  sd.sm.logMonoTime = {name: int(100e9) for name in sd.sm}
  sd.update_always_on_lateral(CS)
  assert not sd.always_on_lateral.active


@pytest.mark.parametrize('pause', ['brake', 'regen', 'parking', 'reverse', 'park', 'neutral'])
def test_selfdrived_brakes_and_gears_pause_then_resume(monkeypatch, pause):
  sd, CS = ready_selfdrived(monkeypatch)
  sd.update_always_on_lateral(CS)
  if pause in ('brake', 'regen', 'parking'):
    setattr(CS, {'brake': 'brakePressed', 'regen': 'regenBraking', 'parking': 'parkingBrake'}[pause], True)
    sd.events.add(EventName.parkBrake if pause == 'parking' else EventName.pedalPressed)
  else:
    CS.gearShifter = pause
    sd.events.add(EventName.reverseGear if pause == 'reverse' else EventName.wrongGear)
  sd.update_always_on_lateral(CS)
  assert sd.always_on_lateral.paused and not sd.always_on_lateral.requested and not sd.always_on_lateral.suspended
  CS.gearShifter = 'drive'
  CS.brakePressed = CS.regenBraking = CS.parkingBrake = False
  sd.events.clear()
  sd.update_always_on_lateral(CS)
  assert sd.always_on_lateral.active


def test_selfdrived_stationary_park_door_pause_resumes_without_cruise(monkeypatch):
  sd, CS = ready_selfdrived(monkeypatch)
  sd.update_always_on_lateral(CS)
  CS.gearShifter, CS.standstill, CS.vEgo, CS.doorOpen = 'park', True, 0., True
  sd.events.add(EventName.wrongGear)
  sd.events.add(EventName.doorOpen)
  sd.update_always_on_lateral(CS)
  assert sd.always_on_lateral.paused and not sd.always_on_lateral.requested and not sd.always_on_lateral.suspended
  CS.doorOpen = False
  sd.events.clear()
  sd.events.add(EventName.wrongGear)
  sd.update_always_on_lateral(CS)
  assert sd.always_on_lateral.paused and not sd.always_on_lateral.suspended
  CS.gearShifter, CS.standstill, CS.vEgo = 'drive', False, 20.
  sd.events.clear()
  sd.update_always_on_lateral(CS)
  assert sd.always_on_lateral.active and sd.always_on_lateral.requested
  assert not sd.enabled and not CS.buttonEnable


@pytest.mark.parametrize('condition', ['drive', 'drive_brake', 'drive_parking_brake', 'reverse', 'neutral',
                                     'moving', 'stale', 'invalid', 'can_invalid', 'can_timeout'])
def test_selfdrived_park_door_exception_requires_verified_stationary_park(monkeypatch, condition):
  sd, CS = ready_selfdrived(monkeypatch)
  sd.update_always_on_lateral(CS)
  CS.gearShifter, CS.standstill, CS.vEgo, CS.doorOpen = 'park', True, 0., True
  if condition in ('drive', 'reverse', 'neutral'):
    CS.gearShifter = condition
  elif condition in ('drive_brake', 'drive_parking_brake'):
    CS.gearShifter = 'drive'
    CS.brakePressed, CS.parkingBrake = condition == 'drive_brake', condition == 'drive_parking_brake'
  elif condition == 'moving':
    CS.standstill, CS.vEgo = False, 2.
  elif condition == 'stale':
    sd.car_state_mono_time = 98.
  elif condition == 'invalid':
    sd.car_state_valid = False
  elif condition == 'can_invalid':
    CS.canValid = False
  elif condition == 'can_timeout':
    CS.canTimeout = True
  sd.events.add(EventName.doorOpen)
  sd.update_always_on_lateral(CS)
  assert sd.always_on_lateral.suspended and not sd.always_on_lateral.requested


@pytest.mark.parametrize('fault', ['comm_issue', 'seatbelt', 'steering', 'firmware'])
def test_selfdrived_park_door_does_not_exempt_simultaneous_fault(monkeypatch, fault):
  sd, CS = ready_selfdrived(monkeypatch)
  sd.update_always_on_lateral(CS)
  sd.always_on_lateral._rearm_until = 0.  # evaluate an established AOL session
  CS.gearShifter, CS.standstill, CS.vEgo, CS.doorOpen = 'park', True, 0., True
  sd.events.add(EventName.wrongGear)
  sd.events.add(EventName.doorOpen)
  if fault == 'firmware':
    sd.sm['pandaStates'][0].lateralSuspended = True
  elif fault == 'steering':
    CS.steerFaultTemporary = True
    sd.events.add(EventName.steerTempUnavailableSilent)
  else:
    sd.events.add(EventName.commIssue if fault == 'comm_issue' else EventName.seatbeltNotLatched)
  sd.update_always_on_lateral(CS)
  assert sd.always_on_lateral.suspended and not sd.always_on_lateral.requested
  CS.gearShifter, CS.standstill, CS.vEgo, CS.doorOpen, CS.steerFaultTemporary = 'drive', False, 20., False, False
  sd.sm['pandaStates'][0].lateralSuspended = False
  sd.events.clear()
  sd.update_always_on_lateral(CS)
  assert sd.always_on_lateral.suspended and not sd.always_on_lateral.requested


def test_selfdrived_open_door_cannot_resume_when_leaving_park(monkeypatch):
  sd, CS = ready_selfdrived(monkeypatch)
  sd.update_always_on_lateral(CS)
  CS.gearShifter, CS.standstill, CS.vEgo, CS.doorOpen = 'park', True, 0., True
  sd.events.add(EventName.doorOpen)
  sd.update_always_on_lateral(CS)
  assert not sd.always_on_lateral.suspended
  CS.gearShifter, CS.standstill, CS.vEgo = 'drive', False, 20.
  sd.update_always_on_lateral(CS)
  assert sd.always_on_lateral.suspended and not sd.always_on_lateral.requested
  CS.doorOpen = False
  sd.events.clear()
  sd.update_always_on_lateral(CS)
  assert sd.always_on_lateral.suspended and not sd.always_on_lateral.active


def test_selfdrived_existing_fault_survives_park_door_cycle(monkeypatch):
  sd, CS = ready_selfdrived(monkeypatch)
  sd.update_always_on_lateral(CS)
  sd.events.add(EventName.commIssue)
  sd.update_always_on_lateral(CS)
  assert sd.always_on_lateral.suspended
  CS.gearShifter, CS.standstill, CS.vEgo, CS.doorOpen = 'park', True, 0., True
  sd.events.clear()
  sd.events.add(EventName.doorOpen)
  sd.update_always_on_lateral(CS)
  assert sd.always_on_lateral.suspended
  CS.gearShifter, CS.standstill, CS.vEgo, CS.doorOpen = 'drive', False, 20., False
  sd.events.clear()
  sd.update_always_on_lateral(CS)
  assert sd.always_on_lateral.suspended and not sd.always_on_lateral.requested


@pytest.mark.parametrize('fault', ['driver_lockout', 'driver_alert', 'calibration', 'can', 'steering', 'heartbeat', 'relay'])
def test_selfdrived_faults_never_activate_lateral_only(monkeypatch, fault):
  sd, CS = ready_selfdrived(monkeypatch)
  if fault == 'driver_lockout':
    sd.sm['driverMonitoringState'].lockout = True
  elif fault == 'driver_alert':
    sd.sm['driverMonitoringState'].alertLevel = 'three'
  elif fault == 'calibration':
    sd.sm['extrinsicsCalibration'].calStatus = 'uncalibrated'
  elif fault == 'can':
    CS.canValid = False
  elif fault == 'steering':
    CS.steerFaultPermanent = True
  elif fault == 'heartbeat':
    sd.sm['pandaStates'][0].heartbeatLost = True
  else:
    sd.sm['pandaStates'][0].faults = ['relayMalfunction']
  sd.update_always_on_lateral(CS)
  assert not sd.always_on_lateral.requested and not sd.always_on_lateral.active


@pytest.mark.parametrize('condition,steering', [('ready', True), ('brake', False), ('reverse', False),
                                              ('stale_state', False), ('stale_car', False), ('not_acknowledged', False),
                                              ('below_aol_speed', False), ('at_aol_speed', True), ('cruise_below_aol_speed', True)])
def test_controlsd_aol_activation_gates_preserve_cruise_separation(monkeypatch, condition, steering):
  from openpilot.selfdrive.controls.controlsd import Controls

  monkeypatch.setattr('openpilot.selfdrive.controls.controlsd.time.monotonic', lambda: 100.)
  sd, CS = ready_selfdrived(monkeypatch)
  CS.vEgo, CS.vCruise = 20., 100.
  CP = sd.CP
  CP.lateralTuning.init('pid')
  ss = log.SelfdriveState.new_message(alwaysOnLateralSupported=True, alwaysOnLateralRequested=True,
                                     alwaysOnLateralActive=True)
  sd.sm.update(carState=CS, selfdriveState=ss, longitudinalPlan=log.LongitudinalPlan.new_message(), onroadEvents=[],
               lateralManeuverPlan=SimpleNamespace(desiredCurvature=0.), lateralDelay=SimpleNamespace(lateralDelay=.2),
               vehicleParameters=SimpleNamespace(stiffnessFactor=1., steerRatio=15., angleOffsetDeg=0., roll=0.))
  sd.sm.valid = dict.fromkeys(sd.sm, True)
  sd.sm.logMonoTime = {service: int(100e9) for service in sd.sm}
  sd.sm.recv_time = dict.fromkeys(sd.sm, 100.0)
  if condition == 'brake':
    CS.brakePressed = True
  elif condition == 'reverse':
    CS.gearShifter = 'reverse'
  elif condition == 'stale_state':
    sd.sm.logMonoTime['selfdriveState'] = int(99e9)
  elif condition == 'stale_car':
    sd.sm.logMonoTime['carState'] = int(99e9)
  elif condition == 'not_acknowledged':
    ss.alwaysOnLateralActive = False
  elif condition in ('below_aol_speed', 'at_aol_speed', 'cruise_below_aol_speed'):
    CS.vEgo = (10. if condition == 'at_aol_speed' else 9.99) * CV.KPH_TO_MS
    ss.enabled = ss.active = condition == 'cruise_below_aol_speed'
  controls = Controls.__new__(Controls)
  controls.CP, controls.sm = CP, sd.sm
  controls.CI = SimpleNamespace(get_pid_accel_limits=Mock(return_value=(-3., 2.)))
  controls.VM = SimpleNamespace(update_params=Mock(), calc_curvature=Mock(return_value=0.))
  controls.LoC = SimpleNamespace(long_control_state=0, reset=Mock(), update=Mock(return_value=0.))
  controls.LaC = SimpleNamespace(reset=Mock(), update=Mock(return_value=(0., 0., None)))
  controls.desired_curvature, controls.steer_limited_by_safety = 0., False
  cc, _ = controls.state_control()
  assert cc.latActive is steering
  cruise = condition == 'cruise_below_aol_speed'
  assert cc.enabled is cruise and cc.longActive is cruise
  assert controls.LoC.update.call_args.args[0] is cruise


def test_selfdrived_speed_floor_is_temporary_and_recovers_without_rearming(monkeypatch):
  sd, CS = ready_selfdrived(monkeypatch)
  for speed in (9.99, 10., 20., 9.99, 10.):
    CS.vEgo = speed * CV.KPH_TO_MS
    sd.update_always_on_lateral(CS)
    assert sd.always_on_lateral.requested is (speed >= 10.)
    assert sd.always_on_lateral.active is (speed >= 10.)
    assert not sd.always_on_lateral.suspended and not sd.always_on_lateral.paused
  assert sd.CP.minSteerSpeed == -1


def test_low_speed_does_not_clear_fault_suspension():
  state = step(AlwaysOnLateralState())
  step(state, speed_available=False)
  assert not state.requested and not state.suspended
  step(state, speed_available=False, ready=False)
  assert state.suspended
  step(state, speed_available=True, now=20.)
  assert not state.requested and not state.active and state.suspended
  step(state, speed_available=True, rearm=True, now=21.)
  assert state.active


def alert_selfdrived(monkeypatch, *, cruise=False, aol=True):
  from openpilot.selfdrive.selfdrived.alertmanager import AlertManager

  sd, CS = ready_selfdrived(monkeypatch)
  sd.AM = AlertManager()
  sd.sm.frame = 0
  sd.personality = log.LongitudinalPersonality.standard
  sd.is_metric = True
  sd.always_on_lateral_enabled = aol
  sd.enabled = cruise
  sd.state_machine.state = log.SelfdriveState.OpenpilotState.enabled if cruise else log.SelfdriveState.OpenpilotState.disabled
  sd.data_sample = Mock(return_value=CS)
  sd.publish_selfdriveState = Mock()
  return sd, CS


def alert_step(sd, CS, *events):
  def update_events(_):
    sd.events.clear()
    for event in events:
      sd.events.add(event)

  sd.update_events = Mock(side_effect=update_events)
  sd.sm.frame += 1
  sd.step()  # real cruise transition, AOL transition, alert creation and selection
  return sd.AM.current_alert


def driver_stop(CS, action):
  if action.startswith('cancel'):
    CS.buttonEvents = [{'type': 'cancel', 'pressed': action == 'cancel'}]
    return EventName.buttonCancel
  setattr(CS, {'brake': 'brakePressed', 'regen': 'regenBraking', 'parking': 'parkingBrake'}[action], True)
  return EventName.parkBrake if action == 'parking' else EventName.pedalPressed


@pytest.mark.parametrize('action', ['cancel', 'cancel_release', 'brake', 'regen', 'parking'])
def test_aol_only_cancel_keeps_steering_and_brake_pauses_are_quiet(monkeypatch, action):
  sd, CS = alert_selfdrived(monkeypatch)
  alert_step(sd, CS)
  assert sd.always_on_lateral.active and not sd.enabled
  event = driver_stop(CS, action)
  for _ in range(25):
    alert = alert_step(sd, CS, event)
    assert alert.audible_alert == log.SelfdriveState.AudibleAlert.none
    assert sd.always_on_lateral.active is action.startswith('cancel')
    assert not sd.always_on_lateral.suspended
  assert sd.always_on_lateral_enabled
  assert f'{EVENT_NAME[event]}/{ET.USER_DISABLE}' not in sd.AM.alerts
  assert EVENTS[event][ET.USER_DISABLE].audible_alert == log.SelfdriveState.AudibleAlert.disengage

  CS.buttonEvents = []
  CS.brakePressed = CS.regenBraking = CS.parkingBrake = False
  assert alert_step(sd, CS).audible_alert == log.SelfdriveState.AudibleAlert.none
  assert sd.always_on_lateral.active


@pytest.mark.parametrize('action', ['cancel', 'brake', 'regen', 'parking'])
@pytest.mark.parametrize('aol', [False, True])
def test_normal_cruise_driver_disengagement_still_chimes(monkeypatch, action, aol):
  sd, CS = alert_selfdrived(monkeypatch, cruise=True, aol=aol)
  alert_step(sd, CS)
  event = driver_stop(CS, action)
  alert = alert_step(sd, CS, event)
  assert not sd.enabled
  assert sd.always_on_lateral.active is (aol and action == 'cancel')
  assert not sd.always_on_lateral.suspended
  assert alert.alert_type == f'{EVENT_NAME[event]}/{ET.USER_DISABLE}'
  assert alert.audible_alert == log.SelfdriveState.AudibleAlert.disengage


@pytest.mark.parametrize('condition', ['active', 'off', 'brake', 'regen', 'parking', 'reverse', 'low_speed', 'fault'])
def test_cancel_press_release_and_repeats_preserve_aol_state(monkeypatch, condition):
  sd, CS = alert_selfdrived(monkeypatch, aol=condition != 'off')
  alert_step(sd, CS)
  if condition in ('brake', 'regen', 'parking'):
    driver_stop(CS, condition)
  elif condition == 'reverse':
    CS.gearShifter = 'reverse'
  elif condition == 'low_speed':
    CS.vEgo = 9.99 * CV.KPH_TO_MS
  elif condition == 'fault':
    CS.steerFaultTemporary = True
  alert_step(sd, CS)
  alert_step(sd, CS)  # settle the one-frame just_stopped transition before Cancel
  before = vars(sd.always_on_lateral).copy()
  for action in ('cancel', 'cancel_release') * 3:
    event = driver_stop(CS, action)
    assert alert_step(sd, CS, event).audible_alert == log.SelfdriveState.AudibleAlert.none
    assert vars(sd.always_on_lateral) == before
    assert sd.always_on_lateral_enabled is (condition != 'off')
    assert not sd.enabled

  # Cancel neither latches a temporary pause nor rearms a genuine fault.
  CS.buttonEvents = []
  CS.brakePressed = CS.regenBraking = CS.parkingBrake = CS.steerFaultTemporary = False
  CS.gearShifter, CS.vEgo = 'drive', 20.
  alert_step(sd, CS)
  assert sd.always_on_lateral.active is (condition not in ('off', 'fault'))
  assert sd.always_on_lateral.suspended is (condition == 'fault')


@pytest.mark.parametrize('fault_cleared', [False, True])
def test_aol_off_on_toggle_rearms_only_when_ready(monkeypatch, fault_cleared):
  sd, CS = alert_selfdrived(monkeypatch)
  alert_step(sd, CS)
  CS.steerFaultTemporary = True
  alert_step(sd, CS)
  assert sd.always_on_lateral.suspended
  sd.always_on_lateral_enabled = False
  alert_step(sd, CS)
  assert not sd.always_on_lateral.active
  CS.steerFaultTemporary = not fault_cleared
  sd.always_on_lateral_enabled = True
  alert_step(sd, CS)
  assert sd.always_on_lateral.active is fault_cleared
  assert sd.always_on_lateral.requested is fault_cleared
  assert not sd.enabled


def test_set_release_into_cancel_cannot_rearm_existing_aol_fault(monkeypatch):
  sd, CS = alert_selfdrived(monkeypatch)
  alert_step(sd, CS)
  CS.steerFaultTemporary = True
  alert_step(sd, CS)
  assert sd.always_on_lateral.suspended
  CS.steerFaultTemporary = False
  CS.buttonEnable = True
  CS.buttonEvents = [{'type': 'decelCruise', 'pressed': False}, {'type': 'cancel', 'pressed': True}]
  alert_step(sd, CS, EventName.buttonEnable, EventName.buttonCancel)
  assert sd.always_on_lateral.suspended and not sd.always_on_lateral.requested
  assert not sd.enabled
  # A subsequent deliberate SET/RESUME still rearms once the fault is clear.
  CS.buttonEvents = [{'type': 'decelCruise', 'pressed': False}]
  alert_step(sd, CS, EventName.buttonEnable)
  assert sd.always_on_lateral.active and not sd.always_on_lateral.suspended


@pytest.mark.parametrize('action', ['cancel', 'brake'])
@pytest.mark.parametrize('fault', [EventName.commIssue, EventName.steerUnavailable, EventName.wrongCarMode,
                                  EventName.driverDistracted3, EventName.steerTempUnavailableSilent, EventName.speedTooHigh])
def test_aol_driver_stop_never_mutes_a_simultaneous_fault(monkeypatch, action, fault):
  sd, CS = alert_selfdrived(monkeypatch)
  alert_step(sd, CS)
  event = driver_stop(CS, action)
  if fault == EventName.driverDistracted3:
    sd.sm['driverMonitoringState'].alertLevel = 'three'
  elif fault == EventName.steerTempUnavailableSilent:
    CS.steerFaultTemporary = True
  alert = alert_step(sd, CS, event, fault)
  assert not sd.always_on_lateral.active and sd.always_on_lateral.suspended
  assert alert.alert_type.startswith(f'{EVENT_NAME[fault]}/')
  assert alert.audible_alert != log.SelfdriveState.AudibleAlert.none
  assert alert_step(sd, CS, event).alert_type == alert.alert_type
  assert sd.AM.current_alert.audible_alert == alert.audible_alert


def test_aol_brake_pause_preserves_existing_normal_cruise_chime_duration(monkeypatch):
  sd, CS = alert_selfdrived(monkeypatch, cruise=True)
  alert_step(sd, CS)
  event = driver_stop(CS, 'brake')
  assert alert_step(sd, CS, event).audible_alert == log.SelfdriveState.AudibleAlert.disengage
  entry = sd.AM.alerts[f'pedalPressed/{ET.USER_DISABLE}']
  original_start, original_end = entry.start_frame, entry.end_frame

  CS.brakePressed = False
  alert_step(sd, CS)
  assert sd.always_on_lateral.active and not sd.enabled
  CS.brakePressed = True
  assert alert_step(sd, CS, event).audible_alert == log.SelfdriveState.AudibleAlert.disengage
  # This sound belongs to the earlier cruise disengagement. The AOL-only
  # brake adds no new alert and neither truncates nor extends the old one.
  assert (entry.start_frame, entry.end_frame) == (original_start, original_end)
  while sd.sm.frame <= original_end:
    alert_step(sd, CS, event)
  assert sd.AM.current_alert.audible_alert == log.SelfdriveState.AudibleAlert.none


def test_other_aol_user_disable_alerts_remain_audible(monkeypatch):
  sd, CS = alert_selfdrived(monkeypatch)
  alert_step(sd, CS)
  CS.steeringDisengage = True
  alert = alert_step(sd, CS, EventName.steerDisengage)
  assert sd.always_on_lateral.suspended and not sd.always_on_lateral.active
  assert alert.alert_type == f'steerDisengage/{ET.USER_DISABLE}'
  assert alert.audible_alert == log.SelfdriveState.AudibleAlert.disengage


@pytest.mark.parametrize('fault', [EventName.steerTempUnavailableSilent, EventName.speedTooHigh])
@pytest.mark.parametrize('action', ['cancel', 'brake'])
@pytest.mark.parametrize('persistent', [False, True])
def test_aol_warning_fault_retains_its_original_duration_without_renewal(monkeypatch, fault, action, persistent):
  sd, CS = alert_selfdrived(monkeypatch)
  alert_step(sd, CS, EventName.laneChangeBlocked)
  sd.sm.frame = 10
  event = driver_stop(CS, action)
  CS.steerFaultTemporary = fault == EventName.steerTempUnavailableSilent
  first = alert_step(sd, CS, event, fault)
  key = f'{EVENT_NAME[fault]}/{ET.WARNING}'
  assert first.alert_type == key and first.audible_alert != log.SelfdriveState.AudibleAlert.none
  entry = sd.AM.alerts[key]
  original_end = entry.end_frame
  assert original_end == sd.sm.frame + first.duration
  assert not sd.AM.alerts[f'laneChangeBlocked/{ET.WARNING}'].active(sd.sm.frame)

  # Even a fault that disappears on the next 10 ms frame must reach 20 Hz
  # soundd. A persistent fault cannot keep extending the original warning.
  while sd.sm.frame <= original_end:
    CS.steerFaultTemporary = persistent and fault == EventName.steerTempUnavailableSilent
    alert = alert_step(sd, CS, event, *((fault,) if persistent else ()))
    assert entry.end_frame == original_end
    if sd.sm.frame <= original_end:
      assert alert.alert_type == key and alert.audible_alert == first.audible_alert
    else:
      assert alert.audible_alert == log.SelfdriveState.AudibleAlert.none
  assert not sd._retained_aol_fault_warnings
  assert alert_step(sd, CS, event, fault).audible_alert == log.SelfdriveState.AudibleAlert.none


@pytest.mark.parametrize('release', ['off', 'rearm', 'offroad'])
def test_explicit_aol_release_clears_only_its_retained_fault_warning(monkeypatch, release):
  sd, CS = alert_selfdrived(monkeypatch)
  alert_step(sd, CS)
  CS.steerFaultTemporary = True
  alert_step(sd, CS, EventName.steerTempUnavailableSilent)
  key = f'steerTempUnavailableSilent/{ET.WARNING}'
  assert sd.AM.alerts[key].active(sd.sm.frame)
  CS.steerFaultTemporary = False
  events = ()
  if release == 'off':
    sd.always_on_lateral_enabled = False
  elif release == 'offroad':
    sd.sm['deviceState'].started = False
  else:
    CS.buttonEnable = True
    events = (EventName.buttonEnable,)
  alert_step(sd, CS, *events)
  assert not sd._retained_aol_fault_warnings
  assert not sd.AM.alerts[key].active(sd.sm.frame)
  assert sd.AM.current_alert.alert_type != key


@pytest.mark.parametrize('event', [EventName.steerSaturated, EventName.laneChangeBlocked])
def test_aol_cancel_keeps_normal_nonblocking_warning_events(monkeypatch, event):
  sd, CS = alert_selfdrived(monkeypatch)
  alert_step(sd, CS)
  cancel = driver_stop(CS, 'cancel')
  for _ in range(6):
    alert = alert_step(sd, CS, cancel, event)
    assert sd.always_on_lateral.active
    assert alert.alert_type == f'{EVENT_NAME[event]}/{ET.WARNING}'
    assert alert.audible_alert == EVENTS[event][ET.WARNING].audible_alert
    assert not sd._retained_aol_fault_warnings
