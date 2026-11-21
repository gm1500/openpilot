from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from openpilot.cereal import log
from opendbc.car.structs import car
from opendbc.car.gm.values import GMSafetyFlags
from openpilot.selfdrive.selfdrived.always_on_lateral import (
  AlwaysOnLateralState, lateral_events_clear, lateral_safety_supported,
)
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.selfdrive.monitoring.policy import DriverMonitoring

EventName = log.OnroadEvent.EventName


def step(state, **kwargs):
  inputs = {'enabled': True, 'onroad': True, 'supported': True, 'ready': True, 'firmware_allowed': True,
            'firmware_suspended': False, 'cancel': False, 'rearm': False, 'now': 10.}
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


def test_cancel_latches_across_readiness_changes_until_explicit_rearm():
  state = step(AlwaysOnLateralState())
  step(state, cancel=True)
  assert state.suspended and not state.active and not state.requested and state.just_stopped
  step(state, ready=False)
  step(state, now=20.)
  assert state.suspended and not state.active
  step(state, rearm=True, now=20., firmware_suspended=True, firmware_allowed=False)
  assert state.requested and not state.active
  step(state, now=20.1)
  assert state.active and not state.suspended


def test_cancel_wins_over_simultaneous_rearm():
  state = step(AlwaysOnLateralState(), rearm=True, cancel=True)
  assert state.suspended and not state.requested


def test_brake_or_non_drive_pauses_without_cancelling_saved_intent():
  state = step(AlwaysOnLateralState())
  step(state, paused=True)
  assert state.paused and not state.active and not state.requested and not state.suspended
  step(state, paused=True, now=20.)
  assert not state.suspended
  step(state, now=21.)
  assert state.active


def test_cancel_during_brake_pause_does_not_resume_on_release():
  state = step(AlwaysOnLateralState())
  step(state, paused=True, cancel=True)
  step(state, paused=False, now=20.)
  assert state.suspended and not state.active


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


def test_firmware_cancel_or_fault_requires_driver_action():
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


@pytest.mark.parametrize('event', [EventName.pedalPressed, EventName.preEnableStandstill, EventName.pcmDisable,
                                   EventName.resumeBlocked, EventName.belowEngageSpeed])
def test_longitudinal_disengagement_does_not_block_lateral(event):
  events = Events()
  events.add(event)
  assert lateral_events_clear(events)


@pytest.mark.parametrize('event', [EventName.buttonCancel, EventName.wrongGear, EventName.reverseGear,
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
  sd.always_on_lateral_enabled = True
  sd.car_state_mono_time, sd.car_state_received, sd.car_state_valid = 100., True, True
  CS = car.CarState.new_message(canValid=True, gearShifter='drive')
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
                                              ('stale_state', False), ('stale_car', False), ('not_acknowledged', False)])
def test_controlsd_lateral_only_never_activates_longitudinal(monkeypatch, condition, steering):
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
  controls = Controls.__new__(Controls)
  controls.CP, controls.sm = CP, sd.sm
  controls.CI = SimpleNamespace(get_pid_accel_limits=Mock(return_value=(-3., 2.)))
  controls.VM = SimpleNamespace(update_params=Mock(), calc_curvature=Mock(return_value=0.))
  controls.LoC = SimpleNamespace(long_control_state=0, reset=Mock(), update=Mock(return_value=0.))
  controls.LaC = SimpleNamespace(reset=Mock(), update=Mock(return_value=(0., 0., None)))
  controls.desired_curvature, controls.steer_limited_by_safety = 0., False
  cc, _ = controls.state_control()
  assert cc.latActive is steering
  assert not cc.enabled and not cc.longActive
  assert controls.LoC.update.call_args.args[0] is False
