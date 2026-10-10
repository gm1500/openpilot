import unittest
from types import SimpleNamespace as NS

from openpilot.cereal import log
from opendbc.car.structs import car
from openpilot.selfdrive.controls.lib.resume_test import ResumeTest, SERVICES, TEST_ACCEL


class SM(dict):
  def __init__(self):
    super().__init__()
    for name in (*SERVICES, 'resumeTestRequest'):
      self[name] = log.Event.new_message().init(name)
    self.valid = dict.fromkeys(self, True)
    self.seen = dict.fromkeys(self, True)
    self.logMonoTime = dict.fromkeys(self, 0)
    self.recv_time = dict.fromkeys(self, 0.)

  def tick(self, now):
    for name in self:
      self.logMonoTime[name] = int(now * 1e9)
      self.recv_time[name] = now


class TestResumeTest(unittest.TestCase):
  def setUp(self):
    cp = NS(carFingerprint='CHEVROLET_SILVERADO', openpilotLongitudinalControl=True, networkLocation='fwdCamera')
    self.test = ResumeTest(cp)
    self.sm = SM()
    cs = self.sm['carState']
    cs.canValid = cs.standstill = True
    cs.gearShifter = 'drive'
    cs.vCruise = 30.
    self.sm['carOutput'].actuatorsOutput.brake = 50.
    self.sm['selfdriveState'].active = True
    self.sm['selfdriveState'].state = 'enabled'
    self.sm['longitudinalPlan'].aTarget = -.37
    self.sm['longitudinalPlan'].shouldStop = True
    self.sm['modelV2'].action.shouldStop = True
    self.now = 100.

  def step(self, dt=.01, stopping=True):
    self.now += dt
    self.sm.tick(self.now)
    return self.test.update(self.sm, self.now, True, stopping)

  def press(self):
    self.sm['resumeTestRequest'].requestId = int((self.now + .01) * 1e9)
    self.sm['resumeTestRequest'].held = True
    return self.step()

  def test_no_touch_preserves_normal_planner(self):
    self.assertEqual(self.step(), (self.sm['longitudinalPlan'].aTarget, True))
    self.assertTrue(self.test.ready)
    self.sm['longitudinalPlan'].aTarget = .8
    self.sm['longitudinalPlan'].shouldStop = False
    self.assertFalse(self.step()[1])
    self.assertFalse(self.test.latched)

  def test_explicit_touch_can_override_existing_model_stop(self):
    self.assertEqual(self.press(), (TEST_ACCEL, False))
    self.assertTrue(self.test.active)

  def test_release_holds_despite_planner_flicker(self):
    self.press()
    self.sm['resumeTestRequest'].held = False
    self.sm['longitudinalPlan'].aTarget = 1.6
    self.sm['longitudinalPlan'].shouldStop = False
    self.assertEqual(self.step(), (0., True))
    self.assertFalse(self.test.active)
    self.assertTrue(self.test.latched)
    self.assertTrue(self.step(2.)[1])

  def test_timeout_never_repeats_while_finger_held(self):
    self.press()
    for _ in range(500):
      self.step(stopping=not self.test.active)
    self.assertFalse(self.test.active)
    self.assertTrue(self.test.latched)
    self.assertEqual(self.test.reason, 'timeout')
    self.assertEqual(self.press(), (TEST_ACCEL, False))

  def test_lost_or_replayed_heartbeat_holds(self):
    self.press()
    self.now += .25
    self.assertTrue(self.test.update(self.sm, self.now, True, False)[1])
    self.assertFalse(self.test.active)
    self.step(2.)
    self.assertFalse(self.test.active)

  def test_old_press_cannot_start_after_control_restart(self):
    self.sm['resumeTestRequest'].requestId = int((self.now - .5) * 1e9)
    self.sm['resumeTestRequest'].held = True
    self.step()
    self.assertFalse(self.test.active)

  def test_cancel_conditions_during_pulse(self):
    for service, field, value in (
      ('carState', 'vEgoRaw', .5), ('carState', 'vEgo', -.5),
      ('carState', 'stockAeb', True), ('carState', 'stockFcw', True),
      ('carState', 'gearShifter', 'reverse'), ('carState', 'doorOpen', True),
      ('carState', 'seatbeltUnlatched', True), ('carState', 'parkingBrake', True),
      ('carState', 'accFaulted', True), ('carState', 'canValid', False),
      ('carState', 'vEgo', float('nan')), ('longitudinalPlan', 'fcw', True),
      ('driverMonitoringState', 'noResponseForceDecel', True),
      ('selfdriveState', 'state', 'softDisabling'),
    ):
      with self.subTest(service=service, field=field):
        self.setUp()
        self.press()
        setattr(self.sm[service], field, value)
        self.assertTrue(self.step(stopping=False)[1])
        self.assertFalse(self.test.active)

  def test_close_or_closing_lead_blocks_and_rejected_press_not_queued(self):
    lead = self.sm['radarState'].leadOne
    lead.present = True
    lead.dRel = 5.
    self.press()
    self.assertFalse(self.test.active)
    lead.dRel = 20.
    self.step()
    self.assertFalse(self.test.active)
    self.press()
    self.assertTrue(self.test.active)
    lead.vRel = -8.
    self.step(stopping=False)
    self.assertFalse(self.test.active)

  def test_every_required_service_stale_blocks(self):
    for name in SERVICES:
      with self.subTest(name=name):
        self.setUp()
        self.sm.valid[name] = False
        self.press()
        self.assertFalse(self.test.active)

  def test_pedals_and_resume_exit_test(self):
    for action in ('gasPressed', 'brakePressed', 'resume', 'disable'):
      with self.subTest(action=action):
        self.setUp()
        self.press()
        if action == 'resume':
          self.sm['carState'].buttonEvents = [car.CarState.ButtonEvent(type='accelCruise', pressed=True)]
        elif action != 'disable':
          setattr(self.sm['carState'], action, True)
        self.test.update(self.sm, self.now, action != 'disable', False)
        self.assertFalse(self.test.active)
        self.assertFalse(self.test.latched)

  def test_non_sierra_or_stock_longitudinal_disabled(self):
    for cp in (NS(carFingerprint='OTHER', openpilotLongitudinalControl=True, networkLocation='fwdCamera'),
               NS(carFingerprint='CHEVROLET_SILVERADO', openpilotLongitudinalControl=False, networkLocation='fwdCamera')):
      self.test = ResumeTest(cp)
      self.press()
      self.assertFalse(self.test.active)

  def test_longcontrol_release_and_immediate_calibrated_hold(self):
    from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
    cp = NS(carFingerprint='CHEVROLET_SILVERADO', openpilotLongitudinalControl=True,
            longitudinalTuning=NS(kiBP=[0.], kiV=[.2]), longitudinalActuatorDelay=.5, stopAccel=-.37)
    control = LongControl(cp)
    control.long_control_state = LongCtrlState.stopping
    control.last_output_accel = cp.stopAccel
    cs = self.sm['carState']
    cs.cruiseState.standstill = True
    target, stop = self.press()
    accel = self.test.apply(control, True, cs, target, stop, (-3.5, 2.))
    self.assertEqual(control.long_control_state, LongCtrlState.pid)
    self.assertGreater(accel, 0.)
    self.assertLessEqual(accel, TEST_ACCEL)
    self.assertEqual(control.pid.i, 0.)
    self.sm['resumeTestRequest'].held = False
    target, stop = self.step(stopping=False)
    accel = self.test.apply(control, True, cs, target, stop, (-3.5, 2.))
    self.assertEqual(control.long_control_state, LongCtrlState.stopping)
    self.assertLessEqual(accel, cp.stopAccel)

  def test_gm_can_release_torque_and_rehold(self):
    from opendbc.car import Bus
    from opendbc.can.dbc import DBC as Database
    from opendbc.car.gm.carcontroller import CarController
    from opendbc.car.gm.values import DBC
    from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
    cp = car.CarParams.new_message(carFingerprint='CHEVROLET_SILVERADO', openpilotLongitudinalControl=True,
                                  networkLocation='fwdCamera', radarUnavailable=True, mass=2586., wheelRadius=.419,
                                  longitudinalActuatorDelay=.5, stopAccel=-.37)
    cp.longitudinalTuning.kiBP = [0.]
    cp.longitudinalTuning.kiV = [.2]
    ctrl = LongControl(cp)
    ctrl.long_control_state = LongCtrlState.stopping
    ctrl.last_output_accel = cp.stopAccel
    cs = self.sm['carState']
    cs.cruiseState.standstill = True
    cc = car.CarControl.new_message(enabled=True, longActive=True)
    gm = CarController(DBC[cp.carFingerprint], cp)
    db = Database(DBC[cp.carFingerprint][Bus.pt])
    status = dict.fromkeys(db.name_to_msg['PSCMStatus'].sigs, 0)
    vehicle = NS(out=cs, cam_lka_steering_cmd_counter=0, pt_lka_steering_cmd_counter=0,
                 loopback_lka_steering_cmd_ts_nanos=0, pscm_status=status)
    decoded = []
    for i in range(145):
      target, stop = self.press() if i == 5 else self.step(stopping=ctrl.long_control_state == LongCtrlState.stopping)
      cc.actuators.accel = self.test.apply(ctrl, True, cs, target, stop, (-3.5, 2.))
      cc.actuators.longControlState = ctrl.long_control_state
      _, messages = gm.update(cc.as_reader(), vehicle, int(self.now * 1e9))
      for addr, data, bus in messages:
        if addr not in (715, 789):
          continue
        values = {}
        for name, sig in db.addr_to_msg[addr].sigs.items():
          value = sig.get_raw_value(data)
          if sig.is_signed:
            value -= ((value >> (sig.size - 1)) & 1) * (1 << sig.size)
          values[name] = value * sig.factor + sig.offset
        decoded.append((self.test.active, addr, values))
        self.assertEqual(bus, 0)
        if addr == 715:
          self.assertEqual(values['GasRegenFullStopActive'], not self.test.active)
          if self.test.active:
            self.assertAlmostEqual(values['GasRegenCmd'], 325., delta=1.)
          else:
            self.assertEqual(values['GasRegenCmd'], -540.)
        else:
          self.assertEqual(values['FrictionBrakeMode'], 1 if self.test.active else 13)
          self.assertEqual(values['FrictionBrakeCmd'], 0 if self.test.active else -49.)
    self.assertTrue(any(active for active, _, _ in decoded))
    self.assertFalse(self.test.active)


if __name__ == '__main__':
  unittest.main()
