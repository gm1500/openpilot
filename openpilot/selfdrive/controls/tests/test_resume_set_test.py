import unittest

from openpilot.cereal import log
from opendbc.car.structs import car
from openpilot.selfdrive.controls.lib.resume_set_test import ResumeSetTest, SERVICES
from opendbc.car.gm.resume_set_test import TEST_ACCEL, MAX_DURATION, MAX_SPEED


def params():
  cp = car.CarParams.new_message(carFingerprint='CHEVROLET_SILVERADO', openpilotLongitudinalControl=True,
                                networkLocation='fwdCamera', flags=2, radarUnavailable=True, mass=2586., wheelRadius=.419,
                                longitudinalActuatorDelay=.5, stopAccel=-.37)
  cp.safetyConfigs = [{'safetyModel': 'gm', 'safetyParam': 11}]
  cp.longitudinalTuning.kiBP = [0.]
  cp.longitudinalTuning.kiV = [.2]
  return cp


class SM(dict):
  def __init__(self):
    super().__init__()
    for name in (*SERVICES, 'resumeSetTestRequest'):
      self[name] = log.Event.new_message().init(name, 1) if name == 'pandaStates' else log.Event.new_message().init(name)
    self.valid = dict.fromkeys(self, True)
    self.seen = dict.fromkeys(self, True)
    self.logMonoTime = dict.fromkeys(self, 0)
    self.recv_time = dict.fromkeys(self, 0.)

  def tick(self, now, except_service=None):
    for name in self:
      if name != except_service:
        self.logMonoTime[name] = int(now * 1e9)
        self.recv_time[name] = now


class TestResumeSetTest(unittest.TestCase):
  def setUp(self):
    self.cp = params()
    self.test = ResumeSetTest(self.cp)
    self.sm = SM()
    cs = self.sm['carState']
    cs.canValid = cs.standstill = True
    cs.gearShifter = 'drive'
    cs.vCruise = 30.
    cs.gmCruiseState = 4
    cs.cruiseState.standstill = cs.cruiseState.enabled = True
    self.sm['carOutput'].actuatorsOutput.brake = 49.
    self.sm['selfdriveState'].active = True
    self.sm['selfdriveState'].state = 'enabled'
    self.sm['longitudinalPlan'].aTarget = -.37
    self.sm['longitudinalPlan'].shouldStop = True
    self.sm['modelV2'].action.shouldStop = True
    panda = self.sm['pandaStates'][0]
    panda.safetyModel, panda.safetyParam = 'gm', 11
    panda.controlsAllowed = True
    self.now = 100.
    self.step()

  def step(self, dt=.01, stopping=True, feedback=True, except_service=None):
    self.now += dt
    if feedback:
      output = self.sm['carOutput'].actuatorsOutput
      output.gmResumeSetHolding, output.gmResumeSetActive = self.test.holding, self.test.active
    self.sm.tick(self.now, except_service)
    return self.test.update(self.sm, self.now, True, stopping)

  def press(self, button=2):
    req = self.sm['resumeSetTestRequest']
    req.requestId = int((self.now + .01) * 1e9)
    req.held, req.button = True, button
    return self.step()

  def test_no_touch_does_not_change_normal_planner(self):
    self.assertTrue(self.test.ready)
    self.sm['longitudinalPlan'].aTarget = 1.2
    self.sm['longitudinalPlan'].shouldStop = False
    self.assertEqual(self.step(), (self.sm['longitudinalPlan'].aTarget, False))
    self.assertFalse(self.test.holding)

  def test_both_buttons_roll_and_release_latches_hold_against_planner_departure(self):
    for button in (2, 3):
      self.setUp()
      self.assertEqual(self.press(button), (TEST_ACCEL, False))
      self.assertEqual(self.test.button, button)
      self.sm['carState'].gmCruiseState = 1
      self.step()
      self.assertTrue(self.test.active)
      self.assertTrue(self.test.recovered)
      self.sm['resumeSetTestRequest'].held = False
      self.sm['longitudinalPlan'].aTarget = 2.
      self.sm['longitudinalPlan'].shouldStop = False
      self.assertEqual(self.step(), (0., True))
      for _ in range(200):
        self.assertTrue(self.step()[1])
      self.assertFalse(self.test.active)
      self.assertTrue(self.test.holding)

  def test_timeout_requires_release_and_new_press(self):
    self.press()
    for _ in range(int(MAX_DURATION * 100) + 20):
      self.step()
    self.assertFalse(self.test.active)
    self.assertEqual(self.test.reason, 'timeout')
    self.step(2.)
    self.press(3)  # a new ID while the physical touch stayed down is insufficient
    self.assertFalse(self.test.active)
    self.sm['resumeSetTestRequest'].held = False
    self.step()
    self.assertEqual(self.press(3), (TEST_ACCEL, False))

  def test_switching_buttons_during_hold_aborts(self):
    self.press(2)
    self.sm['resumeSetTestRequest'].button = 3
    self.assertTrue(self.step()[1])
    self.assertFalse(self.test.active)

  def test_heartbeat_loss_and_control_stall_abort(self):
    for stalled in (False, True):
      self.setUp()
      self.press()
      if stalled:
        self.step(.21)
      else:
        for _ in range(22):
          self.step(except_service='resumeSetTestRequest')
      self.assertFalse(self.test.active)
      self.assertTrue(self.test.holding)

  def test_restart_never_adopts_a_held_request_as_launch(self):
    self.press()
    self.test = ResumeSetTest(self.cp)
    self.step(feedback=False)
    self.assertFalse(self.test.active)
    self.sm['carOutput'].actuatorsOutput.gmResumeSetHolding = True
    self.step(feedback=False)
    self.assertTrue(self.test.holding)
    self.assertFalse(self.test.active)

  def test_rejected_presses_not_queued(self):
    self.sm['carState'].doorOpen = True
    self.press()
    self.sm['carState'].doorOpen = False
    self.step()
    self.assertFalse(self.test.active)

  def test_bounds_faults_and_panda_rejection_end_attempt(self):
    cases = [('carState', 'vEgoRaw', MAX_SPEED), ('carState', 'vEgo', -.1), ('carState', 'vEgo', float('nan')),
             ('carState', 'stockAeb', True), ('carState', 'stockFcw', True), ('carState', 'doorOpen', True),
             ('carState', 'seatbeltUnlatched', True), ('carState', 'gearShifter', 'reverse'),
             ('carState', 'parkingBrake', True), ('carState', 'canValid', False), ('carState', 'canTimeout', True),
             ('carState', 'accFaulted', True), ('carState', 'gmCruiseState', 0), ('carState', 'gmCruiseState', 3),
             ('selfdriveState', 'state', 'softDisabling'), ('driverMonitoringState', 'noResponseForceDecel', True),
             ('longitudinalPlan', 'fcw', True)]
    for service, field, value in cases:
      with self.subTest(service=service, field=field):
        self.setUp()
        self.press()
        setattr(self.sm[service], field, value)
        self.assertTrue(self.step()[1])
        self.assertFalse(self.test.active)
    for field, value in (('controlsAllowed', False), ('safetyParam', 3), ('safetyRxChecksInvalid', True),
                         ('safetyTxBlocked', 1), ('faults', ['interruptRateCan2'])):
      with self.subTest(panda_field=field):
        self.setUp()
        self.press()
        setattr(self.sm['pandaStates'][0], field, value)
        self.assertTrue(self.step()[1])
        self.assertFalse(self.test.active)

  def test_distance_limit(self):
    self.press()
    self.sm['carState'].vEgo = self.sm['carState'].vEgoRaw = 1.5
    for _ in range(670):
      self.step(stopping=False)
    self.assertFalse(self.test.active)
    self.assertEqual(self.test.reason, 'distanceLimit')

  def test_lead_veto_and_every_service_stale(self):
    for name in SERVICES:
      self.setUp()
      self.sm.valid[name] = False
      self.press()
      self.assertFalse(self.test.active, name)
    for drel, vrel in ((5., 0.), (10., -3.)):
      self.setUp()
      self.press()
      lead = self.sm['radarState'].leadOne
      lead.present, lead.dRel, lead.vRel = True, drel, vrel
      self.assertTrue(self.step()[1])
      self.assertFalse(self.test.active)

  def test_pedals_and_disengagement_exit_test(self):
    for action in ('gasPressed', 'brakePressed', 'disable'):
      self.setUp()
      self.press()
      if action != 'disable':
        setattr(self.sm['carState'], action, True)
      self.test.update(self.sm, self.now, action != 'disable', False)
      self.assertFalse(self.test.active)
      self.assertFalse(self.test.holding)

  def test_flags_restrict_to_dedicated_sierra_configuration(self):
    for field, value in (('flags', 0), ('carFingerprint', 'OTHER'), ('networkLocation', 'gateway'),
                         ('openpilotLongitudinalControl', False)):
      cp = params()
      setattr(cp, field, value)
      self.assertFalse(ResumeSetTest(cp).supported)
    cp = params()
    cp.safetyConfigs[0].safetyParam = 3
    self.assertFalse(ResumeSetTest(cp).supported)

  def test_longcontrol_release_and_controlled_rehold(self):
    from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
    controller = LongControl(self.cp)
    controller.long_control_state = LongCtrlState.stopping
    controller.last_output_accel = self.cp.stopAccel
    target, stop = self.press()
    cs = self.sm['carState']
    self.assertAlmostEqual(self.test.apply(controller, True, cs, target, stop, (-3.5, 2.)), TEST_ACCEL)
    self.assertEqual(controller.long_control_state, LongCtrlState.pid)
    cs.standstill, cs.vEgo, cs.vEgoRaw = False, 1.5, 1.5
    self.sm['resumeSetTestRequest'].held = False
    target, stop = self.step(stopping=False)
    self.assertAlmostEqual(self.test.apply(controller, True, cs, target, stop, (-3.5, 2.)), -.8)
    cs.standstill, cs.vEgo, cs.vEgoRaw = True, 0., 0.
    target, stop = self.step()
    self.assertAlmostEqual(self.test.apply(controller, True, cs, target, stop, (-3.5, 2.)), self.cp.stopAccel)


if __name__ == '__main__':
  unittest.main()
