import time
from unittest.mock import patch

from opendbc.car.structs import car
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.car.cruise import VCruiseHelper
from openpilot.selfdrive.car.map_cruise import fallback_e2e_ready
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.controls.lib.longitudinal_mode import LongitudinalMode, automatic_e2e_selected, set_longitudinal_mode
from openpilot.selfdrive.modeld.constants import ModelConstants


class TestSLCPlanner(OpenpilotTestCase):
  """Real MPC, SubMaster health checks and serialized native planner feedback."""

  def setUp(self):
    super().setUp()
    self.now = time.monotonic()
    self.frame = 0
    self.limit = 50.
    self.junction = 0.
    self.slc_enabled = True
    self.params = Params()
    self.params.put_bool('ExperimentalModeConfirmed', True, block=True)
    self.params.put_bool('MapCruiseEnabled', True, block=True)
    set_longitudinal_mode(self.params, LongitudinalMode.conditional)
    self.model_valid = self.model_complete = True
    cp = car.CarParams(openpilotLongitudinalControl=True, longitudinalActuatorDelay=.5, steerRatio=16., wheelbase=3.6)
    self.cs = car.CarState(vEgo=40/3.6, gearShifter='drive', cruiseState={'available': True})
    self.cruise = VCruiseHelper(cp)
    self.cruise.initialize_v_cruise(self.cs, False)
    self.planner = LongitudinalPlanner(cp, init_v=self.cs.vEgo)
    services = ['carState', 'carControl', 'controlsState', 'selfdriveState', 'vehicleParameters',
                'radarState', 'modelV2', 'mapTrafficControl', 'mapCruiseState']
    optional = ['mapTrafficControl', 'mapCruiseState']
    # Inputs are injected together at model cadence; their real publisher rates
    # differ. Keep actual validity/aliveness checks, ignoring only average rate.
    self.sm = messaging.SubMaster(services, ignore_avg_freq=services, ignore_alive=optional, ignore_valid=optional)
    self.ack = messaging.SubMaster(['longitudinalPlan'], ignore_avg_freq=['longitudinalPlan'])
    self.pm = messaging.PubMaster(['longitudinalPlan'])
    self.ack.update(0)  # connect the native reader before the first publication

  def cruise_step(self):
    self.now += .01
    ready = fallback_e2e_ready(self.ack, self.now)
    old_set = self.cruise.v_cruise_kph
    self.cruise.update_v_cruise(self.cs, True, True, map_enabled=self.slc_enabled,
                              map_speed=None if self.limit is None else self.limit/3.6, map_gps_time=self.now,
                              now=self.now, automatic_e2e=self.conditional_selected(), e2e_ready=ready)
    if self.cruise.v_cruise_kph == 105 and old_set != 105:
      self.assertTrue(ready, 'SET rose before a fresh native planner acknowledgement')
    self.cs.vCruise = self.cruise.v_cruise_kph

  def conditional_selected(self):
    return automatic_e2e_selected(self.params, self.cruise.CP, self.params.get_bool('ExperimentalMode'))

  def plan_step(self, set_override=None):
    events = {s: messaging.new_message(s, valid=True, logMonoTime=int(self.now*1e9)) for s in self.sm.services}
    cs = events['carState'].carState
    cs.vEgo = self.cs.vEgo
    cs.vCruise = self.cs.vCruise if set_override is None else set_override
    cs.gearShifter = 'drive'
    events['carControl'].carControl.longActive = True
    events['controlsState'].controlsState.longControlState = 'pid'
    sd = events['selfdriveState'].selfdriveState
    sd.enabled, sd.conditionalExperimental, sd.personality = True, self.conditional_selected(), 'standard'
    sd.experimentalMode = self.params.get_bool('ExperimentalMode')
    model = events['modelV2'].modelV2
    times = ModelConstants.T_IDXS
    model.velocity.x = [self.cs.vEgo] * len(times) if self.model_complete else []
    model.velocity.y = model.position.y = [0.] * len(times)
    model.position.x = [self.cs.vEgo*t for t in times]
    model.action.desiredAcceleration = -.5
    model.meta.disengagePredictions.gasPressProbs = [1., 1.]
    events['modelV2'].valid = self.model_valid
    approach = events['mapTrafficControl'].mapTrafficControl
    approach.kind = 'junction' if self.junction else 'none'
    approach.nodeId = 10 if self.junction else 0
    approach.distance = self.junction
    approach.gpsMonoTime = approach.positionMonoTime = int(int(self.now)*1e9)
    selector = self.cruise.map_cruise
    status = events['mapCruiseState'].mapCruiseState
    status.state = selector.state(True)
    status.automaticE2e, status.e2eFallback = selector.automatic_e2e, selector.e2e_fallback
    status.setSpeed = self.cruise.v_cruise_kph/3.6
    self.sm.update_msgs(self.now, [e.as_reader() for e in events.values()])
    with patch('openpilot.selfdrive.controls.lib.longitudinal_planner.time.monotonic', return_value=self.now):
      self.planner.update(self.sm)
      self.planner.publish(self.sm, self.pm)
      self.ack.update(0)
    self.assertTrue(self.ack.updated['longitudinalPlan'])
    plan = self.ack['longitudinalPlan']
    self.assertLessEqual(plan.aTarget, plan.conditionalExperimental.regularAcceleration + 1e-6)
    return plan.conditionalExperimental

  def tick(self, frames):
    for _ in range(frames):
      self.cruise_step()
      self.frame += 1
      if self.frame % 5 == 0:
        self.plan_step()

  def outage(self):
    self.tick(220)
    self.assertEqual(self.cs.vCruise, 50)
    self.limit = None
    self.tick(95)
    self.assertFalse(self.cruise.map_cruise.e2e_fallback)
    self.assertEqual(self.cs.vCruise, 50)
    self.tick(20)
    self.assertTrue(self.ack['longitudinalPlan'].conditionalExperimental.e2eEnabled)
    self.assertEqual(self.ack['longitudinalPlan'].conditionalExperimental.reason, 'noSpeedLimit')
    self.assertEqual(self.cs.vCruise, 105)

  def recover(self):
    self.limit = 40.
    self.tick(190)
    self.assertEqual(self.cs.vCruise, 105)
    self.tick(20)
    self.assertEqual(self.cs.vCruise, 40)
    self.assertFalse(self.cruise.map_cruise.e2e_fallback)
    self.assertNotEqual(self.ack['longitudinalPlan'].conditionalExperimental.reason, 'noSpeedLimit')

  def test_no_speed_entry_and_recovery_without_junction(self):
    self.outage()
    self.recover()
    self.assertFalse(self.ack['longitudinalPlan'].conditionalExperimental.e2eEnabled)

  def test_recovery_retains_independent_junction_condition(self):
    self.junction = 40.
    self.outage()
    self.recover()
    self.assertTrue(self.ack['longitudinalPlan'].conditionalExperimental.e2eEnabled)
    self.assertEqual(self.ack['longitudinalPlan'].conditionalExperimental.reason, 'junction')

  def test_recovery_rechecks_junction_against_restored_set(self):
    self.junction = 180.
    self.outage()
    self.assertTrue(self.ack['longitudinalPlan'].conditionalExperimental.armed)
    self.recover()
    self.assertFalse(self.ack['longitudinalPlan'].conditionalExperimental.e2eEnabled)

  def test_recovery_status_before_corrected_carstate_retains_e2e(self):
    self.outage()
    self.limit = 40.
    for _ in range(210):
      self.cruise_step()
      if not self.cruise.map_cruise.e2e_fallback:
        break
      self.plan_step()
    self.assertEqual(self.cs.vCruise, 40)
    self.assertEqual(self.plan_step(set_override=105).reason, 'noSpeedLimit')
    self.assertFalse(self.plan_step().e2eEnabled)

  def test_invalid_plan_does_not_acknowledge_set_raise(self):
    self.tick(220)
    self.limit, self.model_valid = None, False
    self.tick(115)
    self.assertTrue(self.cruise.map_cruise.e2e_fallback)
    self.assertFalse(self.ack.valid['longitudinalPlan'])
    self.assertEqual(self.cs.vCruise, 50)

  def test_incomplete_model_does_not_acknowledge_set_raise(self):
    self.tick(220)
    self.limit, self.model_complete = None, False
    self.tick(115)
    self.assertTrue(self.ack.valid['longitudinalPlan'])
    self.assertFalse(self.ack['longitudinalPlan'].conditionalExperimental.e2eEnabled)
    self.assertEqual(self.cs.vCruise, 50)

  def test_slc_off_clears_both_conditions(self):
    self.junction = 40.
    self.outage()
    self.slc_enabled = False
    self.tick(5)
    self.assertFalse(self.ack['longitudinalPlan'].conditionalExperimental.e2eEnabled)
    self.assertEqual(self.ack['longitudinalPlan'].conditionalExperimental.state, 'off')

  def test_slc_alone_keeps_regular_mode_for_junction_and_missing_speed(self):
    set_longitudinal_mode(self.params, LongitudinalMode.voacc)
    self.junction = 40.
    self.tick(220)
    self.assertTrue(self.params.get_bool('MapCruiseEnabled'))
    self.assertEqual(self.cs.vCruise, 50)
    self.assertFalse(self.ack['longitudinalPlan'].conditionalExperimental.e2eEnabled)
    self.limit = None
    self.tick(150)
    self.assertFalse(self.cruise.map_cruise.e2e_fallback)
    self.assertFalse(self.ack['longitudinalPlan'].conditionalExperimental.e2eEnabled)
    self.assertEqual(self.cs.vCruise, 50)
    self.limit = 30.
    self.tick(220)
    self.assertEqual(self.cs.vCruise, 30)  # SLC continues in ordinary mode

  def test_conditional_toggle_controls_both_conditions_without_disabling_slc(self):
    set_longitudinal_mode(self.params, LongitudinalMode.voacc)
    self.junction = 40.
    self.tick(220)
    set_longitudinal_mode(self.params, LongitudinalMode.conditional)
    self.tick(120)
    self.assertTrue(self.ack['longitudinalPlan'].conditionalExperimental.e2eEnabled)
    self.assertEqual(self.ack['longitudinalPlan'].conditionalExperimental.reason, 'junction')
    self.junction, self.limit = 0., None
    self.tick(115)
    self.assertEqual(self.cs.vCruise, 105)
    self.assertEqual(self.ack['longitudinalPlan'].conditionalExperimental.reason, 'noSpeedLimit')
    set_longitudinal_mode(self.params, LongitudinalMode.voacc)
    self.tick(5)
    self.assertTrue(self.params.get_bool('MapCruiseEnabled'))
    self.assertFalse(self.cruise.map_cruise.e2e_fallback)
    self.assertFalse(self.ack['longitudinalPlan'].conditionalExperimental.e2eEnabled)
    self.limit = 40.
    self.tick(220)
    self.assertEqual(self.cs.vCruise, 40)
