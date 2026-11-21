"""SLC recovery exercises card publication and the real longitudinal planner."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from opendbc.car.structs import car
from openpilot.cereal import messaging
from openpilot.selfdrive.car.card import Car
from openpilot.selfdrive.car.cruise import ButtonType, VCruiseHelper
from openpilot.selfdrive.car.slc_settings import SLCSettings
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner, get_cruise_accel
from openpilot.selfdrive.controls.tests import test_longitudinal_modes as planner_fixture


class Publisher:
  def __init__(self):
    self.messages = {}
    self.order = []

  def send(self, service, event):
    self.messages[service] = getattr(messaging.log_from_bytes(event.to_bytes()), service)
    self.order.append(service)


class TestImmediateSLCPlanner(unittest.TestCase):
  def setUp(self):
    self.h = planner_fixture.TestLongitudinalModes()
    self.h.setUp()
    self.cs = self.h.sm['carState']
    self.cs.vEgo = 55. / 3.6
    self.cs.canValid = True
    self.h.planner = LongitudinalPlanner(self.h.cp, init_v=self.cs.vEgo, init_a=1.)
    self.h.sm['modelV2'].action.desiredAcceleration = 2.
    self.settings = SLCSettings()
    self.cruise = VCruiseHelper(self.h.cp)
    self.cruise.update_v_cruise(self.cs, False, True, map_enabled=True, automatic_e2e=True, now=self.h.now)
    self.cruise.initialize_v_cruise(self.cs, 105)
    self.assertEqual(self.cruise.v_cruise_kph, 105.)
    self.h.sm['mapCruiseState'].e2eFallback = True
    self.h.sm['mapCruiseState'].state = 'waiting'

  def cruise_step(self, limit, *, publish_mode=True, e2e_ready=True):
    self.cruise.update_v_cruise(self.cs, True, True, map_enabled=True, automatic_e2e=True,
                              map_speed=limit / 3.6 if limit is not None else None,
                              map_gps_time=self.h.now, now=self.h.now, e2e_ready=e2e_ready,
                              slc_settings=self.settings)
    self.cs.vCruise = float(self.cruise.v_cruise_kph)
    if publish_mode:
      selector, status = self.cruise.map_cruise, self.h.sm['mapCruiseState']
      status.state = selector.state(True)
      status.automaticE2e, status.e2eFallback = selector.automatic_e2e, selector.e2e_fallback
    self.h.set_model_speed(self.cs.vEgo)

  def test_known_limit_at_engagement_never_selects_no_map_ceiling(self):
    cruise = VCruiseHelper(self.h.cp)
    cruise.update_v_cruise(self.cs, False, True, map_enabled=True, automatic_e2e=True,
                          map_speed=60. / 3.6, map_gps_time=self.h.now, now=self.h.now)
    cruise.initialize_v_cruise(self.cs, 105)
    self.assertEqual(cruise.v_cruise_kph, 60.)
    self.assertFalse(cruise.map_cruise.e2e_fallback)

  def test_card_applies_60_before_mode_publication_and_old_ack_cannot_restore_105(self):
    sm = self.h.sm
    # Publisher timestamps precede receipt. Equal synthetic float clocks can
    # round a nanosecond timestamp infinitesimally into the future.
    published = self.h.now - .001
    for service in ('mapSpeedLimit', 'longitudinalPlan'):
      sm[service] = getattr(messaging.new_message(service), service)
      sm.services.append(service)
      sm.valid[service], sm.recv_time[service], sm.logMonoTime[service] = True, self.h.now, int(published * 1e9)
    sm['mapSpeedLimit'].speedLimit = 60. / 3.6
    sm['mapSpeedLimit'].gpsMonoTime = int(published * 1e9)
    sm['mapSpeedLimit'].headingValid = True
    sm['mapSpeedLimit'].matchedWayId = 1
    sm['longitudinalPlan'].slcE2eFallback = True  # fresh acknowledgement from the previous no-limit plan
    sm['carControl'].enabled = True
    sm.frame = 1
    sm.update = lambda _: None
    sm.all_checks = lambda *args: True

    card = Car.__new__(Car)
    card.CP, card.sm, card.pm = self.h.cp, sm, Publisher()
    card.can_sock = None
    card.CI = SimpleNamespace(update=lambda _: self.cs)
    card.RI = SimpleNamespace(update=lambda _: None)
    card.CC_prev = car.CarControl(enabled=True)
    card.can_rcv_cum_timeout_counter = 0
    card.v_cruise_helper = self.cruise
    card.is_metric = card.automatic_e2e = card.map_cruise_enabled = True
    card.manual_e2e = False
    card.slc_settings = self.settings
    card._last_map_status = None
    card.last_actuators_output = car.CarControl.Actuators.new_message()
    card.rk = SimpleNamespace(remaining=0.)
    with patch('openpilot.selfdrive.car.card.messaging.drain_sock_raw', return_value=[]), \
         patch('openpilot.selfdrive.car.card.time.monotonic', return_value=self.h.now):
      cs, radar = card.state_update()
      card.state_publish(cs, radar)
    self.assertEqual(cs.vCruise, 60.)
    self.assertLess(card.pm.order.index('carState'), card.pm.order.index('mapCruiseState'))
    self.assertAlmostEqual(card.pm.messages['mapCruiseState'].targetSpeed, 60. / 3.6, places=5)
    self.assertFalse(card.pm.messages['mapCruiseState'].e2eFallback)

    # Deliberately deliver carState first and retain the previous fallback-mode
    # packet. The old E2E candidate can constrain acceleration, never raise SET.
    self.assertTrue(sm['mapCruiseState'].e2eFallback)
    with patch('openpilot.selfdrive.controls.lib.longitudinal_planner.get_cruise_accel', wraps=get_cruise_accel) as calculate:
      plan = self.h.tick(refresh_map=False)
    self.assertAlmostEqual(calculate.call_args.args[0], 60. / 3.6)
    self.assertLessEqual(plan.aTarget, self.h.planner.a_cruise + 1e-6)
    sm['mapCruiseState'] = card.pm.messages['mapCruiseState']
    self.assertFalse(self.h.tick().slcE2eFallback)
    self.assertEqual(self.cruise.v_cruise_kph, 60.)

    # Read a real RES/+ through card and consume its published override in E2E.
    card.manual_e2e = sm['selfdriveState'].experimentalMode = True
    with patch('openpilot.selfdrive.car.card.messaging.drain_sock_raw', return_value=[]), \
         patch('openpilot.selfdrive.car.card.time.monotonic', return_value=self.h.now):
      for pressed in (True, False):
        self.cs.buttonEvents = [car.CarState.ButtonEvent(type=ButtonType.accelCruise, pressed=pressed)]
        cs, radar = card.state_update()
        card.state_publish(cs, radar)
    sm['mapCruiseState'] = card.pm.messages['mapCruiseState']
    self.assertTrue(sm['mapCruiseState'].driverSpeedOverride)
    self.assertAlmostEqual(sm['mapCruiseState'].targetSpeed * 3.6, 60., places=4)
    self.assertEqual(cs.vCruise, 61.)
    with patch('openpilot.selfdrive.controls.lib.longitudinal_planner.get_cruise_accel', wraps=get_cruise_accel) as calculate:
      self.h.tick()
    self.assertAlmostEqual(calculate.call_args.args[0], 61. / 3.6)

  def test_first_recovery_frame_and_higher_limit_use_actual_target_without_ceiling(self):
    for target in (60., 80., 60.):
      self.cruise_step(target)
      self.assertEqual(self.cs.vCruise, target)
      with patch('openpilot.selfdrive.controls.lib.longitudinal_planner.get_cruise_accel', wraps=get_cruise_accel) as calculate:
        plan = self.h.tick()
      self.assertAlmostEqual(calculate.call_args.args[0], target / 3.6)
      self.assertFalse(plan.slcE2eFallback)

  def test_pedal_and_manual_override_survive_immediate_map_acceptance(self):
    self.cs.gasPressed = True
    self.cruise_step(60.)
    self.assertEqual(self.cs.vCruise, 105.)
    self.assertTrue(self.cruise.map_cruise.e2e_fallback)
    self.cs.gasPressed = False
    self.cruise_step(60.)
    self.assertEqual(self.cs.vCruise, 60.)
    self.assertFalse(self.cruise.map_cruise.e2e_fallback)
    for pressed in (True, False):
      self.cs.buttonEvents = [car.CarState.ButtonEvent(type=ButtonType.accelCruise, pressed=pressed)]
      self.cruise_step(60.)
      self.h.tick()
    self.cs.buttonEvents = []
    self.cruise_step(60.)
    self.assertEqual(self.cs.vCruise, 61.)
    self.assertFalse(self.cruise.map_cruise.tracking)
    self.settings = SLCSettings(resume_policy=1)
    self.cruise_step(50.)
    self.assertEqual(self.cs.vCruise, 61.)

  def test_real_lead_braking_remains_stronger_than_new_map_target(self):
    self.cs.vEgo = 20.
    self.h.planner = LongitudinalPlanner(self.h.cp, init_v=20.)
    lead = self.h.sm['radarState'].leadOne
    lead.present, lead.dRel, lead.vLead, lead.vRel = True, 20., 2., -18.
    lead.aLeadTau, lead.modelProb = 1.5, .99
    for _ in range(8):
      self.cruise_step(60.)
      plan = self.h.tick()
    self.assertEqual(self.cs.vCruise, 60.)
    self.assertLess(plan.aTarget, -.5)
    self.assertEqual(str(plan.longitudinalPlanSource), 'lead0')
    self.assertLessEqual(plan.aTarget, self.h.planner.a_cruise)

  def test_recovery_retargets_immediately_with_positive_previous_acceleration(self):
    for speed in (55., 59.):
      with self.subTest(speed=speed):
        self.setUp()
        self.cs.vEgo = speed / 3.6
        self.h.planner = LongitudinalPlanner(self.h.cp, init_v=self.cs.vEgo, init_a=2.)
        self.cruise_step(60.)
        plan = self.h.tick()
        self.assertEqual(self.cs.vCruise, 60.)
        self.assertFalse(plan.slcE2eFallback)
        self.assertLess(self.h.planner.a_cruise, 2.)
        self.assertLessEqual(plan.aTarget, self.h.planner.a_cruise + 1e-6)
        # This changes SET immediately, not the vehicle's physical speed or the
        # native cruise jerk/smoothing policy. Positive acceleration can decay.
        self.assertGreater(self.h.planner.a_cruise, 0.)


if __name__ == '__main__':
  unittest.main()
