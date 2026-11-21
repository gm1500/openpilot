import unittest
from types import SimpleNamespace as NS

from opendbc.car.structs import car
from openpilot.selfdrive.car.cruise import VCruiseHelper, ButtonType
from openpilot.selfdrive.car.map_cruise import fallback_e2e_ready
from openpilot.selfdrive.car.slc_settings import SLCSettings


class TestSLCFallback(unittest.TestCase):
  def setUp(self):
    self.helper = VCruiseHelper(car.CarParams(openpilotLongitudinalControl=True))
    self.cs = car.CarState(vEgo=40 / 3.6, cruiseState={'available': True})
    self.helper.initialize_v_cruise(self.cs, 40)
    self.now = 100.0
    self.limit = 50.0
    self.enabled = self.toggle = self.automatic = True
    self.manual = False
    self.ready = False
    self.settings = SLCSettings()

  def tick(self, n=1, button=None, pressed=False, fixed_gps=None):
    for _ in range(n):
      self.now += 0.01
      self.cs.buttonEvents = [] if button is None else [car.CarState.ButtonEvent(type=button, pressed=pressed)]
      self.helper.update_v_cruise(
        self.cs,
        self.enabled,
        True,
        map_enabled=self.toggle,
        map_speed=None if self.limit is None else self.limit / 3.6,
        map_gps_time=self.now if fixed_gps is None else fixed_gps,
        now=self.now,
        automatic_e2e=self.automatic,
        manual_e2e=self.manual,
        e2e_ready=self.ready,
        slc_settings=self.settings,
      )

  def unknown(self):
    self.tick(210)
    self.limit = None
    self.tick(110)
    self.assertTrue(self.helper.map_cruise.e2e_fallback)

  def test_outage_requests_e2e_without_changing_set_and_restores_slc_after_recovery(self):
    self.unknown()
    self.assertEqual(self.helper.v_cruise_kph, 50)
    self.ready = True
    self.tick()
    self.assertEqual(self.helper.v_cruise_kph, 50)
    self.assertFalse(self.helper.map_cruise.fallback_set_pending)
    self.limit = 40.0
    self.tick()
    self.assertEqual(self.helper.v_cruise_kph, 40)
    self.assertFalse(self.helper.map_cruise.e2e_fallback)
    self.assertEqual(self.helper.map_cruise.state(True), 'active')

  def test_initial_set_without_map_enters_e2e_at_stock_initial_speed(self):
    self.limit = None
    self.tick()
    self.helper.initialize_v_cruise(self.cs, 40)
    self.assertTrue(self.helper.map_cruise.e2e_fallback)
    self.assertEqual(self.helper.v_cruise_kph, 40)
    self.tick(100)
    self.assertEqual(self.helper.v_cruise_kph, 40)
    self.ready = True
    self.tick()
    self.assertEqual(self.helper.v_cruise_kph, 40)

  def test_explicit_set_without_map_applies_stock_initial_speed_once(self):
    self.helper.initialize_v_cruise(self.cs, 90)
    self.unknown()
    self.assertEqual(self.helper.v_cruise_kph, 50)
    self.tick(button=ButtonType.setCruise, pressed=True)
    self.tick(button=ButtonType.setCruise)
    self.assertEqual(self.helper.v_cruise_kph, 50)
    self.ready = True
    self.tick()
    self.assertEqual(self.helper.v_cruise_kph, 90)
    self.tick(button=ButtonType.decelCruise, pressed=True)
    self.tick(button=ButtonType.decelCruise)
    self.tick(250)
    self.assertEqual(self.helper.v_cruise_kph, 89)
    self.helper.initialize_v_cruise(self.cs, 85)
    self.assertEqual(self.helper.v_cruise_kph, 85)

  def test_fresh_no_map_set_uses_stock_initial_speed_or_higher_current_speed(self):
    for speed, initial_speed, expected in ((40, 105, 105), (115, 105, 115), (150, 105, 145), (10, 30, 40)):
      with self.subTest(speed=speed, initial_speed=initial_speed):
        self.setUp()
        self.cs.vEgo = speed / 3.6
        self.limit = None
        self.tick()
        self.helper.initialize_v_cruise(self.cs, initial_speed)
        self.assertEqual(self.helper.v_cruise_kph, expected)
        self.ready = True
        self.tick(120)
        self.assertEqual(self.helper.v_cruise_kph, expected)

  def test_engaged_explicit_set_waits_for_e2e_then_accepts_higher_current_speed(self):
    self.unknown()
    self.cs.vEgo = 115 / 3.6
    self.tick(button=ButtonType.setCruise, pressed=True)
    self.tick(button=ButtonType.setCruise)
    self.assertEqual(self.helper.v_cruise_kph, 50)
    self.assertTrue(self.helper.map_cruise.fallback_set_pending)
    self.ready = True
    self.tick()
    self.assertEqual(self.helper.v_cruise_kph, 115)

  def test_repeated_map_gaps_retain_previous_set_above_and_below_stock_initial_speed(self):
    for mapped, initial_speed in ((100, 105), (120, 105), (100, 90)):
      with self.subTest(mapped=mapped, initial_speed=initial_speed):
        self.setUp()
        self.helper.initialize_v_cruise(self.cs, initial_speed)
        self.limit = mapped
        self.ready = True
        self.tick()
        for _ in range(3):
          self.limit = None
          self.tick(150)
          self.assertEqual(self.helper.v_cruise_kph, mapped)
          self.assertFalse(self.helper.map_cruise.fallback_set_pending)
          self.limit = mapped
          self.tick()
          self.assertEqual(self.helper.v_cruise_kph, mapped)

  def test_resume_in_map_gap_retains_previous_manual_set(self):
    self.limit = 100
    self.tick()
    self.tick(button=ButtonType.decelCruise, pressed=True)
    self.tick(button=ButtonType.decelCruise)
    self.limit = None
    self.ready = True
    self.tick(150)
    self.enabled = False
    self.tick(button=ButtonType.accelCruise, pressed=True)
    self.helper.initialize_v_cruise(self.cs, 105)
    self.enabled = True
    self.tick(button=ButtonType.accelCruise)
    self.tick(150)
    self.assertEqual(self.helper.v_cruise_kph, 99)

  def test_manual_e2e_map_cap_yields_to_driver_buttons_with_slc_disabled(self):
    self.toggle = False
    self.limit = 60
    self.manual = True
    self.tick()
    self.assertEqual(self.helper.v_cruise_kph, 40)
    self.assertEqual(self.helper.map_cruise.target_kph, 60)
    self.helper.v_cruise_kph = 100
    self.tick()
    self.assertEqual(self.helper.v_cruise_kph, 60)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 60)
    self.tick(button=ButtonType.accelCruise, pressed=True)
    self.tick(button=ButtonType.accelCruise)
    self.tick(110)
    self.assertEqual(self.helper.v_cruise_kph, 61)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 61)
    self.tick(button=ButtonType.decelCruise, pressed=True)
    self.tick(button=ButtonType.decelCruise)
    self.tick(110)
    self.assertEqual(self.helper.v_cruise_kph, 60)

  def test_held_plus_and_gas_set_override_across_mode_and_limit_changes(self):
    self.limit = 110
    self.tick()
    self.tick(button=ButtonType.accelCruise, pressed=True)
    self.tick(50)
    self.tick(button=ButtonType.accelCruise)
    self.assertEqual(self.helper.v_cruise_kph, 115)
    self.manual = True
    self.tick()
    self.assertEqual(self.helper.v_cruise_kph, 115)
    self.cs.gasPressed = True
    self.cs.vEgo = 121 / 3.6
    self.tick(button=ButtonType.decelCruise, pressed=True)
    self.tick(button=ButtonType.decelCruise)
    self.cs.gasPressed = False
    self.limit = None
    self.tick(150)
    self.limit = 110
    self.tick(210)
    self.assertEqual(self.helper.v_cruise_kph, 121)
    self.assertEqual(self.helper.map_cruise.target_kph, 110)
    self.limit = 100
    self.tick()
    self.assertEqual(self.helper.v_cruise_kph, 100)

  def test_explicit_resume_policy_keeps_manual_ceiling_through_new_limit(self):
    self.settings = SLCSettings(resume_policy=1)
    self.manual = True
    self.limit = 110
    self.tick()
    self.tick(button=ButtonType.accelCruise, pressed=True)
    self.tick(button=ButtonType.accelCruise)
    self.limit = 100
    self.tick(110)
    self.assertEqual(self.helper.v_cruise_kph, 111)
    self.tick(button=ButtonType.setCruise, pressed=True)
    self.tick(button=ButtonType.setCruise)
    self.assertEqual(self.helper.v_cruise_kph, 100)
    self.assertFalse(self.helper.map_cruise.driver_speed_override)

  def test_manual_e2e_applies_slc_offset_as_cap_and_retains_it_through_gap(self):
    self.toggle = False
    self.manual = True
    self.settings = SLCSettings(offset_value=5)
    self.limit = 60
    self.helper.v_cruise_kph = 100
    self.tick()
    self.assertEqual(self.helper.v_cruise_kph, 65)
    self.limit = None
    self.ready = True
    self.tick(150)
    self.assertEqual(self.helper.v_cruise_kph, 65)
    self.limit = 50
    self.tick()
    self.assertEqual(self.helper.v_cruise_kph, 55)

  def test_manual_e2e_fresh_engagement_caps_speed_and_engaged_set_accepts_driving_speed(self):
    self.manual = True
    self.cs.vEgo = 80 / 3.6
    self.limit = 60
    self.tick()
    self.helper.initialize_v_cruise(self.cs, 105)
    self.assertEqual(self.helper.v_cruise_kph, 60)
    self.cs.gasPressed = True
    self.tick(button=ButtonType.setCruise, pressed=True)
    self.tick(button=ButtonType.setCruise)
    self.cs.gasPressed = False
    self.tick(110)
    self.assertEqual(self.helper.v_cruise_kph, 80)

  def test_dedicated_set_without_slc_uses_ego_only_for_an_engaged_press(self):
    self.toggle = False
    self.enabled = False
    self.cs.vEgo = 70 / 3.6
    self.tick(button=ButtonType.setCruise, pressed=True)
    self.helper.initialize_v_cruise(self.cs, 90)
    self.enabled = True
    self.tick(button=ButtonType.setCruise)
    self.assertEqual(self.helper.v_cruise_kph, 90)
    for ego, expected in ((75, 75), (155, 145), (5, 8)):
      with self.subTest(ego=ego):
        self.cs.vEgo = ego / 3.6
        self.tick(button=ButtonType.setCruise, pressed=True)
        self.tick(button=ButtonType.setCruise)
        self.tick()
        self.assertEqual(self.helper.v_cruise_kph, expected)
        self.assertEqual(self.helper.v_cruise_cluster_kph, expected)

  def test_disabling_manual_e2e_leaves_manual_set_control_when_slc_is_off(self):
    self.toggle = False
    self.manual = True
    self.limit = 60
    self.helper.v_cruise_kph = 100
    self.tick()
    self.assertEqual(self.helper.v_cruise_kph, 60)
    self.manual = False
    self.tick(button=ButtonType.accelCruise, pressed=True)
    self.tick(button=ButtonType.accelCruise)
    self.tick(110)
    self.assertEqual(self.helper.v_cruise_kph, 61)

  def test_new_outage_preserves_set_when_slc_was_already_paused(self):
    self.limit = 100.
    self.tick(210)
    self.tick(button=ButtonType.resumeCruise, pressed=True)
    self.tick(button=ButtonType.resumeCruise)
    self.assertEqual(self.helper.map_cruise.state(True), 'paused')
    self.limit = None
    self.tick(110)
    self.assertTrue(self.helper.map_cruise.e2e_fallback)
    self.assertEqual(self.helper.v_cruise_kph, 100)
    self.ready = True
    self.tick()
    self.assertEqual(self.helper.v_cruise_kph, 100)
    self.limit = 70.
    self.tick(210)
    self.assertEqual(self.helper.v_cruise_kph, 70)
    self.assertFalse(self.helper.map_cruise.e2e_fallback)

  def test_manual_selection_during_gap_keeps_its_new_set(self):
    self.unknown()
    self.tick(button=ButtonType.decelCruise, pressed=True)
    self.tick(button=ButtonType.decelCruise)
    self.ready = True
    self.tick(250)
    self.assertTrue(self.helper.map_cruise.e2e_fallback)
    self.assertEqual(self.helper.v_cruise_kph, 49)

  def test_no_speed_fallback_clears_old_pulse_and_recovery_starts_map_pulse(self):
    self.tick(210)
    self.assertEqual(self.helper.map_pulse.target_kph, 50)
    self.limit = None
    self.tick(110)
    self.assertTrue(self.helper.map_cruise.e2e_fallback)
    self.assertIsNone(self.helper.map_pulse.target_kph)
    self.ready = True
    self.tick()
    self.assertEqual(self.helper.v_cruise_kph, 50)
    self.assertIsNone(self.helper.map_pulse.target_kph)
    self.limit = 70.
    self.tick()
    self.assertFalse(self.helper.map_cruise.e2e_fallback)
    self.assertEqual(self.helper.v_cruise_kph, 70)
    self.assertEqual(self.helper.map_pulse.target_kph, 70)

  def test_brief_missing_data_does_not_change_set_or_enable_fallback(self):
    self.tick(210)
    self.ready = True
    self.limit = None
    self.tick(50)
    self.assertFalse(self.helper.map_cruise.e2e_fallback)
    self.limit = 50.0
    self.tick(210)
    self.assertFalse(self.helper.map_cruise.e2e_fallback)
    self.assertEqual(self.helper.v_cruise_kph, 50)

  def test_first_valid_sixty_replaces_105_and_never_reenters_fallback_while_fresh(self):
    self.unknown()
    self.helper.initialize_v_cruise(self.cs, 105)
    self.ready = True
    self.tick()
    self.assertEqual(self.helper.v_cruise_kph, 105)
    self.limit = 60.
    self.cs.vEgo = 59 / 3.6
    self.tick(fixed_gps=self.now)
    self.assertEqual(self.helper.v_cruise_kph, 60)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 60)
    self.assertEqual(self.helper.map_cruise.target_kph, 60)
    self.assertFalse(self.helper.map_cruise.e2e_fallback)
    self.assertFalse(self.helper.map_cruise.fallback_set_pending)
    self.cs.vEgo = 60 / 3.6
    for _ in range(500):
      self.tick()
      self.assertEqual(self.helper.v_cruise_kph, 60)
      self.assertFalse(self.helper.map_cruise.e2e_fallback)

  def test_expired_fix_cannot_exit_fallback_but_fresh_same_limit_can(self):
    self.unknown()
    self.ready = True
    self.tick()
    self.limit = 60.
    self.tick(fixed_gps=self.now - 3.1)
    self.assertEqual(self.helper.v_cruise_kph, 50)
    self.assertTrue(self.helper.map_cruise.e2e_fallback)
    self.tick()
    self.assertEqual(self.helper.v_cruise_kph, 60)
    self.assertFalse(self.helper.map_cruise.e2e_fallback)

  def test_manual_lower_set_is_not_repeatedly_raised_and_recovery_resumes_same_limit(self):
    self.unknown()
    self.ready = True
    self.tick()
    self.tick(button=ButtonType.decelCruise, pressed=True)
    self.tick(button=ButtonType.decelCruise)
    self.tick(250)
    self.assertEqual(self.helper.v_cruise_kph, 49)
    self.assertTrue(self.helper.map_cruise.e2e_fallback)
    self.limit = 50.0
    self.tick(210)
    self.assertEqual(self.helper.v_cruise_kph, 50)
    self.assertFalse(self.helper.map_cruise.e2e_fallback)

  def test_fresh_explicit_set_accepts_recovered_limit(self):
    self.unknown()
    self.ready = True
    self.tick()
    self.limit = 30.0
    self.cs.vEgo = 20 / 3.6
    self.tick(button=ButtonType.setCruise, pressed=True)
    self.tick(button=ButtonType.setCruise)
    self.assertEqual(self.helper.v_cruise_kph, 30)
    self.assertFalse(self.helper.map_cruise.e2e_fallback)

  def test_override_disengagement_and_toggle_preserve_outage_set(self):
    for action in ('gasPressed', 'brakePressed', 'enabled', 'toggle', 'automatic'):
      self.setUp()
      self.unknown()
      self.ready = True
      if action in ('gasPressed', 'brakePressed'):
        setattr(self.cs, action, True)
      else:
        setattr(self, action, False)
      self.tick()
      self.assertEqual(self.helper.v_cruise_kph, 50)
      if action in ('toggle', 'automatic'):
        self.assertFalse(self.helper.map_cruise.e2e_fallback)

  def test_gas_override_defers_recovery_release_until_mapped_set_is_applied(self):
    self.unknown()
    self.ready = True
    self.tick()
    self.cs.gasPressed = True
    self.limit = 40.0
    self.tick(210)
    self.assertTrue(self.helper.map_cruise.e2e_fallback)
    self.assertEqual(self.helper.v_cruise_kph, 50)
    self.cs.gasPressed = False
    self.tick()
    self.assertFalse(self.helper.map_cruise.e2e_fallback)
    self.assertEqual(self.helper.v_cruise_kph, 40)

  def test_capnp_request_acknowledgement_and_staleness(self):
    from openpilot.cereal import log

    self.unknown()
    request = log.Event.new_message(valid=True, logMonoTime=int(100e9))
    m = request.init('mapCruiseState')
    m.automaticE2e = self.helper.map_cruise.automatic_e2e
    m.e2eFallback = self.helper.map_cruise.e2e_fallback
    m.setSpeed = self.helper.v_cruise_kph / 3.6
    with log.Event.from_bytes(request.to_bytes()) as decoded:
      self.assertTrue(decoded.mapCruiseState.e2eFallback)
      self.assertTrue(decoded.mapCruiseState.automaticE2e)
      self.assertAlmostEqual(decoded.mapCruiseState.setSpeed * 3.6, 50.0, places=4)

    class SM(dict):
      pass

    sm = SM(longitudinalPlan=NS(slcE2eFallback=True))
    sm.services = list(sm)
    sm.valid = {'longitudinalPlan': True}
    sm.logMonoTime = {'longitudinalPlan': int(100e9)}
    sm.recv_time = {'longitudinalPlan': 100.0}
    self.assertTrue(fallback_e2e_ready(sm, 100.0))
    self.assertFalse(fallback_e2e_ready(sm, 100.4))
    sm['longitudinalPlan'].slcE2eFallback = False
    self.assertFalse(fallback_e2e_ready(sm, 100.0))
    sm['longitudinalPlan'].slcE2eFallback = True
    sm.valid['longitudinalPlan'] = False
    self.assertFalse(fallback_e2e_ready(sm, 100.0))


if __name__ == '__main__':
  unittest.main()
