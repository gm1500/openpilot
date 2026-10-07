import unittest
from types import SimpleNamespace as NS

from opendbc.car.structs import car
from openpilot.selfdrive.car.cruise import VCruiseHelper, ButtonType
from openpilot.selfdrive.car.map_cruise import fallback_e2e_ready


class TestSLCFallback(unittest.TestCase):
  def setUp(self):
    self.helper = VCruiseHelper(car.CarParams(openpilotLongitudinalControl=True))
    self.cs = car.CarState(vEgo=40 / 3.6, cruiseState={'available': True})
    self.helper.initialize_v_cruise(self.cs, False)
    self.now = 100.0
    self.limit = 50.0
    self.enabled = self.toggle = self.automatic = True
    self.ready = False

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
        e2e_ready=self.ready,
      )

  def unknown(self):
    self.tick(210)
    self.limit = None
    self.tick(110)
    self.assertTrue(self.helper.map_cruise.e2e_fallback)

  def test_outage_requests_e2e_before_raising_set_and_restores_slc_after_recovery(self):
    self.unknown()
    self.assertEqual(self.helper.v_cruise_kph, 50)
    self.ready = True
    self.tick()
    self.assertEqual(self.helper.v_cruise_kph, 105)
    self.limit = 40.0
    self.tick(150)
    self.assertTrue(self.helper.map_cruise.e2e_fallback)
    self.assertEqual(self.helper.v_cruise_kph, 105)
    self.tick(60)
    self.assertEqual(self.helper.v_cruise_kph, 40)
    self.assertFalse(self.helper.map_cruise.e2e_fallback)
    self.assertEqual(self.helper.map_cruise.state(True), 'active')

  def test_explicit_set_without_map_requests_immediately_but_still_waits_for_ack(self):
    self.limit = None
    self.tick()
    self.helper.initialize_v_cruise(self.cs, False)
    self.assertTrue(self.helper.map_cruise.e2e_fallback)
    self.assertEqual(self.helper.v_cruise_kph, 40)
    self.tick(100)
    self.assertEqual(self.helper.v_cruise_kph, 40)
    self.ready = True
    self.tick()
    self.assertEqual(self.helper.v_cruise_kph, 105)

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

  def test_recovery_requires_two_seconds_of_advancing_usable_map_data(self):
    self.unknown()
    self.ready = True
    self.tick()
    self.limit = 30.0
    self.tick(210, fixed_gps=self.now)
    self.assertTrue(self.helper.map_cruise.e2e_fallback)
    self.assertEqual(self.helper.v_cruise_kph, 105)
    self.limit = None
    self.tick()
    self.limit = 30.0
    self.tick(150)
    self.limit = None
    self.tick()
    self.limit = 30.0
    self.tick(150)
    self.assertTrue(self.helper.map_cruise.e2e_fallback)
    self.tick(60)
    self.assertEqual(self.helper.v_cruise_kph, 30)
    self.assertFalse(self.helper.map_cruise.e2e_fallback)

  def test_manual_lower_set_is_not_repeatedly_raised_and_recovery_resumes_same_limit(self):
    self.unknown()
    self.ready = True
    self.tick()
    self.tick(button=ButtonType.decelCruise, pressed=True)
    self.tick(button=ButtonType.decelCruise)
    self.tick(250)
    self.assertEqual(self.helper.v_cruise_kph, 104)
    self.assertTrue(self.helper.map_cruise.e2e_fallback)
    self.limit = 50.0
    self.tick(210)
    self.assertEqual(self.helper.v_cruise_kph, 50)
    self.assertFalse(self.helper.map_cruise.e2e_fallback)

  def test_fresh_explicit_set_can_accept_recovered_limit_before_auto_debounce(self):
    self.unknown()
    self.ready = True
    self.tick()
    self.limit = 30.0
    self.cs.vEgo = 20 / 3.6
    self.tick(button=ButtonType.setCruise, pressed=True)
    self.tick(button=ButtonType.setCruise)
    self.assertEqual(self.helper.v_cruise_kph, 30)
    self.assertFalse(self.helper.map_cruise.e2e_fallback)

  def test_override_disengagement_and_toggle_block_automatic_raise(self):
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
    self.assertEqual(self.helper.v_cruise_kph, 105)
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

    sm = SM(longitudinalPlan=NS(conditionalExperimental=NS(e2eEnabled=True, state='active', reason='noSpeedLimit')))
    sm.services = list(sm)
    sm.valid = {'longitudinalPlan': True}
    sm.logMonoTime = {'longitudinalPlan': int(100e9)}
    sm.recv_time = {'longitudinalPlan': 100.0}
    self.assertTrue(fallback_e2e_ready(sm, 100.0))
    self.assertFalse(fallback_e2e_ready(sm, 100.4))
    sm['longitudinalPlan'].conditionalExperimental.reason = 'mapSpeedAvailable'
    self.assertFalse(fallback_e2e_ready(sm, 100.0))
    sm['longitudinalPlan'].conditionalExperimental.reason = 'noSpeedLimit'
    sm.valid['longitudinalPlan'] = False
    self.assertFalse(fallback_e2e_ready(sm, 100.0))


if __name__ == '__main__':
  unittest.main()
