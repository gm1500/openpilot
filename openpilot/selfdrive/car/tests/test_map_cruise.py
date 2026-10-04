import math
import unittest
from types import SimpleNamespace

from opendbc.car.structs import car
from openpilot.selfdrive.car.cruise import VCruiseHelper, ButtonType, IMPERIAL_INCREMENT
from openpilot.selfdrive.car.map_cruise import read_map_speed, read_map_display


class TestMapCruise(unittest.TestCase):
  def setUp(self):
    self.helper = VCruiseHelper(car.CarParams(openpilotLongitudinalControl=True, pcmCruise=False))
    self.cs = car.CarState(vEgo=40 / 3.6, cruiseState={'available': True})
    self.helper.initialize_v_cruise(self.cs, False)
    self.now = 100.
    self.limit = 50.
    self.enabled = True
    self.toggle = True
    self.metric = True

  def tick(self, button=None, pressed=False, ticks=1, fix_time=None):
    for _ in range(ticks):
      self.now += .01
      self.cs.buttonEvents = [] if button is None else [car.CarState.ButtonEvent(type=button, pressed=pressed)]
      self.helper.update_v_cruise(self.cs, self.enabled, self.metric, map_enabled=self.toggle,
                                 map_speed=None if self.limit is None else self.limit / 3.6,
                                 map_gps_time=round(self.now, 6) if fix_time is None else fix_time, now=self.now)

  def tap(self, button):
    self.tick(button, True)
    self.tick(button, False)

  def pending(self):
    return self.helper.pending_map_speed(self.cs, self.enabled)

  def test_pending_preview_clears_when_target_changes(self):
    for limit in (50, 30, 145):
      self.limit = limit
      self.tick(ticks=150)
      self.assertEqual(self.pending(), limit)
      self.assertNotEqual(self.helper.v_cruise_kph, limit)
      self.tick(ticks=60)
      self.assertEqual(self.helper.v_cruise_kph, limit)
      self.assertIsNone(self.pending())

  def test_pending_preview_is_read_only(self):
    self.tick(ticks=100)
    before = (self.helper.v_cruise_kph, vars(self.helper.map_cruise).copy())
    for _ in range(100):
      self.assertEqual(self.pending(), 50)
    self.assertEqual((self.helper.v_cruise_kph, vars(self.helper.map_cruise)), before)

  def test_pending_preview_requires_an_actual_change(self):
    self.limit = 40
    self.tick(ticks=100)
    self.assertIsNone(self.pending())
    self.tick(ticks=110)
    self.tap(ButtonType.accelCruise)
    self.limit = None
    self.tick()
    self.assertIsNone(self.pending())
    self.limit = 40  # same limit returning must preserve manual HOLD
    self.tick(ticks=100)
    self.assertIsNone(self.pending())
    self.tick(ticks=110)
    self.assertEqual(self.helper.v_cruise_kph, 41)
    self.limit = 60  # a different zone will resume automatic tracking
    self.tick(ticks=100)
    self.assertEqual(self.pending(), 60)
    self.tick(ticks=110)
    self.assertIsNone(self.pending())
    self.assertEqual(self.helper.v_cruise_kph, 60)

  def test_pending_preview_suppressed_when_application_blocked(self):
    self.tick(ticks=100)
    self.enabled = False
    self.assertIsNone(self.pending())
    self.enabled = True
    self.cs.gasPressed = True
    self.assertIsNone(self.pending())
    self.cs.gasPressed = False
    self.cs.cruiseState.available = False
    self.assertIsNone(self.pending())
    self.cs.cruiseState.available = True
    self.tick(ButtonType.accelCruise, True)
    self.assertIsNone(self.pending())
    self.tick(ticks=220)
    self.assertIsNone(self.pending())
    self.tick(ButtonType.accelCruise, False)
    self.assertIsNone(self.pending())
    self.limit = 80
    self.tick()
    self.assertEqual(self.pending(), 80)
    self.toggle = False
    self.tick()
    self.assertIsNone(self.pending())

  def test_pending_preview_cancels_on_missing_or_stale_data(self):
    self.tick(ticks=100)
    self.assertEqual(self.pending(), 50)
    self.limit = None
    self.tick()
    self.assertIsNone(self.pending())
    self.limit = 60
    self.tick(fix_time=self.now - 4)
    self.assertIsNone(self.pending())
    self.tick()
    self.assertEqual(self.pending(), 60)

  def test_pending_preview_message_round_trip_and_default(self):
    from openpilot.cereal import log
    event = log.Event.new_message()
    msg = event.init('mapCruiseState')
    self.assertEqual(msg.pendingSpeed, 0.)  # old publishers/logs do not pulse
    self.tick(ticks=100)
    msg.pendingSpeed = self.pending() / 3.6
    with log.Event.from_bytes(event.to_bytes()) as decoded:
      self.assertAlmostEqual(decoded.mapCruiseState.pendingSpeed * 3.6, 50, places=4)

  def test_default_off_call_preserves_legacy(self):
    for button, expected in ((ButtonType.accelCruise, 41), (ButtonType.decelCruise, 40)):
      for pressed in (True, False):
        self.cs.buttonEvents = [car.CarState.ButtonEvent(type=button, pressed=pressed)]
        self.helper.update_v_cruise(self.cs, True, True)
      self.assertEqual(self.helper.v_cruise_kph, expected)

  def test_qualified_map_updates_both_displays(self):
    self.tick(ticks=150)
    self.assertEqual(self.helper.v_cruise_kph, 40)
    self.tick(ticks=60)
    self.assertEqual(self.helper.v_cruise_kph, 50)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 50)
    self.assertEqual(self.helper.map_cruise.state(True), 'active')
    self.limit = 30
    self.tick(ticks=210)
    self.assertEqual(self.helper.v_cruise_kph, 30)

  def test_unknown_stale_frozen_and_bad_limits_hold(self):
    self.tick(ticks=210)
    for limit in (None, math.nan, math.inf, -10, 0, 5, 146):
      self.limit = limit
      self.tick(ticks=220)
      self.assertEqual(self.helper.v_cruise_kph, 50)
      self.assertEqual(self.helper.map_cruise.state(True), 'waiting')
    self.limit = 60
    for stamp in (self.now - 10, self.now + 20, self.now):
      self.tick(ticks=220, fix_time=stamp)
      self.assertEqual(self.helper.v_cruise_kph, 50)

  def test_flapping_limits_and_gap_reset_qualification(self):
    for limit in (50, 60, 50, 60, 50, None, 50):
      self.limit = limit
      self.tick(ticks=100)
      self.assertEqual(self.helper.v_cruise_kph, 40)
    self.tick(ticks=110)
    self.assertEqual(self.helper.v_cruise_kph, 50)

  def test_set_and_resume_reengagement(self):
    self.tick(ticks=210)
    self.tap(ButtonType.accelCruise)  # manual + pauses map
    self.assertEqual(self.helper.v_cruise_kph, 51)
    self.enabled = False
    self.limit = 60
    self.tick(ticks=210)
    self.assertEqual(self.helper.v_cruise_kph, 51)
    self.tick(ButtonType.accelCruise, True)
    previous = car.CarState(**self.cs.to_dict()).as_reader()
    self.enabled = True
    self.tick()
    self.helper.initialize_v_cruise(previous, True)  # RES must bypass the experimental 105 km/h default too
    self.tick(ButtonType.accelCruise, False)
    self.tick(ticks=250)
    self.assertEqual(self.helper.v_cruise_kph, 51)
    self.assertEqual(self.helper.map_cruise.state(True), 'paused')
    self.tap(ButtonType.decelCruise)
    self.assertEqual(self.helper.v_cruise_kph, 60)

  def test_set_engages_at_map_limit_instead_of_initial_minimum(self):
    self.enabled = False
    self.limit = 30
    self.tick(ticks=210)
    self.tick(ButtonType.decelCruise, False)
    previous = car.CarState(**self.cs.to_dict()).as_reader()
    self.enabled = True
    self.tick()
    for experimental in (False, True):
      self.helper.initialize_v_cruise(previous, experimental)
      self.assertEqual(self.helper.v_cruise_kph, 30)
      self.assertEqual(self.helper.v_cruise_cluster_kph, 30)

  def test_experimental_initial_speed_without_qualified_map(self):
    self.limit = None
    self.tick()
    self.helper.initialize_v_cruise(self.cs, True)
    self.assertEqual(self.helper.v_cruise_kph, 105)
    self.assertEqual(self.helper.map_cruise.state(True), 'waiting')

  def test_set_without_map_holds_then_follows(self):
    self.limit = None
    self.tick()
    self.tap(ButtonType.decelCruise)
    self.assertEqual(self.helper.v_cruise_kph, 40)
    self.limit = 30
    self.tick(ticks=210)
    self.assertEqual(self.helper.v_cruise_kph, 30)

  def test_limit_change_updates_manual_target_up_and_down(self):
    self.tick(ticks=210)
    self.tap(ButtonType.accelCruise)
    self.assertEqual(self.helper.v_cruise_kph, 51)
    self.limit = 60
    self.tick(ticks=150)
    self.assertEqual(self.helper.v_cruise_kph, 51)
    self.tick(ticks=60)
    self.assertEqual(self.helper.v_cruise_kph, 60)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 60)
    self.limit = 40
    self.tick(ticks=210)
    self.assertEqual(self.helper.v_cruise_kph, 40)

  def test_same_limit_recovery_preserves_resume_hold(self):
    self.tick(ticks=210)
    self.enabled = False
    self.tick(ButtonType.accelCruise, True)
    previous = car.CarState(**self.cs.to_dict()).as_reader()
    self.enabled = True
    self.tick()
    self.helper.initialize_v_cruise(previous, True)
    self.tick(ButtonType.accelCruise, False)
    self.assertEqual(self.helper.map_cruise.state(True), 'paused')
    self.limit = None
    self.tick(ticks=210)
    self.limit = 50
    self.tick(ticks=210)
    self.assertEqual(self.helper.v_cruise_kph, 50)
    self.assertEqual(self.helper.map_cruise.state(True), 'paused')
    self.limit = 80
    self.tick(ticks=210)
    self.assertEqual(self.helper.v_cruise_kph, 80)

  def test_limit_change_during_long_hold_does_not_undo_manual_release(self):
    self.tick(ticks=210)
    self.tick(ButtonType.accelCruise, True)
    self.tick(ticks=15)
    self.limit = 60
    self.tick(ticks=220)
    self.tick(ButtonType.accelCruise, False)
    manual = self.helper.v_cruise_kph
    self.tick(ticks=210)
    self.assertEqual(self.helper.v_cruise_kph, manual)
    self.limit = 80
    self.tick(ticks=210)
    self.assertEqual(self.helper.v_cruise_kph, 80)

  def test_toggle_does_not_jump_to_old_target(self):
    self.tick(ticks=210)
    self.toggle = False
    self.limit = 60
    self.tick(ticks=210)
    self.assertEqual(self.helper.v_cruise_kph, 50)
    self.tap(ButtonType.decelCruise)
    self.assertEqual(self.helper.v_cruise_kph, 49)
    self.toggle = True
    self.tick()
    self.assertEqual(self.helper.v_cruise_kph, 60)

  def test_long_press_adjusts_and_holds_until_set(self):
    self.tick(ticks=210)
    self.tick(ButtonType.decelCruise, True)
    self.tick(ticks=60)
    self.tick(ButtonType.decelCruise, False)
    self.tick(ticks=210)
    self.assertEqual(self.helper.v_cruise_kph, 45)
    self.assertEqual(self.helper.map_cruise.state(True), 'paused')
    self.tap(ButtonType.decelCruise)
    self.assertEqual(self.helper.v_cruise_kph, 50)

  def test_standstill_resume_keeps_set_speed(self):
    self.tick(ticks=210)
    self.cs.cruiseState.standstill = True
    self.limit = None
    self.tick()
    self.tap(ButtonType.accelCruise)
    self.assertEqual(self.helper.v_cruise_kph, 50)

  def test_long_hold_repeats_five_kph_steps_in_both_directions(self):
    self.tick(ticks=210)
    for button, sign in ((ButtonType.accelCruise, 1), (ButtonType.decelCruise, -1)):
      self.tap(ButtonType.decelCruise)  # restart at the 50 km/h map target
      self.tick(button, True)
      for step in range(1, 6):
        self.tick(ticks=50)
        self.assertEqual(self.helper.v_cruise_kph, 50 + sign * 5 * step)
      self.tick(button, False)
      self.tick(ticks=210)
      self.assertEqual(self.helper.v_cruise_kph, 50 + sign * 25)
      self.assertEqual(self.helper.map_cruise.state(True), 'paused')

  def test_gas_override_holds_auto_target_and_keeps_legacy_set_floor(self):
    self.tick(ticks=210)
    self.cs.gasPressed = True
    self.cs.vEgo = 70 / 3.6
    self.limit = 80
    self.tick(ticks=210)
    self.assertEqual(self.helper.v_cruise_kph, 50)
    self.tap(ButtonType.decelCruise)
    self.assertEqual(self.helper.v_cruise_kph, 70)
    self.cs.gasPressed = False
    self.tick(ticks=210)
    self.assertEqual(self.helper.v_cruise_kph, 70)

  def test_imperial_map_limit_and_manual_increment(self):
    self.metric = False
    self.limit = 60 * 1.609344
    self.tick(ticks=210)
    self.assertAlmostEqual(self.helper.v_cruise_kph, self.limit, places=3)
    self.tap(ButtonType.accelCruise)
    self.assertEqual(self.helper.v_cruise_kph, round(self.limit + IMPERIAL_INCREMENT, 1))

  def test_pcm_lateral_only_and_noncar_are_not_overridden(self):
    for kwargs in ({'pcmCruise': True, 'openpilotLongitudinalControl': True},
                   {'pcmCruise': False, 'openpilotLongitudinalControl': False},
                   {'notCar': True, 'openpilotLongitudinalControl': True},
                   {'passive': True, 'openpilotLongitudinalControl': True}):
      self.helper = VCruiseHelper(car.CarParams(**kwargs))
      self.helper.initialize_v_cruise(self.cs, False)
      self.cs.cruiseState.speed = self.cs.cruiseState.speedCluster = 40 / 3.6
      self.tick(ticks=210)
      self.assertAlmostEqual(self.helper.v_cruise_kph, 40, places=4)
      self.assertEqual(self.helper.map_cruise.state(True), 'unsupported')
      self.assertIsNone(self.pending())

  def test_disengaged_and_unavailable_never_auto_engage(self):
    self.enabled = False
    self.tick(ticks=210)
    self.assertEqual(self.helper.v_cruise_kph, 40)
    self.cs.cruiseState.available = False
    self.enabled = True
    self.tick(ticks=210)
    self.assertEqual(self.helper.v_cruise_kph, 255)
    self.assertIsNone(self.pending())

  def test_transport_staleness_and_heading(self):
    class SM(dict):
      pass
    sm = SM(mapSpeedLimit=SimpleNamespace(gpsMonoTime=int(100e9), speedLimit=50 / 3.6, headingValid=True))
    sm.valid = {'mapSpeedLimit': True}
    sm.logMonoTime = {'mapSpeedLimit': int(100e9)}
    sm.recv_time = {'mapSpeedLimit': 100.}
    self.assertAlmostEqual(read_map_speed(sm, 100.)[0], 50 / 3.6)
    self.assertIsNone(read_map_speed(sm, 101.)[0])
    sm.recv_time['mapSpeedLimit'] = 101.
    self.assertIsNone(read_map_speed(sm, 101.)[0])  # stale publisher, fresh receiver
    sm.logMonoTime['mapSpeedLimit'] = int(104e9)
    sm.recv_time['mapSpeedLimit'] = 104.
    self.assertIsNone(read_map_speed(sm, 104.)[0])  # fresh publisher, stale fix
    sm['mapSpeedLimit'].gpsMonoTime = int(104e9)
    sm['mapSpeedLimit'].headingValid = False
    self.assertIsNone(read_map_speed(sm, 104.)[0])
    self.assertAlmostEqual(read_map_speed(sm, 104., require_heading=False)[0], 50 / 3.6)
    sm['mapSpeedLimit'].headingValid = True
    sm.recv_time['mapSpeedLimit'] = 50000.  # host uptime is unrelated to recorded GPS/CAN time
    self.assertIsNone(read_map_speed(sm, 104.)[0])
    self.assertAlmostEqual(read_map_speed(sm, 104., replay=True)[0], 50 / 3.6)
    self.assertIsNone(read_map_speed(sm, 105., replay=True)[0])

  def test_held_sign_is_display_only_and_expires(self):
    from openpilot.cereal import log
    class SM(dict):
      pass
    event = log.Event.new_message(valid=False, logMonoTime=int(101e9))
    msg = event.init('mapSpeedLimit')
    msg.displayValid = True
    msg.displaySpeedLimit = 110 / 3.6
    msg.displayGpsMonoTime = int(100e9)
    msg.gpsMonoTime = int(101e9)
    msg.headingValid = True
    msg.speedLimit = 110 / 3.6
    with log.Event.from_bytes(event.to_bytes()) as decoded:
      sm = SM(mapSpeedLimit=decoded.mapSpeedLimit)
      sm.valid = {'mapSpeedLimit': decoded.valid}
      sm.logMonoTime = {'mapSpeedLimit': decoded.logMonoTime}
      sm.recv_time = {'mapSpeedLimit': 101.}
      self.assertAlmostEqual(read_map_display(sm, 101.), 110 / 3.6, places=5)
      self.assertIsNone(read_map_speed(sm, 101.)[0])
      sm.logMonoTime['mapSpeedLimit'] = int(102.1e9)
      sm.recv_time['mapSpeedLimit'] = 102.1
      self.assertIsNone(read_map_display(sm, 102.1))  # a new packet cannot prolong old display data


if __name__ == '__main__':
  unittest.main()
