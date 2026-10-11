import math
import unittest
from types import SimpleNamespace

from opendbc.car.structs import car
from openpilot.selfdrive.car.cruise import VCruiseHelper, ButtonType, IMPERIAL_INCREMENT
from openpilot.selfdrive.car.map_cruise import read_map_speed, read_map_display, lead_limits_speed


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


  def test_dedicated_set_accepts_fresh_limit_before_automatic_qualification(self):
    self.tick()
    self.assertIsNone(self.helper.map_cruise.target_kph)
    self.tap(ButtonType.setCruise)
    self.assertEqual(self.helper.v_cruise_kph, 50)
    self.assertEqual(self.helper.map_pulse.target_kph, 50)
    self.tap(ButtonType.accelCruise)
    self.tick(ticks=220)
    self.assertEqual(self.helper.v_cruise_kph, 51)  # no delayed auto reapply after SET then manual +
    self.limit = 60
    self.tick()
    self.assertEqual(self.helper.v_cruise_kph, 51)
    self.tap(ButtonType.setCruise)
    self.assertEqual(self.helper.v_cruise_kph, 60)

  def test_initial_map_set_pulses_from_40_until_60_on_both_engagement_edges(self):
    for release_before_enable in (False, True):
      with self.subTest(release_before_enable=release_before_enable):
        self.setUp()
        self.limit = 60
        self.enabled = False
        self.tick(ButtonType.decelCruise, True)
        if release_before_enable:
          self.tick(ButtonType.decelCruise, False)
        previous = car.CarState(**self.cs.to_dict()).as_reader()
        self.enabled = True
        self.tick()
        # card initializes AFTER update on the enable edge, using the previous CS.
        self.helper.initialize_v_cruise(previous, True)
        self.assertEqual(self.helper.v_cruise_kph, 60)
        self.assertEqual(self.helper.map_pulse.target_kph, 60)
        self.tick(ticks=10)
        if not release_before_enable:
          self.tick(ButtonType.decelCruise, False)
        self.tick(ticks=220)
        self.assertEqual(self.helper.v_cruise_kph, 60)
        self.assertEqual(self.helper.map_pulse.target_kph, 60)  # still travelling at 40
        self.cs.vEgo = 58.9 / 3.6
        self.tick(ticks=110)
        self.assertEqual(self.helper.map_pulse.target_kph, 60)
        self.cs.vEgo = 59.5 / 3.6
        self.tick(ticks=110)
        self.assertIsNone(self.helper.map_pulse.target_kph)
        self.cs.vEgo = 40 / 3.6
        self.tick(ticks=220)
        self.assertIsNone(self.helper.map_pulse.target_kph)

  def test_initial_map_set_cancellation_and_same_limit_reselection(self):
    for action in ('gas', 'brake', 'disengage', 'toggle', 'minus', 'resume', 'cancel'):
      with self.subTest(action=action):
        self.setUp()
        self.limit = 60
        self.enabled = False
        self.tick(ButtonType.decelCruise, True)
        self.helper.initialize_v_cruise(self.cs, False)
        self.enabled = True
        self.tick(ButtonType.decelCruise, False)
        self.assertEqual(self.helper.map_pulse.target_kph, 60)
        if action in ('gas', 'brake'):
          setattr(self.cs, action + 'Pressed', True)
        elif action == 'disengage':
          self.enabled = False
        elif action == 'toggle':
          self.toggle = False
        else:
          self.tap({'minus': ButtonType.decelCruise, 'resume': ButtonType.resumeCruise, 'cancel': ButtonType.cancel}[action])
        self.tick()
        self.assertIsNone(self.helper.map_pulse.target_kph)
        self.cs.gasPressed = self.cs.brakePressed = False
        self.enabled = self.toggle = True
        self.tick(ticks=220)
        self.assertIsNone(self.helper.map_pulse.target_kph)
        self.enabled = False
        self.tick(ButtonType.decelCruise, True)
        self.helper.initialize_v_cruise(self.cs, False)
        self.enabled = True
        self.tick(ButtonType.decelCruise, False)
        self.assertEqual(self.helper.map_pulse.target_kph, 60)  # explicit SET can restart the same limit
        self.enabled = False
        self.tick(ButtonType.accelCruise, True)
        self.helper.initialize_v_cruise(self.cs, False)
        self.enabled = True
        self.tick(ButtonType.accelCruise, False)
        self.tick(ticks=220)
        self.assertIsNone(self.helper.map_pulse.target_kph)  # RES does not request a map adjustment

  def test_initial_map_set_does_not_pulse_during_pedal_override_or_higher_speed_floor(self):
    for speed, gas, brake in ((40, True, False), (40, False, True), (70, False, False)):
      with self.subTest(speed=speed, gas=gas, brake=brake):
        self.setUp()
        self.limit = 60
        self.cs.vEgo = speed / 3.6
        self.cs.gasPressed, self.cs.brakePressed = gas, brake
        self.enabled = False
        self.tick(ButtonType.decelCruise, True)
        self.helper.initialize_v_cruise(self.cs, False)
        self.enabled = True
        self.tick(ButtonType.decelCruise, False)
        self.assertEqual(self.helper.v_cruise_kph, max(60, speed))
        self.assertIsNone(self.helper.map_pulse.target_kph)
        self.cs.gasPressed = self.cs.brakePressed = False
        self.tick(ticks=220)
        self.assertIsNone(self.helper.map_pulse.target_kph)

  def test_engagement_set_uses_fresh_map_but_resume_keeps_previous(self):
    self.enabled = False
    self.cs.vEgo = 20 / 3.6
    self.limit = 30
    self.tick(ButtonType.decelCruise, True)
    self.assertIsNone(self.helper.map_cruise.target_kph)
    self.helper.initialize_v_cruise(self.cs, True)
    self.assertEqual(self.helper.v_cruise_kph, 30)
    self.limit = 60
    self.tick(ButtonType.accelCruise, True)
    self.helper.initialize_v_cruise(self.cs, True)
    self.assertEqual(self.helper.v_cruise_kph, 30)

  def test_set_cannot_select_missing_stale_or_display_only_data(self):
    for limit in (None, math.nan, 146):
      self.limit = limit
      self.tick()
      self.tap(ButtonType.setCruise)
      self.assertEqual(self.helper.v_cruise_kph, 40)
    self.limit = 60
    self.tick(ButtonType.setCruise, True, fix_time=self.now - 10)
    self.tick(ButtonType.setCruise, False, fix_time=self.now - 10)
    self.assertEqual(self.helper.v_cruise_kph, 40)

  def test_pulse_tracks_actual_speed_until_settled_without_restarting(self):
    self.tick(ticks=210)
    self.assertEqual(self.helper.v_cruise_kph, 50)
    self.assertEqual(self.helper.map_pulse.target_kph, 50)  # set speed reached, vehicle still at 40
    self.cs.vEgo = 49.5 / 3.6
    self.tick(ticks=70)
    self.assertEqual(self.helper.map_pulse.target_kph, 50)
    self.cs.vEgo = 48.9 / 3.6
    self.tick()
    self.cs.vEgo = 50.5 / 3.6
    self.tick(ticks=70)
    self.assertEqual(self.helper.map_pulse.target_kph, 50)
    self.tick(ticks=40)
    self.assertIsNone(self.helper.map_pulse.target_kph)
    self.cs.vEgo = 45 / 3.6
    self.tick(ticks=210)
    self.assertIsNone(self.helper.map_pulse.target_kph)
    self.limit = 30
    self.tick(ticks=210)
    self.assertEqual(self.helper.map_pulse.target_kph, 30)
    self.cs.vEgo = 30.8 / 3.6
    self.tick(ticks=110)
    self.assertIsNone(self.helper.map_pulse.target_kph)

  def test_pulse_cancels_on_driver_intervention_and_stays_cancelled(self):
    for action in ('gas', 'brake', 'disengage', 'toggle', 'plus', 'set', 'resume', 'cancel'):
      with self.subTest(action=action):
        self.setUp()
        self.tick(ticks=210)
        self.assertEqual(self.helper.map_pulse.target_kph, 50)
        if action == 'gas':
          self.cs.gasPressed = True
        elif action == 'brake':
          self.cs.brakePressed = True
        elif action == 'disengage':
          self.enabled = False
        elif action == 'toggle':
          self.toggle = False
        else:
          self.tap({'plus': ButtonType.accelCruise, 'set': ButtonType.decelCruise,
                    'resume': ButtonType.resumeCruise, 'cancel': ButtonType.cancel}[action])
        self.tick()
        self.assertIsNone(self.helper.map_pulse.target_kph)
        self.cs.gasPressed = self.cs.brakePressed = False
        self.enabled = self.toggle = True
        self.tick(ticks=210)
        self.assertIsNone(self.helper.map_pulse.target_kph)
        self.limit = 60
        self.tick(ticks=210)
        self.assertEqual(self.helper.map_pulse.target_kph, 60)

  def test_cancel_during_qualification_does_not_restart_pulse_on_apply(self):
    self.tick(ticks=100)
    self.assertEqual(self.helper.map_pulse.target_kph, 50)
    self.cs.gasPressed = True
    self.tick(ticks=120)
    self.cs.gasPressed = False
    self.tick()
    self.assertEqual(self.helper.v_cruise_kph, 50)
    self.assertIsNone(self.helper.map_pulse.target_kph)

  def test_pulse_missing_map_cancels_preview_but_retains_applied_target(self):
    self.tick(ticks=100)
    self.limit = None
    self.tick()
    self.assertIsNone(self.helper.map_pulse.target_kph)
    self.limit = 50
    self.tick(ticks=210)
    self.assertEqual(self.helper.map_pulse.target_kph, 50)
    self.limit = None
    self.tick(ticks=210)
    self.assertEqual(self.helper.map_pulse.target_kph, 50)
    self.cs.vEgo = 50 / 3.6
    self.tick(ticks=110)
    self.assertIsNone(self.helper.map_pulse.target_kph)


  def test_pulse_message_round_trip_and_default(self):
    from openpilot.cereal import log
    event = log.Event.new_message()
    msg = event.init('mapCruiseState')
    self.assertEqual(msg.adjustingSpeed, 0.)
    self.tick(ticks=100)
    msg.adjustingSpeed = self.helper.map_pulse.target_kph / 3.6
    with log.Event.from_bytes(event.to_bytes()) as decoded:
      self.assertAlmostEqual(decoded.mapCruiseState.adjustingSpeed * 3.6, 50, places=4)

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

  def test_merge_approach_qualifies_both_directions_and_preserves_manual_adjustment(self):
    from openpilot.cereal import log
    class SM(dict):
      pass

    def update_map(distance):
      self.now += 1.
      event = log.Event.new_message(valid=True, logMonoTime=int(self.now * 1e9))
      msg = event.init('mapSpeedLimit')
      msg.speedLimit, msg.distanceAhead = self.limit / 3.6, distance
      msg.gpsMonoTime, msg.headingValid = int(self.now * 1e9), True
      with log.Event.from_bytes(event.to_bytes()) as decoded:
        sm = SM(mapSpeedLimit=decoded.mapSpeedLimit)
        sm.valid = {'mapSpeedLimit': decoded.valid}
        sm.logMonoTime = {'mapSpeedLimit': decoded.logMonoTime}
        sm.recv_time = {'mapSpeedLimit': self.now}
        speed, stamp = read_map_speed(sm, self.now)
        self.cs.buttonEvents = []
        self.helper.update_v_cruise(self.cs, True, True, map_enabled=True, map_speed=speed, map_gps_time=stamp, now=self.now)

    for current, upcoming in ((100, 70), (40, 100)):
      with self.subTest(current=current, upcoming=upcoming):
        self.setUp()
        self.limit = current
        self.tick(ticks=210)
        self.limit = upcoming
        for distance in (200., 180.):
          update_map(distance)
          self.assertEqual(self.helper.v_cruise_kph, current)
        update_map(160.)  # two seconds of fresh fixes, still before the merge
        self.assertEqual(self.helper.v_cruise_kph, upcoming)
        self.assertEqual(self.helper.v_cruise_cluster_kph, upcoming)
        self.tap(ButtonType.decelCruise)
        self.assertEqual(self.helper.v_cruise_kph, upcoming - 1)
        update_map(0.)
        self.tick(ticks=220)
        self.assertEqual(self.helper.v_cruise_kph, upcoming - 1)

  def test_ahead_metadata_preserves_legacy_message_defaults(self):
    from openpilot.cereal import log
    event = log.Event.new_message(valid=True)
    msg = event.init('mapSpeedLimit')
    self.assertEqual(msg.distanceAhead, 0.)
    msg.distanceAhead = 203.
    msg.speedLimit = 70 / 3.6
    with log.Event.from_bytes(event.to_bytes()) as decoded:
      self.assertEqual(decoded.mapSpeedLimit.distanceAhead, 203.)
      self.assertAlmostEqual(decoded.mapSpeedLimit.speedLimit * 3.6, 70., places=4)

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
    self.assertEqual(self.helper.v_cruise_kph, 50)
    self.tap(ButtonType.setCruise)
    self.assertEqual(self.helper.v_cruise_kph, 60)

  def test_set_engages_at_map_limit_instead_of_initial_minimum(self):
    self.enabled = False
    self.cs.vEgo = 20 / 3.6
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

  def test_manual_decrease_without_map_holds_then_follows_new_limit(self):
    self.limit = None
    self.tick()
    self.tap(ButtonType.decelCruise)
    self.assertEqual(self.helper.v_cruise_kph, 39)
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

  def test_long_press_then_short_press_hold_until_new_limit(self):
    self.tick(ticks=210)
    self.tick(ButtonType.decelCruise, True)
    self.tick(ticks=60)
    self.tick(ButtonType.decelCruise, False)
    self.tick(ticks=210)
    self.assertEqual(self.helper.v_cruise_kph, 45)
    self.assertEqual(self.helper.map_cruise.state(True), 'paused')
    self.tap(ButtonType.decelCruise)
    self.tick(ticks=210)
    self.assertEqual(self.helper.v_cruise_kph, 44)
    self.limit = 60
    self.tick(ticks=210)
    self.assertEqual(self.helper.v_cruise_kph, 60)

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
      self.tap(ButtonType.setCruise)  # restart at the 50 km/h map target
      self.tick(button, True)
      for step in range(1, 6):
        self.tick(ticks=50)
        self.assertEqual(self.helper.v_cruise_kph, 50 + sign * 5 * step)
      self.tick(button, False)
      self.tick(ticks=210)
      self.assertEqual(self.helper.v_cruise_kph, 50 + sign * 25)
      self.assertEqual(self.helper.map_cruise.state(True), 'paused')

  def test_gas_defers_auto_but_explicit_set_uses_higher_of_map_and_driving_speed(self):
    self.tick(ticks=210)
    self.cs.gasPressed = True
    self.cs.vEgo = 70 / 3.6
    self.limit = 80
    self.tick(ticks=210)
    self.assertEqual(self.helper.v_cruise_kph, 50)
    self.tap(ButtonType.setCruise)
    self.assertEqual(self.helper.v_cruise_kph, 80)
    self.limit = 60
    self.tick()
    self.tap(ButtonType.setCruise)
    self.assertEqual(self.helper.v_cruise_kph, 70)
    self.cs.gasPressed = False
    self.tick(ticks=210)
    self.assertEqual(self.helper.v_cruise_kph, 70)

  def test_short_minus_decreases_one_step_and_survives_same_limit_recovery(self):
    for metric in (True, False):
      with self.subTest(metric=metric):
        self.setUp()
        self.metric = metric
        self.tick(ticks=210)
        expected = self.helper.v_cruise_kph
        for _ in range(3):
          self.tap(ButtonType.decelCruise)
          expected = round(expected - (1 if metric else IMPERIAL_INCREMENT), 1)
          self.assertEqual(self.helper.v_cruise_kph, expected)
          self.tick(ticks=220)
          self.assertEqual(self.helper.v_cruise_kph, expected)
        self.limit = None
        self.tick(ticks=220)
        self.limit = 50
        self.tick(ticks=220)
        self.assertEqual(self.helper.v_cruise_kph, expected)
        self.limit = 60
        self.tick(ticks=220)
        self.assertEqual(self.helper.v_cruise_kph, 60)

  def test_manual_buttons_win_over_pending_map_qualification(self):
    for button, expected in ((ButtonType.decelCruise, 39), (ButtonType.accelCruise, 41)):
      with self.subTest(button=button):
        self.setUp()
        self.tick(ticks=100)
        self.tap(button)
        self.tick(ticks=220)
        self.assertEqual(self.helper.v_cruise_kph, expected)
        self.assertEqual(self.helper.map_cruise.state(True), 'paused')
        self.assertIsNone(self.helper.map_pulse.target_kph)

  def test_engagement_set_release_does_not_decrement_the_map_target(self):
    self.enabled = False
    self.tick(ButtonType.decelCruise, True)
    self.helper.initialize_v_cruise(self.cs, True)
    self.enabled = True
    self.tick(ButtonType.decelCruise, False)
    self.tick(ticks=220)
    self.assertEqual(self.helper.v_cruise_kph, 50)
    self.tap(ButtonType.decelCruise)
    self.assertEqual(self.helper.v_cruise_kph, 49)

  def test_resume_keeps_previous_speed_while_new_map_limit_is_pending(self):
    self.tick(ticks=220)
    self.enabled = False
    self.limit = 60
    self.tick(ButtonType.accelCruise, True)
    self.helper.initialize_v_cruise(self.cs, True)
    self.enabled = True
    self.tick(ButtonType.accelCruise, False)
    self.tick(ticks=220)
    self.assertEqual(self.helper.v_cruise_kph, 50)

  def test_map_set_initialization_matches_default_speed_floor_with_or_without_gas(self):
    for gas in (False, True):
      for driving, expected in ((20, 60), (70, 70)):
        self.setUp()
        self.enabled = False
        self.cs.vEgo = driving / 3.6
        self.cs.gasPressed = gas
        self.limit = 60
        self.tick(ButtonType.decelCruise, True)
        self.helper.initialize_v_cruise(self.cs, True)
        self.assertEqual(self.helper.v_cruise_kph, expected)
        self.enabled = True
        self.cs.gasPressed = False
        self.tick(ticks=220)
        self.assertEqual(self.helper.v_cruise_kph, expected)

  def test_dedicated_set_with_gas_selects_map_immediately(self):
    self.limit = 60
    self.cs.gasPressed = True
    self.tick()
    self.tap(ButtonType.setCruise)
    self.assertEqual(self.helper.v_cruise_kph, 60)

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
      self.assertIsNone(self.helper.map_pulse.target_kph)

  def test_disengaged_and_unavailable_never_auto_engage(self):
    self.enabled = False
    self.tick(ticks=210)
    self.assertEqual(self.helper.v_cruise_kph, 40)
    self.cs.cruiseState.available = False
    self.enabled = True
    self.tick(ticks=210)
    self.assertEqual(self.helper.v_cruise_kph, 255)
    self.assertIsNone(self.helper.map_pulse.target_kph)

  def test_transport_staleness_and_heading(self):
    class SM(dict):
      pass
    sm = SM(mapSpeedLimit=SimpleNamespace(gpsMonoTime=int(100e9), speedLimit=50 / 3.6, headingValid=True,
                                        positionEstimated=False, positionMonoTime=0))
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
    sm['mapSpeedLimit'].headingValid = True
    sm.recv_time['mapSpeedLimit'] = 50000.  # host uptime is unrelated to recorded GPS/CAN time
    self.assertIsNone(read_map_speed(sm, 104.)[0])
    self.assertAlmostEqual(read_map_speed(sm, 104., replay=True)[0], 50 / 3.6)
    self.assertIsNone(read_map_speed(sm, 105., replay=True)[0])

  def test_projected_position_has_separate_freshness_and_preserves_true_gps_age(self):
    from openpilot.cereal import log
    class SM(dict):
      pass
    event = log.Event.new_message(valid=True, logMonoTime=int(100e9))
    msg = event.init('mapSpeedLimit')
    msg.speedLimit, msg.headingValid = 60 / 3.6, True
    msg.gpsMonoTime = 0  # no live GPS this trip; never relabel the stored point as a new fix
    msg.positionEstimated, msg.positionMonoTime = True, int(100e9)
    with log.Event.from_bytes(event.to_bytes()) as decoded:
      sm = SM(mapSpeedLimit=decoded.mapSpeedLimit)
      sm.valid = {'mapSpeedLimit': True}
      sm.logMonoTime = {'mapSpeedLimit': int(100e9)}
      sm.recv_time = {'mapSpeedLimit': 100.}
      self.assertAlmostEqual(read_map_speed(sm, 100.)[0] * 3.6, 60., places=4)
      self.assertEqual(decoded.mapSpeedLimit.gpsMonoTime, 0)
      sm.logMonoTime['mapSpeedLimit'] = int(101e9)
      sm.recv_time['mapSpeedLimit'] = 101.
      self.assertIsNone(read_map_speed(sm, 101.)[0])
    msg.positionEstimated = False
    sm['mapSpeedLimit'] = msg
    self.assertIsNone(read_map_speed(sm, 100.)[0])

  def test_held_sign_is_hidden_when_not_selectable(self):
    from openpilot.cereal import log
    class SM(dict):
      pass
    event = log.Event.new_message(valid=False, logMonoTime=int(101e9))
    msg = event.init('mapSpeedLimit')
    msg.displayValidDEPRECATED = True
    msg.displaySpeedLimitDEPRECATED = 110 / 3.6
    msg.displayGpsMonoTimeDEPRECATED = int(100e9)
    msg.gpsMonoTime = int(101e9)
    msg.headingValid = True
    msg.speedLimit = 110 / 3.6
    with log.Event.from_bytes(event.to_bytes()) as decoded:
      sm = SM(mapSpeedLimit=decoded.mapSpeedLimit)
      sm.valid = {'mapSpeedLimit': decoded.valid}
      sm.logMonoTime = {'mapSpeedLimit': decoded.logMonoTime}
      sm.recv_time = {'mapSpeedLimit': 101.}
      self.assertIsNone(read_map_display(sm, 101.))
      self.assertIsNone(read_map_speed(sm, 101.)[0])
      sm.logMonoTime['mapSpeedLimit'] = int(102.1e9)
      sm.recv_time['mapSpeedLimit'] = 102.1
      self.assertIsNone(read_map_display(sm, 102.1))  # a new packet cannot prolong old display data

  def test_advisory_sign_serialization_does_not_override_legal_target(self):
    from openpilot.cereal import log
    class SM(dict):
      pass
    for legal in (0., 100 / 3.6):
      event = log.Event.new_message(valid=legal > 0, logMonoTime=int(100e9))
      msg = event.init('mapSpeedLimit')
      self.assertFalse(msg.displayIsAdvisoryDEPRECATED)  # older recordings decode as legal signs
      msg.speedLimit = legal
      msg.gpsMonoTime = int(100e9)
      msg.headingValid = True
      msg.displayValidDEPRECATED = True
      msg.displaySpeedLimitDEPRECATED = 50 / 3.6
      msg.displayGpsMonoTimeDEPRECATED = int(100e9)
      msg.displayIsAdvisoryDEPRECATED = True
      with log.Event.from_bytes(event.to_bytes()) as decoded:
        sm = SM(mapSpeedLimit=decoded.mapSpeedLimit)
        sm.valid = {'mapSpeedLimit': decoded.valid}
        sm.logMonoTime = {'mapSpeedLimit': decoded.logMonoTime}
        sm.recv_time = {'mapSpeedLimit': 100.}
        self.assertTrue(decoded.mapSpeedLimit.displayIsAdvisoryDEPRECATED)
        if legal:
          self.assertAlmostEqual(read_map_display(sm, 100.), legal, places=5)
          self.assertAlmostEqual(read_map_speed(sm, 100.)[0], legal, places=5)
        else:
          self.assertIsNone(read_map_display(sm, 100.))
          self.assertIsNone(read_map_speed(sm, 100.)[0])

  def test_fresh_advisory_fallback_preserves_legal_priority_and_validity(self):
    from openpilot.cereal import log
    class SM(dict):
      pass
    event = log.Event.new_message(valid=True, logMonoTime=int(100e9))
    msg = event.init('mapSpeedLimit')
    msg.gpsMonoTime = int(100e9)
    msg.headingValid = True
    msg.advisorySpeed = 50 / 3.6
    msg.displayValidDEPRECATED = True
    msg.displaySpeedLimitDEPRECATED = 30 / 3.6  # display selection/hold is never the fallback source
    msg.displayGpsMonoTimeDEPRECATED = int(100e9)
    for legal, expected in ((0., 50.), (100., 100.), (40., 40.)):
      msg.speedLimit = legal / 3.6
      event.clear_write_flag()
      with log.Event.from_bytes(event.to_bytes()) as decoded:
        sm = SM(mapSpeedLimit=decoded.mapSpeedLimit)
        sm.valid = {'mapSpeedLimit': decoded.valid}
        sm.logMonoTime = {'mapSpeedLimit': decoded.logMonoTime}
        sm.recv_time = {'mapSpeedLimit': 100.}
        self.assertAlmostEqual(read_map_speed(sm, 100.)[0] * 3.6, expected, places=4)
        self.assertAlmostEqual(read_map_display(sm, 100.) * 3.6, expected, places=4)
    sm['mapSpeedLimit'] = msg
    msg.speedLimit = 0.
    self.assertIsNone(read_map_speed(sm, 101.)[0])  # stale publisher
    sm.logMonoTime['mapSpeedLimit'] = int(104e9)
    sm.recv_time['mapSpeedLimit'] = 104.
    self.assertIsNone(read_map_speed(sm, 104.)[0])  # fresh publisher, stale GPS
    msg.gpsMonoTime = int(104e9)
    msg.headingValid = False
    self.assertIsNone(read_map_speed(sm, 104.)[0])
    msg.headingValid = True
    sm.valid['mapSpeedLimit'] = False
    self.assertIsNone(read_map_speed(sm, 104.)[0])
    sm.valid['mapSpeedLimit'] = True
    for legal in (-1., math.nan, math.inf):
      msg.speedLimit = legal
      self.assertIsNone(read_map_speed(sm, 104.)[0])  # malformed legal data is not absence
    msg.speedLimit = msg.advisorySpeed = 0.
    self.assertIsNone(read_map_speed(sm, 104.)[0])  # held display alone is insufficient


class TestMapPulseLeadLimit(unittest.TestCase):
  def setUp(self):
    class SM(dict):
      pass
    self.sm = SM(longitudinalPlan=SimpleNamespace(longitudinalPlanSource='cruise', e2eAssistActive=False),
                 radarState=SimpleNamespace(leadOne=SimpleNamespace(present=True), leadTwo=SimpleNamespace(present=False)))
    self.sm.valid = dict.fromkeys(self.sm, True)
    self.sm.recv_frame = dict.fromkeys(self.sm, 20)
    self.sm.recv_time = dict.fromkeys(self.sm, 100.)
    self.sm.logMonoTime = dict.fromkeys(self.sm, int(100e9))

  def test_only_selected_real_lead_suppresses_pulse(self):
    plan, radar = self.sm['longitudinalPlan'], self.sm['radarState']
    self.assertFalse(lead_limits_speed(self.sm, 10, 100.))  # lead exists, but cruise governs
    plan.longitudinalPlanSource = 'lead0'
    self.assertTrue(lead_limits_speed(self.sm, 10, 100.))
    radar.leadOne.present = False
    self.assertFalse(lead_limits_speed(self.sm, 10, 100.))  # MPC's synthetic no-lead obstacle
    plan.longitudinalPlanSource = 'lead1'
    self.assertFalse(lead_limits_speed(self.sm, 10, 100.))
    radar.leadTwo.present = True
    self.assertTrue(lead_limits_speed(self.sm, 10, 100.))
    plan.longitudinalPlanSource = 'e2e'
    radar.leadOne.present = True
    self.assertFalse(lead_limits_speed(self.sm, 10, 100.))  # full experimental mode is not lead assistance
    plan.e2eAssistActive = True
    self.assertTrue(lead_limits_speed(self.sm, 10, 100.))
    radar.leadOne.present = False
    self.assertFalse(lead_limits_speed(self.sm, 10, 100.))  # lead-loss handoff
    plan.longitudinalPlanSource = 'cruise'
    self.assertFalse(lead_limits_speed(self.sm, 10, 100.))  # remaining adjustment may pulse again

  def test_stale_invalid_or_previous_drive_plan_cannot_suppress_pulse(self):
    self.sm['longitudinalPlan'].longitudinalPlanSource = 'lead0'
    self.assertFalse(lead_limits_speed(self.sm, 10, 100.31))
    self.assertFalse(lead_limits_speed(self.sm, 10, 99.9))
    for service in self.sm:
      for name, value in (('valid', False), ('recv_frame', 9), ('recv_time', 99.), ('logMonoTime', int(99e9))):
        data = getattr(self.sm, name)
        old = data[service]
        data[service] = value
        self.assertFalse(lead_limits_speed(self.sm, 10, 100.), (service, name))
        data[service] = old


if __name__ == '__main__':
  unittest.main()
