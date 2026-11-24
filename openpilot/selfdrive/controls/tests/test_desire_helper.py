import unittest
from types import SimpleNamespace

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import (
  DesireHelper, LaneChangeDirection, LaneChangeState, LANE_CHANGE_SPEED_MIN, LANE_CHANGE_START_TIME, LANE_CHANGE_TIME_MAX,
)


class TestDesireHelper(unittest.TestCase):
  def setUp(self):
    self.helper = DesireHelper()
    self.carstate = SimpleNamespace(vEgo=25., leftBlinker=False, rightBlinker=False, leftBlindspot=False, rightBlindspot=False)

  def step(self, signal="off", active=True, prob=1., speed=25., blindspot=False, frames=1, cruise_enabled=True):
    self.carstate.leftBlinker = signal in ("left", "hazards")
    self.carstate.rightBlinker = signal in ("right", "hazards")
    self.carstate.leftBlindspot = self.carstate.rightBlindspot = blindspot
    self.carstate.vEgo = speed
    for _ in range(frames):
      self.helper.update(self.carstate, active, prob, lane_change_enabled=cruise_enabled)
    return self.helper.lane_change_state

  def assert_off(self):
    self.assertEqual(self.helper.lane_change_state, LaneChangeState.off)
    self.assertEqual(self.helper.lane_change_direction, LaneChangeDirection.none)
    self.assertEqual(self.helper.desire, log.Desire.none)

  def start(self, signal="right"):
    self.step()
    self.assertEqual(self.step(signal), LaneChangeState.preLaneChange)
    self.assertEqual(self.helper.desire, log.Desire.none)
    self.assertEqual(self.step(signal), LaneChangeState.laneChangeStarting)
    self.assertEqual(self.helper.desire, log.Desire.laneChangeLeft if signal == "left" else log.Desire.laneChangeRight)

  def complete(self, signal="right"):
    self.step(signal, prob=0., frames=int(LANE_CHANGE_START_TIME / DT_MDL) + 2)
    self.assert_off()

  def test_fresh_signal_starts_in_either_direction(self):
    for signal in ("left", "right"):
      with self.subTest(signal=signal):
        self.setUp()
        self.start(signal)

  def test_held_signal_cannot_repeat_after_completion(self):
    for signal in ("left", "right"):
      with self.subTest(signal=signal):
        self.setUp()
        self.start(signal)
        self.complete(signal)
        for _ in range(400):
          self.step(signal, prob=0.)
          self.assert_off()

  def test_new_signal_after_completion_allows_next_lane_change(self):
    for signal in ("left", "right"):
      with self.subTest(signal=signal):
        self.setUp()
        self.start(signal)
        self.complete(signal)
        self.start(signal)

  def test_engagement_with_existing_signal_does_not_request_lane_change(self):
    for signal in ("left", "right"):
      with self.subTest(signal=signal):
        self.setUp()
        self.step(signal, active=False, frames=10)
        for _ in range(50):
          self.step(signal)
          self.assert_off()
        self.start(signal)

  def test_signal_and_engagement_in_same_frame_are_not_a_request(self):
    self.step(active=False)
    self.step("right", frames=50)
    self.assert_off()
    self.start()

  def test_startup_with_existing_signal_is_not_a_request(self):
    self.step("right", frames=50)
    self.assert_off()
    self.start()

  def test_aol_signal_edges_never_start_lane_changes(self):
    for signal in ("left", "right", "hazards"):
      with self.subTest(signal=signal):
        self.setUp()
        self.step(cruise_enabled=False)
        self.step(signal, cruise_enabled=False, frames=100)
        self.assert_off()
        self.step(cruise_enabled=False)
        self.step(signal, cruise_enabled=False, frames=100)
        self.assert_off()

  def test_aol_to_cruise_with_held_signal_requires_new_signal(self):
    for signal in ("left", "right"):
      with self.subTest(signal=signal):
        self.setUp()
        self.step(cruise_enabled=False)
        self.step(signal, cruise_enabled=False, frames=20)
        self.step(signal, frames=100)
        self.assert_off()
        self.start(signal)

  def test_signal_on_same_frame_as_aol_to_cruise_is_not_a_request(self):
    self.step(cruise_enabled=False)
    self.step("right", frames=50)
    self.assert_off()
    self.start()

  def test_cruise_to_aol_cancels_pending_or_active_maneuver(self):
    for waiting in (True, False):
      with self.subTest(waiting=waiting):
        self.setUp()
        self.step()
        self.step("right", blindspot=waiting, frames=2)
        self.assertNotEqual(self.helper.lane_change_state, LaneChangeState.off)
        self.step("right", cruise_enabled=False)
        self.assert_off()
        self.assertEqual(self.helper.lane_change_timer, 0.)
        self.step("right", frames=50)
        self.assert_off()
        self.start()

  def test_inference_failure_cannot_keep_request_across_aol_to_cruise(self):
    self.start()
    desire = self.helper.prepare_model_desire(self.carstate, True, lane_change_enabled=False)
    self.assertEqual(desire, log.Desire.none)
    self.assert_off()
    # No post-inference update occurred. The next frame is cruise-engaged with
    # the same signal; neither the model input nor helper may reuse that pulse.
    desire = self.helper.prepare_model_desire(self.carstate, True, lane_change_enabled=True)
    self.assertEqual(desire, log.Desire.none)
    self.step("right", frames=50)
    self.assert_off()
    self.start()

  def test_disengagement_cancels_and_does_not_rearm_held_signal(self):
    for waiting in (True, False):
      with self.subTest(waiting=waiting):
        self.setUp()
        self.step()
        self.step("right", blindspot=waiting, frames=2)
        self.assertNotEqual(self.helper.lane_change_state, LaneChangeState.off)
        self.step("right", active=False)
        self.assert_off()
        self.assertEqual(self.helper.lane_change_timer, 0.)
        self.step("right", frames=50)
        self.assert_off()
        self.start()

  def test_signal_cycle_while_disengaged_is_not_queued(self):
    self.step(active=False)
    self.step("right", active=False)
    self.step(active=False)
    self.step("left", active=False)
    self.step("left", frames=50)
    self.assert_off()
    self.start("left")

  def test_hazards_and_hazards_to_single_signal_do_not_start(self):
    for signal in ("left", "right"):
      with self.subTest(signal=signal):
        self.setUp()
        self.step()
        self.step("hazards", frames=20)
        self.assert_off()
        self.step(signal, frames=20)
        self.assert_off()
        self.start(signal)

  def test_below_minimum_speed_signal_is_not_deferred(self):
    self.step()
    self.step("right", speed=LANE_CHANGE_SPEED_MIN - .1, frames=5)
    self.assert_off()
    self.step("right", frames=50)
    self.assert_off()
    self.start()

  def test_minimum_speed_still_allows_fresh_signal(self):
    self.step(speed=LANE_CHANGE_SPEED_MIN)
    self.assertEqual(self.step("right", speed=LANE_CHANGE_SPEED_MIN), LaneChangeState.preLaneChange)
    self.assertEqual(self.step("right", speed=LANE_CHANGE_SPEED_MIN), LaneChangeState.laneChangeStarting)

  def test_blindspot_blocks_start_until_clear_for_same_request(self):
    self.step()
    self.step("right", blindspot=True, frames=20)
    self.assertEqual(self.helper.lane_change_state, LaneChangeState.preLaneChange)
    self.assertEqual(self.helper.desire, log.Desire.none)
    self.assertEqual(self.step("right"), LaneChangeState.laneChangeStarting)
    self.complete()

  def test_cancel_pending_request_requires_new_signal(self):
    for cancel_signal in ("off", "hazards"):
      with self.subTest(cancel_signal=cancel_signal):
        self.setUp()
        self.step()
        self.step("right", blindspot=True, frames=5)
        self.step(cancel_signal)
        self.assert_off()
        if cancel_signal == "hazards":
          self.step("right", frames=5)
          self.assert_off()
        self.start()

  def test_direction_switch_without_off_cancels_pending_request(self):
    self.step()
    self.step("right", blindspot=True, frames=5)
    self.step("left", frames=20)
    self.assert_off()
    self.start("left")

  def test_speed_drop_cancels_pending_request_without_rearm(self):
    self.step()
    self.step("right", blindspot=True, frames=5)
    self.step("right", speed=LANE_CHANGE_SPEED_MIN - .1)
    self.assert_off()
    self.step("right", frames=20)
    self.assert_off()
    self.start()

  def test_signal_cycle_during_maneuver_is_not_queued(self):
    self.start()
    self.step("off")
    self.step("left")
    self.assertEqual(self.helper.desire, log.Desire.laneChangeRight)
    self.complete("left")
    self.step("left", frames=20)
    self.assert_off()
    self.start("left")

  def test_timeout_consumes_request_until_new_signal(self):
    self.start()
    self.step("right", frames=int(LANE_CHANGE_TIME_MAX / DT_MDL) + 3)
    self.assert_off()
    self.step("right", frames=20)
    self.assert_off()
    self.start()

  def test_completion_still_requires_minimum_time_and_low_probability(self):
    self.start()
    self.step("right", prob=0.)
    self.assertEqual(self.helper.lane_change_state, LaneChangeState.laneChangeStarting)
    self.step("right", prob=1., frames=int(1. / DT_MDL))
    self.assertEqual(self.helper.lane_change_state, LaneChangeState.laneChangeStarting)
    self.step("right", prob=0.)
    self.assert_off()


if __name__ == "__main__":
  unittest.main()
