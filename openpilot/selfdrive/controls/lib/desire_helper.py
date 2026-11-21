from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.
LANE_CHANGE_START_TIME = 0.5

class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.prev_blinker = False
    self.prev_lane_change_enabled = False
    self.desire = log.Desire.none

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  def prepare_model_desire(self, carstate, lateral_active, *, lane_change_enabled):
    # Clear permission before inference: a missing model output must not leave
    # an old request queued across disengagement or AOL-only operation.
    if not lateral_active or not lane_change_enabled:
      self.update(carstate, lateral_active, 1., lane_change_enabled=lane_change_enabled)
    return self.desire

  def update(self, carstate, lateral_active, lane_change_prob, *, lane_change_enabled=True):
    v_ego = carstate.vEgo
    blinker = carstate.leftBlinker or carstate.rightBlinker
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN
    maneuver_enabled = lateral_active and lane_change_enabled
    # AOL keeps lane tracking active without authorizing lane changes. Only a
    # fresh signal after cruise engagement can initiate a maneuver; signals
    # during AOL and signals held across engagement are never queued.
    new_signal = one_blinker and not self.prev_blinker and self.prev_lane_change_enabled

    if not maneuver_enabled or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
    else:
      if self.lane_change_state == LaneChangeState.off and new_signal and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_timer = 0.0
        # Initialize lane change direction to prevent UI alert flicker
        self.lane_change_direction = self.get_lane_change_direction(carstate)

      elif self.lane_change_state == LaneChangeState.preLaneChange:
        blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                              (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

        if not one_blinker or below_lane_change_speed or self.get_lane_change_direction(carstate) != self.lane_change_direction:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
        elif not blindspot_detected:
          self.lane_change_state = LaneChangeState.laneChangeStarting
          self.lane_change_timer = 0.0

      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.lane_change_timer += DT_MDL

        if lane_change_prob < 0.02 and self.lane_change_timer >= LANE_CHANGE_START_TIME:
          self.lane_change_timer = 0.0
          # Consume this request even if the signal remains on. Another lane
          # change needs both signals off, then a fresh activation while idle.
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none

    # Track inputs while inactive too; disengagement is not a signal release.
    self.prev_blinker = blinker
    self.prev_lane_change_enabled = maneuver_enabled

    self.desire = log.Desire.none
    if self.lane_change_state == LaneChangeState.laneChangeStarting:
      if self.lane_change_direction == LaneChangeDirection.left:
        self.desire = log.Desire.laneChangeLeft
      elif self.lane_change_direction == LaneChangeDirection.right:
        self.desire = log.Desire.laneChangeRight
