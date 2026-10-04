import math

from openpilot.common.constants import CV

ASSIST_MAX_SPEED = 65 / CV.MS_TO_KPH
ASSIST_REARM_SPEED = 63 / CV.MS_TO_KPH
BRAKE_THRESHOLD = -0.2  # m/s^2; ignore near-zero model fluctuations before entry
BRAKE_CONFIRM_TIME = 0.15
GO_THRESHOLD = 0.1  # m/s^2
GO_CONFIRM_TIME = 0.2
RELEASE_JERK = 1.0  # m/s^3; stronger braking is never slew-limited


class E2ESlowingAssist:
  """A deceleration constraint alongside ordinary cruise/lead control, never a mode switch."""

  def __init__(self, dt):
    self.dt = dt
    self.reset()

  def reset(self):
    self.braking = False
    self.brake_time = 0.0
    self.go_time = 0.0
    self.accel_limit = None
    self.speed_blocked = False

  def update(self, *, eligible, v_ego, regular_accel, e2e_accel, e2e_stop):
    # No held model commands after override, disengagement, loss of lead or invalid data.
    if not eligible or not all(math.isfinite(x) for x in (v_ego, regular_accel, e2e_accel)):
      self.reset()
      return regular_accel, False, False

    # Exit at 65; a speed crossing must return below 63 before another entry.
    if v_ego >= ASSIST_MAX_SPEED:
      self.speed_blocked = True
    elif v_ego < ASSIST_REARM_SPEED:
      self.speed_blocked = False

    if self.speed_blocked:
      self.braking = False
      self.brake_time = self.go_time = 0.0
    else:
      self.brake_time = self.brake_time + self.dt if e2e_accel <= BRAKE_THRESHOLD else 0.0
      if self.brake_time >= BRAKE_CONFIRM_TIME:
        self.braking = True

      self.go_time = self.go_time + self.dt if e2e_accel > GO_THRESHOLD and not e2e_stop else 0.0
      if self.go_time >= GO_CONFIRM_TIME:
        self.braking = False

    if self.braking:
      target = min(e2e_accel, 0.0)
    elif self.accel_limit is not None:
      target = regular_accel
    else:
      return regular_accel, False, False

    # Only soften the return toward acceleration. Keep the previous selected
    # command, including stronger regular braking, as the handoff starting point.
    if self.accel_limit is not None:
      target = min(target, self.accel_limit + RELEASE_JERK * self.dt)
    selected = min(regular_accel, target)
    contributing = selected < regular_accel
    self.accel_limit = selected if contributing or (self.braking and self.accel_limit is not None) else None
    return selected, self.braking and e2e_stop, contributing
