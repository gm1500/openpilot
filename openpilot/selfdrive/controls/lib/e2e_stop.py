"""Track a stationary model stop target for the ordinary longitudinal MPC."""
import math

from openpilot.selfdrive.controls.lib.model_intent import model_intent
from openpilot.selfdrive.modeld.constants import ModelConstants

MAX_MODEL_AGE = .3
TARGET_FILTER_TIME = .2
MAX_STOP_DISTANCE = 150.
STOP_POSITION_MARGIN = 1.5  # ego stopping position; separate from MPC's lead gap


def real_lead_present(radar_state):
  """Use the same accepted lead slots as ordinary VOACC, including lead two."""
  return radar_state.leadOne.present or radar_state.leadTwo.present


def departing(model, v_ego, e2e_accel, e2e_stop):
  """A launch can start with a near-zero prefix that still satisfies model_intent."""
  if e2e_stop or v_ego >= .5 or e2e_accel <= .1:
    return False
  # Require a sustained moving trajectory, not just one positive action sample.
  speeds = [math.hypot(x, y) for t, x, y in zip(ModelConstants.T_IDXS, model.velocity.x, model.velocity.y, strict=True)
            if 2. <= t <= 4.]
  return bool(speeds) and min(speeds) > .5


class E2EStopTarget:
  def __init__(self):
    self.reset()

  def reset(self):
    self.requested = self.active = self.holding = False
    self.distance = self.model_distance = -1.
    self.filtered_distance = -1.
    self.timestamp = self.update_time = self.speed = None

  def update(self, *, eligible, model, now, model_time, v_ego, e2e_accel, e2e_stop, lead_present=False):
    if (not eligible or lead_present or not all(math.isfinite(x) for x in (now, model_time, v_ego, e2e_accel)) or
        not -.1 <= v_ego <= 75 or not 0 <= now - model_time <= MAX_MODEL_AGE or
        self.timestamp is not None and not 0 <= model_time - self.timestamp <= MAX_MODEL_AGE or
        self.update_time is not None and not 0 <= now - self.update_time <= MAX_MODEL_AGE):
      self.reset()
      return
    intent = model_intent(model, v_ego)
    if not intent.valid:
      self.reset()
      return

    speed = max(0., v_ego)
    dt = now - self.update_time if self.update_time is not None else 0.
    if self.active:
      travel = .5 * (self.speed + speed) * dt
      self.distance = max(0., self.distance - travel)
      self.filtered_distance = max(0., self.filtered_distance - travel)
    if model_time != self.timestamp:
      # A fresh withdrawal releases the position immediately. No grace period
      # may keep a phantom stopped obstacle after the model requests departure.
      self.requested = (intent.stop_distance >= 0. or e2e_stop) and not departing(model, speed, e2e_accel, e2e_stop)
      if not self.requested:
        self.reset()
        return
      self.model_distance = intent.stop_distance
      if 0 <= intent.stop_distance <= MAX_STOP_DISTANCE:
        # Shift each model observation once, before tracking vehicle travel.
        # The published target (and UI marker) includes this margin; raw model
        # distance and MPC's separate standstill lead gap remain unchanged.
        observed = max(0., intent.stop_distance - speed * (now - model_time) - STOP_POSITION_MARGIN)
        if not self.active:
          self.distance = self.filtered_distance = observed
          self.active = True
        else:
          # Filter both signs of observation error before accepting an earlier
          # target. Filtering only negative errors would ratchet the target
          # closer on every noise cycle. The committed stop cannot recede.
          alpha = dt / (TARGET_FILTER_TIME + dt)
          self.filtered_distance += alpha * (observed - self.filtered_distance)
          self.distance = min(self.distance, self.filtered_distance)
    self.holding = self.requested and (e2e_stop or self.active and self.distance <= .5 and speed < .3)
    self.timestamp, self.update_time, self.speed = model_time, now, speed
