"""Track a stationary model stop target for the ordinary longitudinal MPC."""
import math

from openpilot.selfdrive.controls.lib.model_intent import model_intent
from openpilot.selfdrive.controls.lib.lead_stop import lead_covers_stop
from openpilot.selfdrive.controls.lib.stop_horizon import StableStopHorizon
from openpilot.selfdrive.modeld.constants import ModelConstants

MAX_MODEL_AGE = .3
TARGET_FILTER_TIME = .2
MAX_STOP_DISTANCE = 150.


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
    self.horizon = StableStopHorizon()
    self.reset()

  def reset(self):
    self.horizon.reset()
    self._clear_target()

  def _clear_target(self):
    self.requested = self.active = self.holding = False
    self.distance = self.model_distance = -1.
    self.filtered_distance = -1.
    self.timestamp = self.update_time = self.speed = None

  def update(self, *, eligible, model, now, model_time, v_ego, e2e_accel, e2e_stop, lead_stop_distance=None):
    if (not eligible or not all(math.isfinite(x) for x in (now, model_time, v_ego, e2e_accel)) or
        not -.1 <= v_ego <= 75 or not 0 <= now - model_time <= MAX_MODEL_AGE or
        self.timestamp is not None and not 0 <= model_time - self.timestamp <= MAX_MODEL_AGE or
        self.update_time is not None and not 0 <= now - self.update_time <= MAX_MODEL_AGE):
      self.reset()
      return
    intent = model_intent(model, v_ego)
    if not intent.valid:
      self.reset()
      return

    stop_distance = intent.stop_distance
    self.horizon.update(model, now=now, model_time=model_time, v_ego=v_ego)
    # A stable endpoint is an additional stop trigger, never a delay on the
    # existing near-zero trajectory detector. Once acquired, either ongoing
    # stop trajectory can retain the target through endpoint jitter.
    if stop_distance < 0. and (self.horizon.stable or self.active and self.horizon.observation.distance >= 0.):
      stop_distance = self.horizon.observation.distance

    speed = max(0., v_ego)
    dt = now - self.update_time if self.update_time is not None else 0.
    travel = .5 * (self.speed + speed) * dt if self.active else 0.
    if lead_stop_distance is not None and math.isfinite(lead_stop_distance):
      # Compare model stopping positions after compensating for vehicle travel.
      comparison = stop_distance
      if comparison < 0. and self.horizon.observation.distance >= 0.:
        comparison = self.horizon.observation.distance
      raw_stop = comparison - speed * (now - model_time) if comparison >= 0. else None
      if raw_stop is None and self.active:
        raw_stop = max(0., self.distance - travel)
      if lead_covers_stop(raw_stop, lead_stop_distance, self.active):
        self.reset()  # ordinary lead control already covers this stop
        return
    if self.active:
      self.distance = max(0., self.distance - travel)
      self.filtered_distance = max(0., self.filtered_distance - travel)
    if model_time != self.timestamp:
      # A fresh withdrawal releases the position immediately. No grace period
      # may keep a phantom stopped obstacle after the model requests departure.
      self.requested = (stop_distance >= 0. or e2e_stop) and not departing(model, speed, e2e_accel, e2e_stop)
      if not self.requested:
        self._clear_target()  # retain only the new detector's qualification history
        return
      self.model_distance = stop_distance
      if 0 <= stop_distance <= MAX_STOP_DISTANCE:
        # Compensate for travel since the observation without adding an offset.
        observed = max(0., stop_distance - speed * (now - model_time))
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
