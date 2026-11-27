"""Constrain a qualified model stop without changing real-lead following policy."""
import math

from openpilot.selfdrive.controls.lib.model_intent import model_intent
from openpilot.selfdrive.controls.lib.lead_stop import lead_covers_stop, lead_blocks_stop
from openpilot.selfdrive.controls.lib.stop_horizon import StopHorizon
from openpilot.selfdrive.modeld.constants import ModelConstants

MAX_MODEL_AGE = .3
TARGET_FILTER_TIME = .2
MAX_STOP_DISTANCE = 150.
STOP_WITHDRAWAL_TIME = 2.
STOP_WITHDRAWAL_BRAKE = -.2
STOP_WITHDRAWAL_SPEED = 1.5
STOP_WITHDRAWAL_DISTANCE = 3.


def supports_recent_stop(model, v_ego, distance):
  """Require a sustained near-stop at the held position, not braking alone."""
  acceleration = getattr(getattr(model, 'action', None), 'desiredAcceleration', math.nan)
  if not math.isfinite(acceleration) or acceleration > STOP_WITHDRAWAL_BRAKE:
    return False
  times = ModelConstants.T_IDXS
  speeds = [math.hypot(x, y) for x, y in zip(model.velocity.x, model.velocity.y, strict=True)]
  arc = [0.]
  for i in range(1, len(times)):
    arc.append(arc[-1] + math.hypot(model.position.x[i] - model.position.x[i - 1],
                                  model.position.y[i] - model.position.y[i - 1]))
  speed_limit = min(STOP_WITHDRAWAL_SPEED, .35 * speeds[0])
  tolerance = max(STOP_WITHDRAWAL_DISTANCE, .5 * max(0., v_ego))
  for i in range(1, len(times) - 1):
    end = next((j for j in range(i + 1, len(times)) if times[j] - times[i] >= .75), None)
    if (end is not None and max(speeds[i:end + 1]) <= speed_limit and
        distance - tolerance <= arc[i] and arc[end] <= distance + tolerance):
      return True
  return False


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
    self.horizon = StopHorizon()
    self.reset()

  def reset(self):
    self.horizon.reset()
    self._clear_target()

  def _clear_target(self):
    self.requested = self.active = self.holding = False
    self.approaching = False
    self.approach_distance = -1.
    self.distance = self.model_distance = -1.
    self.filtered_distance = -1.
    self.timestamp = self.update_time = self.speed = None
    self.last_stop_observation = None

  def update(self, *, eligible, model, now, model_time, v_ego, e2e_accel, e2e_stop, lead_stop_distance=None, lead_distance=None):
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
    endpoint = self.horizon.observation.distance
    approach = 0. <= endpoint <= MAX_STOP_DISTANCE
    # StopHorizon is the qualified want-to-stop signal on this model API: a
    # sustained, compact, slow terminal path, not merely a finite path endpoint.
    # Acquire its original position as soon as it qualifies, before shouldStop
    # or a near-zero velocity tail. Keep the existing filter and release gates.
    if stop_distance < 0. and approach:
      stop_distance = endpoint

    speed = max(0., v_ego)
    dt = now - self.update_time if self.update_time is not None else 0.
    travel = .5 * (self.speed + speed) * dt if self.active else 0.
    covered = False
    comparison = stop_distance if stop_distance >= 0. else endpoint if approach else None
    raw_stop = comparison - speed * (now - model_time) if comparison is not None else None
    if raw_stop is None and self.active:
      raw_stop = max(0., self.distance - travel)
    if (lead_covers_stop(raw_stop, lead_stop_distance, self.active) or
        lead_blocks_stop(raw_stop, lead_distance, self.active)):
      # A virtual obstacle must be distinct from, and physically ahead of, the
      # real lead. A car running the light cannot pull the stop beyond itself.
      # Suppress the duplicate obstacle, not the fresh model braking request.
      covered = True
      self.active = False
      self.distance = self.filtered_distance = -1.
    if self.active:
      self.distance = max(0., self.distance - travel)
      self.filtered_distance = max(0., self.filtered_distance - travel)
    if (self.active and stop_distance < 0. and not e2e_stop and self.last_stop_observation is not None and
        now - self.last_stop_observation > STOP_WITHDRAWAL_TIME):
      # Repeated, still-fresh continuation frames cannot extend the deadline.
      self._clear_target()
      return
    if model_time != self.timestamp:
      # A small near-zero trajectory change must not abruptly discard braking
      # for an already observed stop. Continue only with fresh braking and a
      # sustained near-stop at that same location. The deadline is anchored to
      # the last actual stop observation; continuation never renews itself.
      continued = (stop_distance < 0. and not e2e_stop and self.active and self.last_stop_observation is not None and
                   0 <= now - self.last_stop_observation <= STOP_WITHDRAWAL_TIME and intent.slowing and
                   supports_recent_stop(model, speed, self.distance + speed * (now - model_time)))
      self.requested = (stop_distance >= 0. or e2e_stop or approach or continued) and not departing(model, speed, e2e_accel, e2e_stop)
      if not self.requested:
        self._clear_target()  # retain the current horizon observation
        return
      self.model_distance = stop_distance if stop_distance >= 0. else endpoint
      if not covered and 0 <= stop_distance <= MAX_STOP_DISTANCE:
        self.last_stop_observation = model_time
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
    self.approaching = self.requested and approach and not self.active and not e2e_stop
    self.approach_distance = max(0., endpoint - speed * (now - model_time)) if self.approaching else -1.
    self.holding = self.requested and (e2e_stop or self.active and self.distance <= .5 and speed < .3)
    self.timestamp, self.update_time, self.speed = model_time, now, speed
