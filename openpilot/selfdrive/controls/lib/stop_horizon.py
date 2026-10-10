"""Recognize a converging model endpoint for immediate E2E approach assistance."""
import math
from dataclasses import dataclass

from openpilot.selfdrive.controls.lib.model_intent import model_intent
from openpilot.selfdrive.modeld.constants import ModelConstants

MAX_MODEL_AGE = .3
MIN_TAIL_TIME = 1.5
MAX_TAIL_TRAVEL = 4.
MAX_TAIL_SPEED = 2.5
MAX_TAIL_SPEED_RATIO = .35
MAX_STATIONARY_TRAVEL = .75


@dataclass(frozen=True)
class HorizonStop:
  distance: float = -1.
  remaining_time: float = 0.


def horizon_stop(model, v_ego):
  """A compact, slow suffix with time left, not the finite end of every path.

  Arc length and speed magnitude also work through turns. Use the existing
  near-zero stop location when available; otherwise use the actual endpoint.
  A looser tail must not move the target backward to the start of deceleration.
  """
  intent = model_intent(model, v_ego)
  if not intent.valid:
    return HorizonStop()
  times = ModelConstants.T_IDXS
  speeds = [math.hypot(x, y) for x, y in zip(model.velocity.x, model.velocity.y, strict=True)]
  arc = [0.]
  for i in range(1, len(times)):
    arc.append(arc[-1] + math.hypot(model.position.x[i] - model.position.x[i - 1],
                                  model.position.y[i] - model.position.y[i - 1]))
  drop = max(1., .08 * max(0., v_ego))
  stationary = v_ego < .1 and max(speeds) <= .5 and arc[-1] <= MAX_STATIONARY_TRAVEL
  for i, t in enumerate(times):
    tail_speed = max(speeds[i:])
    if (times[-1] - t >= MIN_TAIL_TIME and arc[-1] - arc[i] <= MAX_TAIL_TRAVEL and
        tail_speed <= MAX_TAIL_SPEED and
        (stationary or tail_speed <= min(speeds[0] - drop, MAX_TAIL_SPEED_RATIO * speeds[0]))):
      distance = intent.stop_distance if intent.stop_distance >= 0. else arc[-1]
      return HorizonStop(distance, float(times[-1] - t))
  return HorizonStop()


class StopHorizon:
  def __init__(self):
    self.reset()

  def reset(self):
    self.observation = HorizonStop()
    self.timestamp = self.update_time = None

  def update(self, model, *, now, model_time, v_ego):
    if (not all(math.isfinite(x) for x in (now, model_time, v_ego)) or not -.1 <= v_ego <= 75 or
        not 0 <= now - model_time <= MAX_MODEL_AGE or
        self.timestamp is not None and not 0 <= model_time - self.timestamp <= MAX_MODEL_AGE or
        self.update_time is not None and not 0 <= now - self.update_time <= MAX_MODEL_AGE):
      self.reset()
      return
    self.update_time = now
    if model_time == self.timestamp:
      return  # one shape calculation per new model frame
    self.timestamp = model_time
    self.observation = horizon_stop(model, max(0., v_ego))
