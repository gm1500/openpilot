"""Qualify a stationary endpoint before acquiring a virtual stopped lead."""
import math
from collections import deque
from dataclasses import dataclass

from openpilot.selfdrive.controls.lib.model_intent import model_intent
from openpilot.selfdrive.modeld.constants import ModelConstants

MAX_MODEL_AGE = .3
CONFIRM_TIME = .3
MIN_TAIL_TIME = 1.5
MAX_TAIL_TRAVEL = .75
MAX_TAIL_SPEED = 1.
MAX_ENDPOINT_SPREAD = 1.


@dataclass(frozen=True)
class HorizonStop:
  distance: float = -1.
  endpoint: float = -1.
  remaining_time: float = 0.


def horizon_stop(model, v_ego):
  """A compact, slow suffix with time left, not the finite end of every path.

  Arc length and speed magnitude also work through turns. Use the existing
  near-zero stop location when available; otherwise use the plateau's start.
  The final endpoint is used only to check stability across observations.
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
  stationary = v_ego < .1 and max(speeds) <= .5 and arc[-1] <= MAX_TAIL_TRAVEL
  for i, t in enumerate(times):
    tail_speed = max(speeds[i:])
    if (times[-1] - t >= MIN_TAIL_TIME and arc[-1] - arc[i] <= MAX_TAIL_TRAVEL and
        tail_speed <= MAX_TAIL_SPEED and (stationary or tail_speed <= speeds[0] - drop)):
      distance = intent.stop_distance if intent.stop_distance >= 0. else arc[i]
      return HorizonStop(distance, arc[-1], float(times[-1] - t))
  return HorizonStop()


class StableStopHorizon:
  def __init__(self):
    self.reset()

  def reset(self):
    self.observation = HorizonStop()
    self.stable = False
    self.stable_time = self.spread = 0.
    self.timestamp = self.update_time = None
    self.travel = self.speed = 0.
    self.samples = deque()

  def update(self, model, *, now, model_time, v_ego):
    if (not all(math.isfinite(x) for x in (now, model_time, v_ego)) or not -.1 <= v_ego <= 75 or
        not 0 <= now - model_time <= MAX_MODEL_AGE or
        self.timestamp is not None and not 0 <= model_time - self.timestamp <= MAX_MODEL_AGE or
        self.update_time is not None and not 0 <= now - self.update_time <= MAX_MODEL_AGE):
      self.reset()
      return
    speed = max(0., v_ego)
    if self.update_time is not None:
      self.travel += .5 * (self.speed + speed) * (now - self.update_time)
    self.update_time, self.speed = now, speed
    if model_time == self.timestamp:
      return  # repeated frames cannot confirm stability
    self.timestamp = model_time
    self.observation = horizon_stop(model, speed)
    self.stable = False
    self.stable_time = self.spread = 0.
    if not 0 <= self.observation.distance <= 150.:
      self.samples.clear()
      return
    # Compare in a travelled-distance frame, including observation age. An
    # endpoint fixed relative to the moving car is not a fixed road location.
    endpoint = self.observation.endpoint - speed * (now - model_time) + self.travel
    self.samples.append((model_time, endpoint))
    while len(self.samples) > 1 and self.samples[1][0] <= model_time - CONFIRM_TIME:
      self.samples.popleft()
    positions = [s[1] for s in self.samples]
    self.spread = max(positions) - min(positions)
    # A jump restarts qualification; no stale stable result survives it.
    while len(self.samples) > 1 and max(positions) - min(positions) > MAX_ENDPOINT_SPREAD:
      self.samples.popleft()
      positions = [s[1] for s in self.samples]
    self.stable_time = model_time - self.samples[0][0]
    self.stable = len(self.samples) >= 4 and self.stable_time >= CONFIRM_TIME - 1e-6
