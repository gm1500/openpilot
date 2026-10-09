"""Shared interpretation of model slowing and sustained stop trajectories."""
import math
from dataclasses import dataclass

from openpilot.selfdrive.modeld.constants import ModelConstants


@dataclass(frozen=True)
class ModelIntent:
  valid: bool = False
  slowing: bool = False
  stop_distance: float = -1.


def model_intent(model, v_ego: float) -> ModelIntent:
  """Use speed reduction and sustained near-zero speed, not the drawn path length.

  Speed magnitude avoids interpreting a turn's falling forward component as a
  stop. Relative speed reduction avoids an instantaneous model/vehicle bias.
  """
  arrays = (model.velocity.x, model.velocity.y, model.position.x, model.position.y)
  times = ModelConstants.T_IDXS
  if any(len(a) != len(times) or not all(math.isfinite(x) for x in a) for a in arrays):
    return ModelIntent()
  vx, vy, px, py = arrays
  speeds = [math.hypot(x, y) for x, y in zip(vx, vy, strict=True)]
  if max(speeds) > 90:
    return ModelIntent()
  distance = [0.]
  for i in range(1, len(times)):
    step = math.hypot(px[i] - px[i - 1], py[i] - py[i - 1])
    if step > 100 * (times[i] - times[i - 1]) + 2:
      return ModelIntent()
    distance.append(distance[-1] + step)
  drop = max(1., .08 * max(v_ego, 0.))
  slowing = any(1. <= times[i] <= 6. and max(speeds[i:i + 2]) <= speeds[0] - drop for i in range(len(times) - 1))
  stop_distance = -1.
  for i in range(1, len(times) - 1):
    end = next((j for j in range(i + 1, len(times)) if times[j] - times[i] >= .75), None)
    if end is not None and max(speeds[i:end + 1]) <= .5:
      stop_distance = distance[i]
      break
  return ModelIntent(True, slowing, stop_distance)
