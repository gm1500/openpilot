"""Small luminance detector: exposure compensation, spatial support and persistence."""
from dataclasses import dataclass
import numpy as np


@dataclass
class Motion:
  active: bool = False
  score: float = 0.0
  x: float = 0.5
  y: float = 0.5


class MotionDetector:
  def __init__(self):
    self.background = None
    self.last_time = None
    self.warm_until = 0.
    self.hits = 0

  def update(self, image, now, sensitivity=50, armed=True):
    """image is a small 2-D luma view, sampled about five times per second."""
    current = np.asarray(image, dtype=np.float32)
    if current.ndim != 2 or min(current.shape) < 8 or not np.isfinite(current).all():
      return Motion()
    if self.background is None or self.background.shape != current.shape or self.last_time is None or not 0 < now-self.last_time < 1.:
      self.background = current.copy()
      self.warm_until = now + 2.
      self.last_time = now
      self.hits = 0
      return Motion()
    dt = now-self.last_time
    self.last_time = now
    if not armed:
      # Follow the live exposure throughout camera warmup. Do not carry the
      # dark startup image or accumulated hits into the armed detector.
      self.background = current.copy()
      self.warm_until = now
      self.hits = 0
      return Motion()
    sensitivity = min(100, max(1, sensitivity)) / 100.
    difference = current-self.background
    # Reject uniform brightness changes (auto exposure, headlights), not local objects.
    difference -= np.median(difference)
    threshold = 28.-20.*sensitivity
    mask = np.abs(difference) > threshold
    # Require local support; isolate hot pixels, sensor noise and rain speckles.
    support = sum(np.roll(np.roll(mask, dy, 0), dx, 1) for dy in (-1, 0, 1) for dx in (-1, 0, 1))
    mask &= support >= 5
    mask[[0, -1], :] = False
    mask[:, [0, -1]] = False
    score = float(mask.mean())
    required = .045-.04*sensitivity
    # Global scene changes and camera startup are not a localized motion event.
    candidate = required <= score < .65 and now >= self.warm_until
    self.hits = min(self.hits+1, 5) if candidate else 0
    active = self.hits >= 3
    # Slow adaptation retains slow-moving objects, but eventually learns a parked object.
    self.background += min(1., dt/4.)*(current-self.background)
    if score >= .65:
      self.background = current.copy()
      self.warm_until = now+1.
    y, x = np.nonzero(mask)
    return Motion(active, score, float(x.mean()/(current.shape[1]-1)) if x.size else .5,
                  float(y.mean()/(current.shape[0]-1)) if y.size else .5)
