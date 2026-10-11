"""Immediate eye motion and sustained, connected-region recording triggers."""
from dataclasses import dataclass
import numpy as np

MOTION_HOLD_SECONDS = 3.
MOTION_GAP_SECONDS = .5


@dataclass
class Motion:
  active: bool = False
  score: float = 0.0
  x: float = 0.5
  y: float = 0.5
  triggered: bool = False


@dataclass
class Region:
  area: int = 0
  moving: int = 0
  sum_x: float = 0.
  sum_y: float = 0.
  left: int = 0
  top: int = 0
  right: int = 0
  bottom: int = 0

  def near(self, other):
    return (self.left <= other.right+4 and other.left <= self.right+4 and
            self.top <= other.bottom+4 and other.top <= self.bottom+4)


def regions(mask, changed):
  """Eight-connected regions, using row runs to avoid a per-pixel Python flood fill."""
  edges = np.diff(np.pad(mask.astype(np.int8), ((0, 0), (1, 1))), axis=1)
  parents, runs, previous = [], [], []

  def root(i):
    while parents[i] != i:
      parents[i] = parents[parents[i]]
      i = parents[i]
    return i

  for y, row in enumerate(edges):
    current = []
    first = 0
    for left, right in zip(np.flatnonzero(row == 1), np.flatnonzero(row == -1), strict=True):
      left, right = int(left), int(right)
      i = len(parents)
      parents.append(i)
      runs.append((y, left, right))
      current.append((left, right, i))
      while first < len(previous) and previous[first][1] < left:
        first += 1
      for prev_left, _, j in previous[first:]:
        if prev_left > right:
          break
        parents[root(j)] = root(i)
    previous = current

  groups = {}
  for i, (y, left, right) in enumerate(runs):
    key = root(i)
    if key not in groups:
      groups[key] = Region(left=left, right=right, top=y, bottom=y+1)
    region = groups[key]
    width = right-left
    region.area += width
    region.moving += int(np.count_nonzero(changed[y, left:right]))
    region.sum_x += (left+right-1)*width/2.
    region.sum_y += y*width
    region.left, region.right = min(left, region.left), max(right, region.right)
    region.bottom = y+1
  return list(groups.values())


class MotionDetector:
  def __init__(self):
    self.background = None
    self.previous = None
    self.last_time = None
    self.warm_until = 0.
    self.candidate_since = None
    self.candidate_last = 0.
    self.candidate_region = None

  def clear_candidate(self):
    self.candidate_since = None
    self.candidate_region = None

  def update(self, image, now, sensitivity=50, armed=True):
    """image is a small 2-D luma view, sampled about five times per second."""
    current = np.asarray(image, dtype=np.float32)
    if current.ndim != 2 or min(current.shape) < 8 or not np.isfinite(current).all():
      self.clear_candidate()
      return Motion()
    if self.background is None or self.background.shape != current.shape or self.last_time is None or not 0 < now-self.last_time < 1.:
      self.background = current.copy()
      self.previous = current.copy()
      self.warm_until = now + 2.
      self.last_time = now
      self.clear_candidate()
      return Motion()
    dt = now-self.last_time
    self.last_time = now
    if not armed:
      # Follow the live exposure throughout camera warmup. Do not carry the
      # dark startup image or accumulated hits into the armed detector.
      self.background = current.copy()
      self.previous = current.copy()
      self.warm_until = now
      self.clear_candidate()
      return Motion()
    sensitivity = min(100, max(1, sensitivity)) / 100.
    difference = current-self.background
    # Reject uniform brightness changes (auto exposure, headlights), not local objects.
    difference -= np.median(difference)
    threshold = 28.-20.*sensitivity
    movement = current-self.previous
    movement -= np.median(movement)
    changed = np.abs(movement) > max(6., threshold*.6)
    self.previous = current.copy()
    mask = np.abs(difference) > threshold
    # Require local support; isolate hot pixels, sensor noise and rain speckles.
    support = sum(np.roll(np.roll(mask, dy, 0), dx, 1) for dy in (-1, 0, 1) for dx in (-1, 0, 1))
    mask &= support >= 5
    mask[[0, -1], :] = False
    mask[:, [0, -1]] = False
    # Slow adaptation retains slow-moving objects, but eventually learns a parked object.
    self.background += min(1., dt/4.)*(current-self.background)
    if mask.mean() >= .65 or changed.mean() >= .65:
      self.background = current.copy()
      self.warm_until = now+1.
    if now < self.warm_until:
      self.clear_candidate()
      return Motion()
    # A stationary new object or its background afterimage is not ongoing motion.
    moving = [r for r in regions(mask, changed) if r.area >= 9 and r.moving >= max(3, r.area*.01)]
    eye = max(moving, key=lambda r: r.area, default=None)
    # A single region must occupy 1.5–6% of the image (2.625% at sensitivity 75).
    # Scattered rain/snow regions cannot pool their area to qualify.
    required = (.06-.045*sensitivity)*current.size
    large = [r for r in moving if r.area >= required]
    candidate = max(large, key=lambda r: (self.candidate_region is not None and r.near(self.candidate_region), r.area), default=None)
    triggered = False
    if candidate is not None:
      if (self.candidate_since is None or now-self.candidate_last > MOTION_GAP_SECONDS or
          not candidate.near(self.candidate_region)):
        self.candidate_since = now
      self.candidate_last, self.candidate_region = now, candidate
      triggered = now-self.candidate_since >= MOTION_HOLD_SECONDS
    elif now-self.candidate_last > MOTION_GAP_SECONDS:
      self.clear_candidate()
    if eye is None:
      return Motion()
    return Motion(True, eye.area/current.size, eye.sum_x/eye.area/(current.shape[1]-1),
                  eye.sum_y/eye.area/(current.shape[0]-1), triggered)
