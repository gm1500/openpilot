"""Immediate eye motion and sustained, connected-region recording triggers."""
from dataclasses import dataclass
import math
import numpy as np

from openpilot.system.sentry.config import MOTION_CAMERAS

MOTION_HOLD_SECONDS = .6
MOTION_GAP_SECONDS = .5
MIN_REGION_FRACTION = .005


def camera_area_scale(stream, width, height):
  """Approximate projected area from the configured lens, before the speck floor."""
  from openpilot.common.transformations.camera import DEVICE_CAMERAS
  name = ('narrow_road', 'cabin', 'wide_road')[stream]
  cameras = DEVICE_CAMERAS[('tizi', 'ox03c10')]
  # The supported comma 3X sensor families have distinct output dimensions.
  # AR/OX share optics; use those as the fallback until dimensions are known.
  for sensor in ('ox03c10', 'os04c10'):
    candidate = DEVICE_CAMERAS[('tizi', sensor)]
    if getattr(candidate, name).size == (width, height):
      cameras = candidate
      break
  camera, narrow = getattr(cameras, name), cameras.narrow_road
  projected_area = camera.focal_length**2/(camera.width*camera.height)
  reference_area = narrow.focal_length**2/(narrow.width*narrow.height)
  # Raise narrow's minimum area; compensate both wide lenses for smaller
  # projected objects. Fisheye scaling is approximate, especially at the edges.
  return 2.*projected_area/reference_area


@dataclass
class Motion:
  active: bool = False
  score: float = 0.0
  x: float = 0.5
  y: float = 0.5
  triggered: bool = False


class PreviewSelector:
  """Follow stronger motion without alternating views on near-ties or brief shadows."""
  def __init__(self):
    self.reset()

  def reset(self):
    self.camera = 2
    self.scores = {}
    self.last_time = None
    self.switched_at = -math.inf
    self.pending = None
    self.pending_since = 0.

  def update(self, results, now, recording=False, manual=False):
    fresh = {stream: m.score for stream, m in results.items() if stream in MOTION_CAMERAS and m.active}
    dt = now-self.last_time if self.last_time is not None else 0.
    self.last_time = now
    if not recording or manual:
      candidates = {s: score for s, score in fresh.items() if manual or results[s].triggered}
      if candidates:
        self.camera = max(candidates, key=lambda s: (candidates[s], s == self.camera))
      elif manual:
        self.camera = next((s for s in MOTION_CAMERAS if s in results), self.camera)
      self.scores, self.pending, self.switched_at = fresh, None, now
      return self.camera

    # Brief pauses in a person's stride should not let a reflection steal the
    # view. Only fresh motion can challenge; decayed scores support staying put.
    if not 0 < dt <= 1.:
      self.scores, self.pending = {}, None
    self.scores = {s: max(fresh.get(s, 0.), self.scores.get(s, 0.)*math.exp(-max(dt, 0.)/.2)) for s in MOTION_CAMERAS}
    challenger = max(fresh, key=lambda s: (fresh[s], s == self.camera), default=self.camera)
    score, current = fresh.get(challenger, 0.), self.scores.get(self.camera, 0.)
    if challenger == self.camera or score < max(.001, 1.35*current+.0005):
      self.pending = None
      return self.camera

    # A clear new subject still hands off immediately. Smaller differences
    # need two samples (0.2 s at 5 Hz), plus a one-second hold after a cut.
    decisive = score >= max(.005, 2.5*current+.002)
    if self.pending != challenger:
      self.pending, self.pending_since = challenger, now
    # Leave scheduling tolerance around the nominal 0.2 s sample interval.
    if decisive or (now-self.pending_since >= .15 and now-self.switched_at >= 1.-1e-6):
      self.camera, self.switched_at, self.pending = challenger, now, None
    return self.camera


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


def box_mean(values):
  padded = np.pad(values, 1, mode='edge')
  return sum(padded[y:y+values.shape[0], x:x+values.shape[1]] for y in range(3) for x in range(3))/9.


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
    # Track the pixels changing NOW, not the center of a slowly fading
    # background footprint left behind by the subject.
    changed_x = np.flatnonzero(changed[y, left:right])+left
    region.moving += changed_x.size
    region.sum_x += int(changed_x.sum())
    region.sum_y += y*changed_x.size
    region.left, region.right = min(left, region.left), max(right, region.right)
    region.bottom = y+1
  return list(groups.values())


class MotionDetector:
  def __init__(self, area_scale=1., midpoint_threshold=18.):
    self.area_scale = area_scale
    self.midpoint_threshold = midpoint_threshold
    self.background = None
    self.previous = None
    self.last_time = None
    self.warm_until = 0.
    self.candidate_since = None
    self.candidate_last = 0.
    self.candidate_region = None

  @classmethod
  def for_camera(cls, stream, width, height):
    # Cabin IR/window motion has lower luma contrast in the supplied footage.
    # Calibrate the slider's midpoint per camera while keeping the original
    # least/most-sensitive endpoints, lens area floor and confirmation time.
    return cls(camera_area_scale(stream, width, height), midpoint_threshold=10. if stream == 1 else 18.)

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
    threshold = float(np.interp(sensitivity, (.01, .5, 1.), (27.8, self.midpoint_threshold, 8.)))
    movement = current-self.previous
    movement -= np.median(movement)
    movement_threshold = max(6., threshold*.6)
    # In a dark garage the subject can differ by just 3-5 luma levels. Use a
    # lower contrast threshold only in low light, with spatial averaging and
    # a measured temporal-noise floor. Daylight thresholds stay unchanged.
    light = float(np.clip((np.percentile(self.background, 80)-28.)/36., 0., 1.))
    if light < 1.:
      difference = light*difference+(1.-light)*box_mean(difference)
      movement = light*movement+(1.-light)*box_mean(movement)
      noise = float(np.median(np.abs(movement)))
      dark_threshold = float(np.interp(sensitivity, (.01, .5, 1.), (6., 2.5, 1.75)))
      dark_movement = float(np.interp(sensitivity, (.01, .5, 1.), (3., 1., .75)))
      threshold = max(light*threshold+(1.-light)*dark_threshold, 4.*noise)
      movement_threshold = max(light*movement_threshold+(1.-light)*dark_movement, 3.*noise)
    changed = np.abs(movement) > movement_threshold
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
    eye = max(moving, key=lambda r: (r.moving, r.area), default=None)
    # Scale recording area by the lens, but never below the speck floor.
    # Scattered rain/snow cannot pool area. The immediate eye is independent.
    required = max(MIN_REGION_FRACTION, (.06-.045*sensitivity)*self.area_scale)*current.size
    large = [r for r in moving if r.area >= required]
    candidate = max(large, key=lambda r: (self.candidate_region is not None and r.near(self.candidate_region), r.area), default=None)
    triggered = False
    if candidate is not None:
      if (self.candidate_since is None or now-self.candidate_last > MOTION_GAP_SECONDS or
          not candidate.near(self.candidate_region)):
        self.candidate_since = now
      self.candidate_last, self.candidate_region = now, candidate
      triggered = now-self.candidate_since >= MOTION_HOLD_SECONDS-1e-6
    elif now-self.candidate_last > MOTION_GAP_SECONDS:
      self.clear_candidate()
    if eye is None:
      return Motion()
    return Motion(True, eye.moving/current.size, eye.sum_x/eye.moving/(current.shape[1]-1),
                  eye.sum_y/eye.moving/(current.shape[0]-1), triggered)
