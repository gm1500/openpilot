"""Gyro-bounded alignment of the small Sentry luma images; no video is warped."""
from collections import deque
import math

import numpy as np


class GyroHistory:
  def __init__(self):
    self.samples = deque()

  def add(self, timestamp, rate):
    if not math.isfinite(timestamp) or len(rate) != 3 or not all(math.isfinite(v) for v in rate):
      return
    if self.samples and timestamp <= self.samples[-1][0]:
      return
    self.samples.append((timestamp, *rate))
    while self.samples and timestamp-self.samples[0][0] > 2.:
      self.samples.popleft()

  def angle(self, start, end):
    """Small net rotation between camera timestamps, or None without coverage."""
    if start is None or not 0 < end-start < 1. or len(self.samples) < 2:
      return None
    samples = np.asarray(self.samples)
    times, rates = samples[:, 0], samples[:, 1:]
    if start < times[0] or end > times[-1]+.03:
      return None
    left = max(0, int(np.searchsorted(times, start))-1)
    right = min(len(times), int(np.searchsorted(times, end))+1)
    if np.any(np.diff(times[left:right]) > .05):
      return None
    inner = times[(times > start) & (times < end)]
    knots = np.r_[start, inner, end]
    speed = np.stack([np.interp(knots, times, rates[:, axis]) for axis in range(3)], axis=1)
    rotation = np.sum(.5*(speed[1:]+speed[:-1])*np.diff(knots)[:, None], axis=0)
    return float(np.linalg.norm(rotation))


def sample_image(image, x, y):
  """Bilinear sampling, returning a mask for newly exposed image borders."""
  h, w = image.shape
  valid = (x >= 0) & (x < w-1) & (y >= 0) & (y < h-1)
  x, y = np.clip(x, 0, w-1), np.clip(y, 0, h-1)
  x0, y0 = x.astype(int), y.astype(int)
  x1, y1 = np.minimum(x0+1, w-1), np.minimum(y0+1, h-1)
  dx, dy = x-x0, y-y0
  value = (1-dy)*((1-dx)*image[y0, x0]+dx*image[y0, x1]) + dy*((1-dx)*image[y1, x0]+dx*image[y1, x1])
  return value, valid


def coordinates(shape, parameters):
  y, x = np.indices(shape, dtype=np.float32)
  xn, yn = (x-(shape[1]-1)/2)/shape[1], (y-(shape[0]-1)/2)/shape[0]
  basis = np.stack((xn, yn, np.ones_like(x)), axis=-1)
  return x+basis @ parameters[:3], y+basis @ parameters[3:], basis


def blur(image):
  padded = np.pad(image, 1, mode='edge')
  h, w = image.shape
  return sum(padded[dy:dy+h, dx:dx+w] for dy in range(3) for dx in range(3))/9.


def alignment(previous, current, angle):
  """Fit common frame motion only when fresh gyro data supports camera movement.

  Estimate each lens separately: the cabin is reversed and the wide lenses are
  distorted. Gyro bounds the displacement; distributed background agreement,
  rather than a guessed fisheye/mount transform, determines the image warp.
  """
  if angle is None or not math.isfinite(angle) or not .0001 <= angle <= .08:
    return None
  h, w = current.shape
  limit = min(5., max(.35, 2*max(h, w)*angle))
  previous, current = blur(previous), blur(current)
  parameters = np.zeros(6)
  # Two small pyramid levels handle pixel shifts without losing subpixel shake.
  for stride in (2, 1):
    source, target = previous[::stride, ::stride], current[::stride, ::stride]
    p = parameters/stride
    gy, gx = np.gradient(source)
    for _ in range(6):
      x, y, basis = coordinates(source.shape, p)
      warped, valid = sample_image(source, x, y)
      dx, _ = sample_image(gx, x, y)
      dy, _ = sample_image(gy, x, y)
      valid &= dx*dx+dy*dy > 1.
      if np.count_nonzero(valid) < 80:
        return None
      residual = target-warped
      residual -= np.median(residual[valid])  # exposure changes are not motion
      r = residual[valid]
      # Outlying local subjects must not pull the whole background with them.
      cutoff = max(1., 2.*float(np.median(np.abs(r))))
      weights = np.minimum(1., cutoff/np.maximum(np.abs(r), .001))
      jacobian = np.concatenate((basis*dx[..., None], basis*dy[..., None]), axis=-1)[valid]
      root = np.sqrt(weights)
      step, _, rank, _ = np.linalg.lstsq(jacobian*root[:, None], r*root, rcond=None)
      if rank < 6 or not np.isfinite(step).all():
        return None
      p += step
      if np.max(np.abs(step)) < .005:
        break
    parameters = p*stride
  x, y, _ = coordinates(current.shape, parameters)
  grid_y, grid_x = np.indices(current.shape)
  displacement = np.hypot(x-grid_x, y-grid_y)
  if displacement.max() > limit or displacement.max() < .015:
    return None
  warped, valid = sample_image(previous, x, y)
  before = current-previous
  after = current-warped
  before -= np.median(before[valid])
  after -= np.median(after[valid])
  gy, gx = np.gradient(previous)
  textured = valid & (gx*gx+gy*gy > 4.)
  # Demand improvement in spatially spread tiles, not just one moving person.
  supported = []
  available = 0
  for row in range(4):
    for col in range(4):
      tile = np.s_[row*h//4:(row+1)*h//4, col*w//4:(col+1)*w//4]
      mask = textured[tile]
      if np.count_nonzero(mask) < 12:
        continue
      available += 1
      old = np.median(np.abs(before[tile][mask]))
      new = np.median(np.abs(after[tile][mask]))
      if old > .15 and new < .8*old and new < 4.:
        supported.append((row, col))
  if (len(supported) < max(6, math.ceil(.6*available)) or
      len({r//2 for r, _ in supported}) < 2 or len({c//2 for _, c in supported}) < 2):
    return None
  return x, y


def compensate(previous, background, current, angle):
  mapping = alignment(previous, current, angle)
  if mapping is None:
    return previous, background
  previous, valid = sample_image(previous, *mapping)
  background, _ = sample_image(background, *mapping)
  # A border entering view has no prior observation and cannot be a subject.
  return np.where(valid, previous, current), np.where(valid, background, current)
