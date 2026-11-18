#!/usr/bin/env python3
"""Offline, manually seeded feature continuity experiment. Never controls a lead.

Requires optional OpenCV. An ROI is a human-selected hypothesis, not an object
detector. Output cannot establish vehicle identity, an empty lane or safe braking.
"""

import argparse
from bisect import bisect_right
from dataclasses import asdict, dataclass, field
import json
import math

import cv2
import numpy as np


def rotation_homography(intrinsics, old_camera_to_new_camera):
  """Return K R K^-1. R rotates old camera coordinates into new coordinates.

  Use time-aligned camera calibration and rotation, not a guessed yaw sign or
  vehicle-frame rotation. Translation and object motion remain in the flow.
  """
  k, r = np.asarray(intrinsics, dtype=float), np.asarray(old_camera_to_new_camera, dtype=float)
  if (k.shape != (3, 3) or r.shape != (3, 3) or not np.isfinite(k).all() or not np.isfinite(r).all() or
      not np.allclose(r.T @ r, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(r), 1., atol=1e-5) or
      not math.isfinite(np.linalg.det(k)) or abs(np.linalg.det(k)) < 1e-9):
    raise ValueError('invalid camera calibration or rotation')
  return k @ r @ np.linalg.inv(k)


@dataclass(frozen=True)
class VisualEvidence:
  valid: bool
  reason: str
  age: float = 0.
  features: int = 0
  inlier_fraction: float = 0.
  median_fb_error: float | None = None
  visual_scale: float | None = None
  range_scale: float | None = None
  log_scale_residual: float | None = None
  rotation_compensated: bool = False
  # Deliberately invariant, including for strong feature continuity.
  can_ignore_lead: bool = field(default=False, init=False)


class RangeSamples:
  """Causal camera-time lookup. Times must already share the camera clock."""

  def __init__(self, observations):
    self.times = [float(row['time']) for row in observations]
    self.distances = [float(row['distance']) for row in observations]
    if (not all(math.isfinite(t) for t in self.times) or
        not all(math.isfinite(d) and d > 0. for d in self.distances) or
        any(b <= a for a, b in zip(self.times, self.times[1:], strict=False))):
      raise ValueError('range samples must be finite, positive and strictly time ordered')

  def at(self, timestamp):
    index = bisect_right(self.times, timestamp) - 1
    if not math.isfinite(timestamp) or index < 0 or timestamp - self.times[index] > .1 + 1e-9:
      return None
    return self.distances[index]


class LeadFeatureTrack:
  """Retain original features for a short observation; never reseed after loss."""

  MIN_FEATURES = 12
  MAX_FEATURES = 80
  MAX_GAP = .15
  MAX_AGE = 2.5

  def __init__(self):
    self.reset()

  def reset(self):
    self.previous = None
    self.points = None
    self.started = None
    self.last_time = None
    self.reference_range = None
    self.scale = 1.
    self.initial_feature_count = 0

  @staticmethod
  def _image(frame):
    return isinstance(frame, np.ndarray) and frame.dtype == np.uint8 and frame.ndim == 2 and min(frame.shape) >= 24

  def _reject(self, reason):
    self.reset()
    return VisualEvidence(False, reason)

  def acquire(self, frame, timestamp, roi, distance=None):
    """Acquire BEFORE a suspected split. roi=(left, top, width, height), pixels."""
    self.reset()
    if not self._image(frame) or not math.isfinite(timestamp):
      return self._reject('invalid_input')
    if distance is not None and (not math.isfinite(distance) or distance <= 0.):
      return self._reject('invalid_range')
    if len(roi) != 4 or not all(math.isfinite(v) for v in roi):
      return self._reject('invalid_roi')
    x, y, w, h = (int(v) for v in roi)
    if x < 0 or y < 0 or w < 20 or h < 15 or x + w > frame.shape[1] or y + h > frame.shape[0]:
      return self._reject('invalid_roi')
    mask = np.zeros_like(frame)
    mask[y:y + h, x:x + w] = 255
    points = cv2.goodFeaturesToTrack(frame, self.MAX_FEATURES, .03, 3., mask=mask, blockSize=3)
    if points is None or len(points) < self.MIN_FEATURES:
      return self._reject('insufficient_texture')
    self.previous, self.points = frame.copy(), points
    self.started = self.last_time = timestamp
    self.reference_range = distance
    self.initial_feature_count = len(points)
    return VisualEvidence(True, 'acquired', features=len(points), visual_scale=1.)

  def update(self, frame, timestamp, distance=None, camera_rotation=None):
    """Observe the next frame. camera_rotation is an optional K R K^-1 matrix.

    Range scale assumes an unchanged object face and orientation. Disagreement
    is diagnostic only: rotation, articulation and mixed ROI texture confound it.
    Any rejected update clears the track; reacquisition must be explicit.
    """
    if self.previous is None:
      return VisualEvidence(False, 'untracked')
    if not self._image(frame) or frame.shape != self.previous.shape or not math.isfinite(timestamp):
      return self._reject('invalid_input')
    dt, age = timestamp - self.last_time, timestamp - self.started
    if dt <= 0. or dt > self.MAX_GAP + 1e-9:
      return self._reject('timestamp_gap')
    if age > self.MAX_AGE + 1e-9:
      return self._reject('expired')
    if distance is not None and (not math.isfinite(distance) or distance <= 0.):
      return self._reject('invalid_range')
    predicted = self.points.copy()
    if camera_rotation is not None:
      h = np.asarray(camera_rotation, dtype=float)
      if (h.shape != (3, 3) or not np.isfinite(h).all() or not math.isfinite(np.linalg.det(h)) or
          abs(np.linalg.det(h)) < 1e-9):
        return self._reject('invalid_rotation')
      homogeneous = np.column_stack((self.points[:, 0], np.ones(len(self.points)))) @ h.T
      if not np.isfinite(homogeneous).all() or np.any(homogeneous[:, 2] <= 1e-6):
        return self._reject('invalid_rotation')
      predicted = (homogeneous[:, :2] / homogeneous[:, 2:]).astype(np.float32).reshape(-1, 1, 2)
    try:
      following, status, error = cv2.calcOpticalFlowPyrLK(
        self.previous, frame, self.points, predicted.copy(), winSize=(15, 15), maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 30, .01), flags=cv2.OPTFLOW_USE_INITIAL_FLOW)
      if following is None:
        return self._reject('flow_failed')
      backwards, backward_status, _ = cv2.calcOpticalFlowPyrLK(
        frame, self.previous, following, None, winSize=(15, 15), maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 30, .01))
      if backwards is None:
        return self._reject('flow_failed')
      fb = np.linalg.norm(backwards[:, 0] - self.points[:, 0], axis=1)
      inside = ((following[:, 0, 0] >= 0.) & (following[:, 0, 0] < frame.shape[1]) &
                (following[:, 0, 1] >= 0.) & (following[:, 0, 1] < frame.shape[0]))
      good = ((status[:, 0] == 1) & (backward_status[:, 0] == 1) & inside &
              np.isfinite(following[:, 0]).all(axis=1) & np.isfinite(fb) & (fb <= 1.25) & (error[:, 0] < 30.))
      count = int(good.sum())
      if count < self.MIN_FEATURES or count < .6 * self.initial_feature_count:
        return self._reject('feature_loss')
      affine, inliers = cv2.estimateAffinePartial2D(predicted[good, 0], following[good, 0], method=cv2.RANSAC,
                                                  ransacReprojThreshold=1.5, maxIters=1000, confidence=.99, refineIters=10)
      if affine is None or inliers is None or not np.isfinite(affine).all():
        return self._reject('inconsistent_flow')
      fraction = float(inliers.mean())
      keep = inliers[:, 0].astype(bool)
      if fraction < .7 or int(keep.sum()) < self.MIN_FEATURES:
        return self._reject('inconsistent_flow')
      step_scale = float(np.hypot(affine[0, 0], affine[1, 0]))
      # Gross jumps cannot be supported by this small-displacement tracker.
      if not .85 <= step_scale <= 1.15:
        return self._reject('scale_jump')
      self.scale *= step_scale
      self.points = following[good][keep].copy()
      self.previous, self.last_time = frame.copy(), timestamp
      expected = self.reference_range / distance if self.reference_range is not None and distance is not None else None
      if expected is not None and (not math.isfinite(expected) or expected <= 0.):
        return self._reject('invalid_range_ratio')
      residual = math.log(self.scale / expected) if expected is not None else None
      return VisualEvidence(True, 'continuity_only', age, len(self.points), fraction, float(np.median(fb[good][keep])),
                            self.scale, expected, residual, camera_rotation is not None)
    except cv2.error:
      return self._reject('opencv_failure')


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('video')
  parser.add_argument('--start', type=float, required=True, help='manual acquisition time, seconds')
  parser.add_argument('--duration', type=float, default=2., help='no more than 2.5 seconds')
  parser.add_argument('--roi', type=int, nargs=4, required=True, metavar=('X', 'Y', 'W', 'H'))
  parser.add_argument('--frame-times', help='JSON array of capture times, indexed by decoded frame; required for raw HEVC without PTS')
  ranges = parser.add_mutually_exclusive_group()
  ranges.add_argument('--range', type=float, dest='distance', help='initial model distance, optional')
  ranges.add_argument('--ranges', help='JSON [{"time": seconds, "distance": metres}], on the camera clock')
  args = parser.parse_args()
  if (not math.isfinite(args.start) or not math.isfinite(args.duration) or args.start < 0. or
      not 0. < args.duration <= LeadFeatureTrack.MAX_AGE):
    parser.error('start must be finite and nonnegative; duration must be in (0, 2.5] seconds')
  observations = RangeSamples(json.load(open(args.ranges))) if args.ranges else None
  frame_times = json.load(open(args.frame_times)) if args.frame_times else None
  if frame_times is not None and (not all(math.isfinite(t) and t >= 0. for t in frame_times) or
                                  any(b <= a for a, b in zip(frame_times, frame_times[1:], strict=False))):
    parser.error('frame capture times must be finite, nonnegative and strictly time ordered')
  video = cv2.VideoCapture(args.video)
  if not video.isOpened():
    video.release()
    parser.error('video could not be opened')
  if frame_times is None:
    video.set(cv2.CAP_PROP_POS_MSEC, args.start * 1000.)
  tracker, acquired = LeadFeatureTrack(), False
  frame_index = -1
  last_capture = None
  try:
    while True:
      ok, frame = video.read()
      if not ok:
        break
      frame_index += 1
      if frame_times is not None and frame_index >= len(frame_times):
        parser.error('missing capture timestamp for decoded frame')
      timestamp = frame_times[frame_index] if frame_times is not None else video.get(cv2.CAP_PROP_POS_MSEC) / 1000.
      if not math.isfinite(timestamp) or (last_capture is not None and timestamp <= last_capture):
        parser.error('video has no usable monotonic PTS; supply exact --frame-times')
      last_capture = timestamp
      if timestamp < args.start:
        continue
      if timestamp > args.start + args.duration:
        break
      gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
      distance = observations.at(timestamp) if observations is not None else (None if acquired else args.distance)
      result = tracker.update(gray, timestamp, distance) if acquired else tracker.acquire(gray, timestamp, args.roi, distance)
      acquired = True
      print(json.dumps(dict(time=timestamp, **asdict(result)), allow_nan=False))
      if not result.valid:
        break
  finally:
    video.release()
  if not acquired:
    parser.error('no decoded frames at the requested acquisition time')


if __name__ == '__main__':
  main()
