"""Qualify a road edge as one missing lane boundary, anchored to real paint.

Filter lane width rather than ego-frame edge coordinates: steering and curve
entry must move the boundary immediately with the observed opposite lane line.
These are lane hypotheses, not vehicle-footprint or obstacle-clearance checks.
"""
from collections import deque

import numpy as np

from openpilot.selfdrive.modeld.constants import ModelConstants

DT = 1.0 / ModelConstants.MODEL_RUN_FREQ
EDGE_MIN_WIDTH, EDGE_MAX_WIDTH = 3.0, 4.6
EDGE_ARM_STD, EDGE_RETAIN_STD = 0.50, 0.65
EDGE_ARM_FRAMES = round(0.75 / DT)
EDGE_RECOVERY_FRAMES = round(0.15 / DT)
EDGE_WIDTH_TIME = 1.0
EDGE_MAX_WIDTH_RATE = 0.20  # m/s; widening must not chase driveway openings
EDGE_MAX_SPREAD = 0.75     # m across the validated horizon
EDGE_MAX_HISTORY_SPREAD = 0.30
EDGE_HOLD_TIME, EDGE_HOLD_DISTANCE = 0.30, 5.0
EDGE_FIT_START, EDGE_FIT_END = 5.0, 55.0


class EdgeBoundary:
  def __init__(self):
    self.reset()

  def reset(self):
    self.available = self.held = False
    self.width = None
    self.filtered_width = self.observed_width = None
    self.observed_frames = 0
    self.y = None
    self.history = deque(maxlen=EDGE_ARM_FRAMES)
    self.gap_time = self.gap_distance = 0.0

  def update(self, opposite, edge, opposite_prob, edge_std, x, speed):
    # Both bounds must bracket the car; a nearby parallel road is not our lane.
    fit = (x >= EDGE_FIT_START) & (x <= EDGE_FIT_END)
    near = (x >= 0.0) & (x <= 10.0)
    direction = self.direction
    valid = (np.isfinite(opposite).all() and np.isfinite(edge).all() and
             np.isfinite(opposite_prob) and np.isfinite(edge_std) and edge_std >= 0 and
             opposite_prob >= 0.70 and np.count_nonzero(fit) >= 4)
    if not valid:
      self.reset()
      return
    widths = direction * (edge - opposite)
    low, median, high = np.percentile(widths[fit], (10, 50, 90))
    # A narrowed/inverted corridor, lost real line, or an implausible near edge
    # invalidates immediately. Only brief outward openings/uncertainty may coast.
    hard_invalid = (low < EDGE_MIN_WIDTH or np.min(widths[near]) < EDGE_MIN_WIDTH or
                    np.max(direction * opposite[near]) > -0.3 or np.min(direction * edge[near]) < 0.3)
    if hard_invalid:
      self.reset()
      return
    geometric = high <= EDGE_MAX_WIDTH and high - low <= EDGE_MAX_SPREAD
    # Compare successive observations, not the deliberately conservative output
    # boundary. Clamping that boundary inward must not poison curb continuity.
    compatible = self.observed_width is None or abs(median - self.observed_width) <= EDGE_MAX_HISTORY_SPREAD
    observed = geometric and compatible and edge_std <= (EDGE_RETAIN_STD if self.available else EDGE_ARM_STD)
    if observed:
      self.observed_frames = min(self.observed_frames + 1, EDGE_ARM_FRAMES)
      self.gap_time = self.gap_distance = 0.0
      self.held = False
      if not self.available:
        if opposite_prob < 0.92:
          self.history.clear()
          return
        self.history.append(float(median))
        if max(self.history) - min(self.history) > EDGE_MAX_HISTORY_SPREAD:
          self.history.clear()
          self.history.append(float(median))
        if len(self.history) < EDGE_ARM_FRAMES:
          return
        self.filtered_width = float(np.median(self.history))
        self.available = True
      self.observed_width = float(median)
      change = np.clip((median - self.filtered_width) * DT / (EDGE_WIDTH_TIME + DT),
                       -EDGE_MAX_WIDTH_RATE * DT, EDGE_MAX_WIDTH_RATE * DT)
      self.filtered_width += float(change)
      # Do not lag an observed narrowing. A virtual boundary must not extend
      # beyond a credible edge merely because its earlier width was greater.
      # The instantaneous inward cap is not fed back into the width filter.
      # Otherwise repeated noisy lows ratchet it down and reject good geometry.
      self.width = min(self.filtered_width, float(low), float(np.min(widths[near])))
    elif self.available:
      self.observed_frames = 0
      self.gap_time += DT
      self.gap_distance += max(speed, 0.0) * DT
      if self.gap_time > EDGE_HOLD_TIME or self.gap_distance > EDGE_HOLD_DISTANCE:
        self.reset()
        return
      self.held = True
      # Even a held hypothesis must not extend beyond a newly observed inward
      # edge. This cap cannot turn a large jump into an accepted observation.
      self.width = min(self.width, float(low), float(np.min(widths[near])))
    else:
      self.observed_frames = 0
      self.history.clear()
      return
    self.y = opposite + direction * self.width


class RoadEdgePolicy:
  def __init__(self):
    self.edges = [EdgeBoundary(), EdgeBoundary()]
    self.edges[0].direction, self.edges[1].direction = -1, 1
    self.last_time = None

  def reset(self):
    for edge in self.edges:
      edge.reset()
    self.last_time = None

  def update(self, output, x, speed, frame_time=None):
    if frame_time is not None:
      if self.last_time is not None and not 0.0 < frame_time - self.last_time <= 0.15:
        self.reset()
      self.last_time = frame_time
    try:
      lines = np.asarray(output['lane_lines'][0, :, :, 0], dtype=float)
      edges = np.asarray(output['road_edges'][0, :, :, 0], dtype=float)
      stds = np.asarray(output['road_edges_stds'][0, :, 0, 0], dtype=float)
      probs = np.asarray(output['lane_lines_prob'][0, 1::2], dtype=float)
      if lines.shape != (4, len(x)) or edges.shape != (2, len(x)) or stds.shape != (2,) or probs.shape != (4,):
        raise ValueError('invalid boundary arrays')
      for side, candidate in enumerate(self.edges):
        candidate.update(lines[2 - side], edges[side], probs[2 - side], stds[side], x, speed)
    except (KeyError, IndexError, TypeError, ValueError):
      self.reset()
