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
EDGE_ARM_PROB, EDGE_ARM_PAUSE_PROB = 0.92, 0.85
EDGE_ARM_PAUSE_TIME, EDGE_ARM_WINDOW = 0.25, 1.25
EDGE_RECOVERY_FRAMES = round(0.15 / DT)
EDGE_WIDTH_TIME = 1.0
EDGE_MAX_WIDTH_RATE = 0.20  # m/s; widening must not chase driveway openings
EDGE_MAX_SPREAD = 0.75     # m across the validated horizon
EDGE_MAX_HISTORY_SPREAD = 0.30
EDGE_HOLD_TIME, EDGE_HOLD_DISTANCE = 0.30, 5.0
EDGE_OPENING_TIME, EDGE_OPENING_DISTANCE = 1.25, 20.0
EDGE_CONFIDENCE_TIME, EDGE_CONFIDENCE_DISTANCE = 0.50, 8.0
EDGE_HOLD_STD = 1.0
EDGE_BRIDGE_RETAIN_PROB = 0.85
EDGE_MAX_ANCHOR_STEP = 0.20  # m/frame near the car; do not bridge a painted-line jump
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
    self.arm_age = self.arm_pause = 0.0
    self.previous_arm_anchor = None
    self.gap_time = self.gap_distance = 0.0
    self.gap_extendable = False
    self.gap_width = None
    self.previous_anchor = None

  def update(self, opposite, edge, opposite_prob, edge_std, x, speed, dt=DT):
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
    anchor = opposite[near]
    anchor_continuous = self.previous_anchor is not None and np.max(np.abs(anchor - self.previous_anchor)) <= EDGE_MAX_ANCHOR_STEP
    self.previous_anchor = anchor.copy()
    geometric = high <= EDGE_MAX_WIDTH and high - low <= EDGE_MAX_SPREAD
    # Compare successive observations, not the deliberately conservative output
    # boundary. Clamping that boundary inward must not poison curb continuity.
    compatible = self.observed_width is None or abs(median - self.observed_width) <= EDGE_MAX_HISTORY_SPREAD
    observed = geometric and compatible and edge_std <= (EDGE_RETAIN_STD if self.available else EDGE_ARM_STD)
    # A single returning frame must not replenish a long gap's time/distance
    # budget. Keep the width frozen until three consecutive observations agree.
    recovering = observed and self.available and self.gap_time > 0.0 and self.observed_frames + 1 < EDGE_RECOVERY_FRAMES
    if recovering:
      self.observed_frames += 1
    elif not observed:
      if self.gap_time == 0.0:
        self.gap_extendable = self.available and opposite_prob >= 0.92
        self.gap_width = self.width
      self.observed_frames = 0
    if recovering:
      observed = False
    if observed:
      self.observed_frames = min(self.observed_frames + 1, EDGE_ARM_FRAMES)
      self.gap_time = self.gap_distance = 0.0
      self.held = False
      self.gap_extendable = False
      if not self.available:
        # Only strong paint earns admission. Brief confidence-only dips may
        # pause it, provided the same paint/curb corridor remains observed.
        # Age the evidence as well as limiting each dip: scattered strong
        # frames must not eventually add up to a qualified boundary.
        self.arm_age += dt
        while self.history and self.arm_age - self.history[0][0] > EDGE_ARM_WINDOW:
          self.history.popleft()
        arm_anchor = opposite[(x >= 0.0) & (x <= 20.0)]
        arm_continuous = (self.previous_arm_anchor is not None and
                          np.max(np.abs(arm_anchor - self.previous_arm_anchor)) <= EDGE_MAX_ANCHOR_STEP + 1e-6)
        self.previous_arm_anchor = arm_anchor.copy()
        if not arm_continuous:
          self.history.clear()
        if opposite_prob < EDGE_ARM_PROB:
          self.arm_pause += dt
          same_width = bool(self.history) and abs(median - np.median([width for _, width in self.history])) <= EDGE_MAX_HISTORY_SPREAD
          if opposite_prob < EDGE_ARM_PAUSE_PROB or self.arm_pause > EDGE_ARM_PAUSE_TIME + 1e-6 or not same_width:
            self.history.clear()
          return
        self.arm_pause = 0.0
        self.history.append((self.arm_age, float(median)))
        if len(self.history) < EDGE_ARM_FRAMES:
          return
        history_low, history_median, history_high = np.percentile([width for _, width in self.history], (10, 50, 90))
        # One old width outlier should not erase a steady curb. Sustained
        # changes and a disagreeing latest width still cannot qualify.
        if history_high - history_low > EDGE_MAX_HISTORY_SPREAD or abs(median - history_median) > EDGE_MAX_HISTORY_SPREAD:
          return
        self.filtered_width = float(history_median)
        self.available = True
      self.observed_width = float(median)
      change = np.clip((median - self.filtered_width) * dt / (EDGE_WIDTH_TIME + dt),
                       -EDGE_MAX_WIDTH_RATE * dt, EDGE_MAX_WIDTH_RATE * dt)
      self.filtered_width += float(change)
      # Do not lag an observed narrowing. A virtual boundary must not extend
      # beyond a credible edge merely because its earlier width was greater.
      # The instantaneous inward cap is not fed back into the width filter.
      # Otherwise repeated noisy lows ratchet it down and reject good geometry.
      self.width = min(self.filtered_width, float(low), float(np.min(widths[near])))
    elif self.available:
      self.gap_time += dt
      self.gap_distance += max(speed, 0.0) * dt
      # Only an established curb with continuously strong, stable paint earns a
      # longer bridge. Outward openings freeze the old width; they never teach
      # a wider lane. Inward changes keep the original short budget and cap.
      self.gap_extendable &= opposite_prob >= EDGE_BRIDGE_RETAIN_PROB and anchor_continuous and edge_std <= EDGE_HOLD_STD
      # Compare like geometry: a valid curved corridor's lower width can be
      # below its median. Freeze the pre-gap cap so inward steps cannot ratchet
      # the extension reference inward from one held frame to the next.
      outward = min(low, np.min(widths[near])) >= self.gap_width - EDGE_MAX_HISTORY_SPREAD
      self.gap_extendable &= outward
      hold_time, hold_distance = EDGE_HOLD_TIME, EDGE_HOLD_DISTANCE
      if self.gap_extendable:
        hold_time, hold_distance = ((EDGE_OPENING_TIME, EDGE_OPENING_DISTANCE) if edge_std <= EDGE_RETAIN_STD else
                                    (EDGE_CONFIDENCE_TIME, EDGE_CONFIDENCE_DISTANCE))
      if self.gap_time > hold_time + 1e-6 or self.gap_distance > hold_distance:
        self.reset()
        return
      self.held = True
      # Even a held hypothesis must not extend beyond a newly observed inward
      # edge. This cap cannot turn a large jump into an accepted observation.
      self.width = min(self.width, float(low), float(np.min(widths[near])))
    else:
      self.observed_frames = 0
      self.history.clear()
      self.arm_age = self.arm_pause = 0.0
      self.previous_arm_anchor = None
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
    dt = DT
    if frame_time is not None:
      if self.last_time is not None:
        dt = frame_time - self.last_time
        if not 0.0 < dt <= 0.15:
          self.reset()
          dt = DT
      self.last_time = frame_time
    try:
      lines = np.asarray(output['lane_lines'][0, :, :, 0], dtype=float)
      edges = np.asarray(output['road_edges'][0, :, :, 0], dtype=float)
      stds = np.asarray(output['road_edges_stds'][0, :, 0, 0], dtype=float)
      probs = np.asarray(output['lane_lines_prob'][0, 1::2], dtype=float)
      if lines.shape != (4, len(x)) or edges.shape != (2, len(x)) or stds.shape != (2,) or probs.shape != (4,):
        raise ValueError('invalid boundary arrays')
      for side, candidate in enumerate(self.edges):
        candidate.update(lines[2 - side], edges[side], probs[2 - side], stds[side], x, speed, dt)
    except (KeyError, IndexError, TypeError, ValueError):
      self.reset()
