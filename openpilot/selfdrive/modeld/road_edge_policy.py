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
EDGE_STABLE_FRAMES, EDGE_STABLE_WINDOW = round(1.50 / DT), 2.0
EDGE_STABLE_HISTORY_SPREAD, EDGE_STABLE_SPREAD = 0.20, 0.50
EDGE_RECOVERY_FRAMES = round(0.15 / DT)
EDGE_WIDTH_TIME = 1.0
EDGE_MAX_WIDTH_RATE = 0.20  # m/s; widening must not chase driveway openings
EDGE_MAX_SPREAD = 0.75     # m across the validated horizon
EDGE_MAX_HISTORY_SPREAD = 0.30
EDGE_HOLD_TIME, EDGE_HOLD_DISTANCE = 0.30, 5.0
EDGE_OPENING_TIME, EDGE_OPENING_DISTANCE = 2.0, 30.0
EDGE_MEMORY_TIME, EDGE_MEMORY_DISTANCE = 5.0, 100.0
EDGE_MEMORY_FRAMES = round(0.30 / DT)
EDGE_DISENGAGE_TIME = 5.0
EDGE_CONFIDENCE_TIME, EDGE_CONFIDENCE_DISTANCE = 0.50, 8.0
EDGE_HOLD_STD = 1.0
EDGE_BRIDGE_RETAIN_PROB = 0.85
EDGE_MAX_ANCHOR_STEP = 0.20  # m/frame near the car; do not bridge a painted-line jump
EDGE_FIT_START, EDGE_FIT_END = 5.0, 55.0
INNER_LINE_ARM_PROB, INNER_LINE_PAUSE_PROB = 0.10, 0.05
INNER_LINE_MIN_GAP, INNER_LINE_MAX_GAP = 0.10, 0.80
INNER_LINE_MAX_SPREAD = 0.35


class InnerLineConstraint:
  """Stable inward lane evidence may narrow a curb corridor, never qualify it."""
  def __init__(self):
    self.reset()

  def reset(self):
    self.width = self.applied_width = None
    self.from_paint = False
    self.paint_returns = 0
    self.age = self.distance = self.clock = self.pause = 0.0
    self.history = deque(maxlen=EDGE_ARM_FRAMES)
    self.previous_anchor = None

  def update(self, candidate, opposite, line, edge, opposite_prob, line_prob, edge_std, x, speed, dt, allowed, paint_memory):
    near = (x >= 0.0) & (x <= 20.0)
    fit = (x >= EDGE_FIT_START) & (x <= EDGE_FIT_END)
    horizon = (x >= 0.0) & (x <= EDGE_FIT_END)
    anchor = opposite[near]
    if (not allowed or not np.isfinite(opposite).all() or not np.isfinite(opposite_prob) or
        opposite_prob < 0.70 or np.count_nonzero(fit) < 4 or
        (self.previous_anchor is not None and np.max(np.abs(anchor - self.previous_anchor)) > EDGE_MAX_ANCHOR_STEP + 1e-6)):
      self.reset()
      return
    self.previous_anchor = anchor.copy()
    self.clock += dt
    self.age += dt
    self.distance += max(speed, 0.0) * dt
    if opposite_prob < EDGE_BRIDGE_RETAIN_PROB:
      # The ordinary observed-curb policy may still retain this painted anchor.
      # Discard the learned constraint, but unwind its output at the normal
      # width rate instead of jumping outward to the curb on a confidence dip.
      self.width = None
      self.from_paint = False
      self.paint_returns = 0
      self.history.clear()
    if self.from_paint and (paint_memory.width is None or abs(self.width - paint_memory.width) > EDGE_STABLE_HISTORY_SPREAD):
      self.width = None
      self.from_paint = False
      self.paint_returns = 0
    direction = candidate.direction
    widths = direction * (line - opposite)
    gaps = direction * (edge - line)
    geometry = np.isfinite(widths).all() and np.isfinite(gaps).all() and np.isfinite(line_prob)
    median = float(np.median(widths[fit])) if geometry else 0.0
    if geometry:
      low, high = np.percentile(widths[fit], (10, 90))
      geometry = (np.min(widths[horizon]) >= EDGE_MIN_WIDTH and np.max(widths[horizon]) <= EDGE_MAX_WIDTH and
                  high - low <= INNER_LINE_MAX_SPREAD and np.min(direction * line[x <= 10.0]) >= 0.3 and
                  np.min(gaps[horizon]) >= INNER_LINE_MIN_GAP and np.max(gaps[horizon]) <= INNER_LINE_MAX_GAP)
    # Qualification must accompany fresh curb observations. A weak line cannot
    # renew either the edge's lease or this constraint across an opening.
    observed = (not candidate.held and candidate.observed_frames > 0 and
                np.isfinite(edge_std) and 0.0 <= edge_std <= EDGE_RETAIN_STD)
    while self.history and self.clock - self.history[0][0] > EDGE_ARM_WINDOW:
      self.history.popleft()
    earns = geometry and observed and opposite_prob >= EDGE_ARM_PROB and line_prob >= INNER_LINE_ARM_PROB
    paint_matches = (earns and paint_memory.width is not None and
                     np.max(np.abs(widths[horizon] - paint_memory.width)) <= EDGE_STABLE_HISTORY_SPREAD)
    self.paint_returns = self.paint_returns + 1 if paint_matches else 0
    if earns:
      self.pause = 0.0
      self.history.append((self.clock, median))
      if len(self.history) == self.history.maxlen:
        low, center, high = np.percentile([width for _, width in self.history], (10, 50, 90))
        if high - low <= EDGE_STABLE_HISTORY_SPREAD and abs(median - center) <= EDGE_STABLE_HISTORY_SPREAD:
          self.width = float(center)
          self.from_paint = False
          self.age = self.distance = 0.0
    else:
      self.pause += dt
      agrees = bool(self.history) and abs(median - np.median([width for _, width in self.history])) <= EDGE_STABLE_HISTORY_SPREAD
      if (not geometry or not observed or line_prob < INNER_LINE_PAUSE_PROB or not agrees or
          self.pause > EDGE_ARM_PAUSE_TIME + 1e-6):
        self.history.clear()
    if self.width is None and self.paint_returns >= EDGE_MEMORY_FRAMES:
      # A recent strong two-paint width can confirm the inward constraint with
      # six fresh agreeing observations. It does not seed or qualify the wider
      # curb. Inherit its age/distance; weak lines cannot renew painted memory.
      self.width = paint_memory.width
      self.age, self.distance = paint_memory.age, paint_memory.distance
      self.from_paint = True
    if self.age > EDGE_MEMORY_TIME + 1e-6 or self.distance > EDGE_MEMORY_DISTANCE + 1e-6:
      self.width = None
      self.from_paint = False
    raw_width = candidate.width if candidate.available else (
      float(np.min(direction * (candidate.memory_y - opposite))) if candidate.memory_y is not None else None)
    if raw_width is None:
      self.applied_width = None
      return
    if self.width is None and self.applied_width is None:
      return
    target = min(raw_width, self.width) if self.width is not None else raw_width
    previous = raw_width if self.applied_width is None else self.applied_width
    # Smooth both acquiring the inward bound and releasing it after expiry.
    # Raw curb narrowing still wins immediately. Never feed the cap into curb
    # qualification, filtered width, opening detection or paint-width learning.
    self.applied_width = min(raw_width, previous + float(np.clip(target - previous, -EDGE_MAX_WIDTH_RATE * dt, EDGE_MAX_WIDTH_RATE * dt)))
    if self.width is None and self.applied_width >= raw_width:
      self.applied_width = None
      return
    if candidate.available:
      candidate.y = opposite + direction * self.applied_width
    if candidate.memory_y is not None:
      memory_width = float(np.min(direction * (candidate.memory_y - opposite)))
      candidate.memory_y = opposite + direction * min(memory_width, self.applied_width)


class PaintWidthMemory:
  """Recent two-paint evidence for edge acquisition; never a steering boundary."""
  def __init__(self):
    self.reset()

  def reset(self):
    self.width = None
    self.age = self.distance = 0.0
    self.clock = self.pause = 0.0
    self.history = deque(maxlen=EDGE_ARM_FRAMES)
    self.previous = None
    self.continuous = np.zeros(2, dtype=bool)
    self.used = [False, False]

  def update(self, lines, probs, x, speed, dt, allowed):
    if not allowed:
      self.reset()
      return
    self.clock += dt
    while self.history and self.clock - self.history[0][0] > EDGE_ARM_WINDOW:
      self.history.popleft()
    anchor = (x >= 0.0) & (x <= 20.0)
    near = (x >= 0.0) & (x <= 10.0)
    horizon = (x >= 0.0) & (x <= EDGE_FIT_END)
    fit = (x >= EDGE_FIT_START) & (x <= EDGE_FIT_END)
    if not np.any(near) or np.count_nonzero(fit) < 4:
      self.reset()
      return
    shape = lines[:, anchor]
    self.continuous = (np.zeros(2, dtype=bool) if self.previous is None else
                       np.max(np.abs(shape - self.previous), axis=1) <= EDGE_MAX_ANCHOR_STEP + 1e-6)
    self.previous = shape.copy()
    finite = np.isfinite(lines).all(axis=1) & np.isfinite(probs)
    if self.width is not None:
      self.age += dt
      self.distance += max(speed, 0.0) * dt
      retained = finite & (probs >= EDGE_BRIDGE_RETAIN_PROB)
      if (self.age > EDGE_MEMORY_TIME + 1e-6 or self.distance > EDGE_MEMORY_DISTANCE + 1e-6 or
          not np.any(retained) or not np.all(self.continuous[retained])):
        self.width = None
        self.history.clear()
    strong = np.all(finite) and np.min(probs) >= EDGE_ARM_PROB
    if not np.all(finite):
      self.history.clear()
      return
    widths = lines[1] - lines[0]
    low, median, high = np.percentile(widths[fit], (10, 50, 90))
    valid = (np.min(widths[horizon]) >= EDGE_MIN_WIDTH and np.max(widths[horizon]) <= EDGE_MAX_WIDTH and
             high - low <= EDGE_STABLE_SPREAD and np.max(lines[0, near]) <= -0.3 and np.min(lines[1, near]) >= 0.3)
    if not strong:
      self.pause += dt
      agrees = bool(self.history) and abs(median - np.median([width for _, width in self.history])) <= EDGE_STABLE_HISTORY_SPREAD
      if (not valid or not np.all(self.continuous) or np.min(probs) < EDGE_ARM_PAUSE_PROB or
          not agrees or self.pause > EDGE_ARM_PAUSE_TIME + 1e-6):
        self.history.clear()
      return  # Paused observations cannot teach width or refresh its lease.
    self.pause = 0.0
    if not valid:
      self.width = None
      self.history.clear()
      return
    if self.width is not None and abs(median - self.width) > EDGE_STABLE_HISTORY_SPREAD:
      # Do not let a slowly filtered old lane width survive a confident change.
      self.width = None
      self.history.clear()
    if not np.all(self.continuous):
      self.history.clear()
    self.history.append((self.clock, float(median)))
    if len(self.history) == self.history.maxlen:
      low, center, high = np.percentile([width for _, width in self.history], (10, 50, 90))
      if high - low <= EDGE_STABLE_HISTORY_SPREAD and abs(median - center) <= EDGE_STABLE_HISTORY_SPREAD:
        self.width = float(center)
        self.age = self.distance = 0.0
        self.used = [False, False]

  def seed(self, candidate, side, opposite, missing_prob, opposite_prob, edge, edge_std, x, speed, dt):
    if (self.width is None or self.used[side] or candidate.available or candidate.memory_width is not None or
        not np.isfinite(missing_prob) or missing_prob >= 0.70 or not np.isfinite(opposite_prob) or
        opposite_prob < EDGE_ARM_PROB or not self.continuous[1 - side] or
        not np.isfinite(edge_std) or not 0.0 <= edge_std <= EDGE_RETAIN_STD):
      return
    fit = (x >= EDGE_FIT_START) & (x <= EDGE_FIT_END)
    horizon = (x >= 0.0) & (x <= EDGE_FIT_END)
    widths = candidate.direction * (edge - opposite)
    if (not np.isfinite(widths).all() or abs(np.median(widths[fit]) - self.width) > EDGE_STABLE_HISTORY_SPREAD or
        np.max(np.abs(widths[horizon] - self.width)) > EDGE_MAX_HISTORY_SPREAD):
      return
    # Transfer the source's age, not a fresh lease. Each confirmed paint window
    # may seed a side once; rejected observations cannot repeatedly revive it.
    candidate.memory_width = candidate.memory_median = self.width
    candidate.memory_from_paint = True
    candidate.memory_time = max(0.0, self.age - dt)
    candidate.memory_distance = max(0.0, self.distance - max(speed, 0.0) * dt)
    self.used[side] = True


class EdgeBoundary:
  def __init__(self):
    self.inner_line = InnerLineConstraint()
    self.reset()

  def forget_memory(self):
    self.memory_width = self.memory_median = None
    self.memory_from_paint = False
    self.memory_y = None
    self.memory_time = self.memory_distance = 0.0
    self.memory_returns = 0

  def reset(self, keep_memory=False):
    if not keep_memory:
      self.forget_memory()
      self.inner_line.reset()
      self.previous_anchor = None
    self.memory_guided = False
    self.memory_returns = 0
    self.available = self.held = False
    self.width = None
    self.filtered_width = self.observed_width = None
    self.observed_frames = 0
    self.y = None
    self.history = deque(maxlen=EDGE_ARM_FRAMES)
    self.stable_history = deque(maxlen=EDGE_STABLE_FRAMES)
    self.arm_age = 0.0
    self.arm_pauses = [0.0, 0.0]
    self.previous_arm_anchor = None
    self.gap_time = self.gap_distance = 0.0
    self.gap_extendable = False
    self.gap_width = None

  def _clear_qualification(self):
    self.history.clear()
    self.stable_history.clear()
    self.arm_age = 0.0
    self.arm_pauses = [0.0, 0.0]
    self.previous_arm_anchor = None
    self.memory_returns = 0

  def _qualify(self, opposite, opposite_prob, edge_std, x, median, spread, dt):
    # Strong paint is required on every earned observation. Moderate edge
    # uncertainty needs twice the evidence and a tighter, stable corridor.
    self.arm_age += dt
    arm_anchor = opposite[(x >= 0.0) & (x <= 20.0)]
    continuous = (self.previous_arm_anchor is not None and
                  np.max(np.abs(arm_anchor - self.previous_arm_anchor)) <= EDGE_MAX_ANCHOR_STEP + 1e-6)
    self.previous_arm_anchor = arm_anchor.copy()
    tracks = ((self.history, EDGE_ARM_STD, EDGE_ARM_WINDOW, EDGE_MAX_HISTORY_SPREAD, True),
              (self.stable_history, EDGE_RETAIN_STD, EDGE_STABLE_WINDOW, EDGE_STABLE_HISTORY_SPREAD,
               spread <= EDGE_STABLE_SPREAD))
    for index, (history, std_limit, window, tolerance, shape_ok) in enumerate(tracks):
      while history and self.arm_age - history[0][0] > window:
        history.popleft()
      if not continuous or not shape_ok:
        history.clear()
      earns = opposite_prob >= EDGE_ARM_PROB and edge_std <= std_limit and shape_ok
      if not earns:
        self.arm_pauses[index] += dt
        same_width = bool(history) and abs(median - np.median([width for _, width in history])) <= tolerance
        if (opposite_prob < EDGE_ARM_PAUSE_PROB or self.arm_pauses[index] > EDGE_ARM_PAUSE_TIME + 1e-6 or
            not same_width or not shape_ok):
          history.clear()
        continue
      self.arm_pauses[index] = 0.0
      history.append((self.arm_age, float(median)))
      if len(history) < history.maxlen:
        continue
      low, center, high = np.percentile([width for _, width in history], (10, 50, 90))
      if high - low <= tolerance and abs(median - center) <= tolerance:
        return float(center)
    return None

  def update(self, opposite, edge, opposite_prob, edge_std, x, speed, dt=DT):
    if self.memory_width is not None:
      self.memory_time += dt
      self.memory_distance += max(speed, 0.0) * dt
      if self.memory_time > EDGE_MEMORY_TIME + 1e-6 or self.memory_distance > EDGE_MEMORY_DISTANCE + 1e-6:
        self.forget_memory()
        if self.memory_guided:
          self.reset()
          return
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
    anchor_continuous = self.previous_anchor is not None and np.max(np.abs(anchor - self.previous_anchor)) <= EDGE_MAX_ANCHOR_STEP + 1e-6
    anchor_jump = self.previous_anchor is not None and not anchor_continuous
    self.previous_anchor = anchor.copy()
    if opposite_prob < EDGE_BRIDGE_RETAIN_PROB:
      self.forget_memory()
      if self.held or self.memory_guided:
        self.reset()
        return
    geometric = high <= EDGE_MAX_WIDTH and high - low <= EDGE_MAX_SPREAD
    # Compare observations, not the conservative output cap.
    compatible = self.observed_width is None or abs(median - self.observed_width) <= EDGE_MAX_HISTORY_SPREAD
    observed = geometric and compatible and edge_std <= EDGE_RETAIN_STD
    if self.memory_from_paint and observed:
      horizon = (x >= 0.0) & (x <= EDGE_FIT_END)
      if np.max(np.abs(widths[horizon] - self.memory_median)) > EDGE_MAX_HISTORY_SPREAD:
        # Matching average width cannot hide a peeling-away or bowed boundary.
        self.reset()
        return
    if anchor_jump:
      # A simultaneous motion of fresh paint and curb need not drop observed
      # centering, but old width memory must never bridge an anchor jump.
      if not observed or self.held or self.memory_guided or not self.available:
        self.reset()
        return
      self.forget_memory()
      self._clear_qualification()
    if self.memory_width is not None and min(low, np.min(widths[near])) < self.memory_width - EDGE_MAX_HISTORY_SPREAD:
      # A currently observed, compatible edge can cap inward immediately. An
      # unsupported inward jump must discard the prior and qualify again.
      if not observed or self.held or self.memory_guided:
        self.reset()
        return
      self.forget_memory()
    self.memory_y = (opposite + direction * min(self.memory_width, float(low), float(np.min(widths[near])))
                     if self.memory_width is not None else None)
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
    if not observed:
      self._clear_qualification()
    if observed:
      self.observed_frames = min(self.observed_frames + 1, EDGE_ARM_FRAMES)
      self.gap_time = self.gap_distance = 0.0
      self.held = False
      self.gap_extendable = False
      # Continue full qualification even after memory-assisted reacquisition.
      # Six agreeing observations may reuse the prior; only a full qualification
      # window can renew its age or teach a different width.
      qualified_width = self._qualify(opposite, opposite_prob, edge_std, x, median, high - low, dt)
      memory_matches = (self.memory_width is not None and opposite_prob >= EDGE_ARM_PROB and
                        high - low <= EDGE_STABLE_SPREAD and abs(median - self.memory_median) <= EDGE_STABLE_HISTORY_SPREAD)
      self.memory_returns = self.memory_returns + 1 if memory_matches else 0
      if self.memory_guided and not memory_matches:
        self.reset()
        return
      if not self.available:
        if qualified_width is not None:
          self.filtered_width = qualified_width
        elif self.memory_returns >= EDGE_MEMORY_FRAMES:
          self.filtered_width = self.memory_width
          self.memory_guided = True
        else:
          return
        self.available = True
      if qualified_width is not None:
        self.memory_guided = False
      self.observed_width = self.memory_median if self.memory_guided else float(median)
      change = np.clip((median - self.filtered_width) * dt / (EDGE_WIDTH_TIME + dt),
                       -EDGE_MAX_WIDTH_RATE * dt, EDGE_MAX_WIDTH_RATE * dt)
      if not self.memory_guided:
        self.filtered_width += float(change)
      # Do not lag an observed narrowing. A virtual boundary must not extend
      # beyond a credible edge merely because its earlier width was greater.
      # The instantaneous inward cap is not fed back into the width filter.
      # Otherwise repeated noisy lows ratchet it down and reject good geometry.
      self.width = min(self.filtered_width, float(low), float(np.min(widths[near])))
      if qualified_width is not None:
        self.memory_width, self.memory_median = self.width, float(median)
        self.memory_from_paint = False
        self.memory_time = self.memory_distance = 0.0
        self.memory_y = opposite + direction * self.memory_width
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
      if self.gap_time > hold_time + 1e-6 or self.gap_distance > hold_distance + 1e-6:
        self.reset(keep_memory=True)
        return
      self.held = True
      # Even a held hypothesis must not extend beyond a newly observed inward
      # edge. This cap cannot turn a large jump into an accepted observation.
      self.width = min(self.width, float(low), float(np.min(widths[near])))
    else:
      self.observed_frames = 0
      self._clear_qualification()
      return
    self.y = opposite + direction * self.width


class RoadEdgePolicy:
  def __init__(self):
    self.edges = [EdgeBoundary(), EdgeBoundary()]
    self.paint_memory = PaintWidthMemory()
    self.edges[0].direction, self.edges[1].direction = -1, 1
    self.last_time = None
    self.inactive_time = 0.0

  def reset(self):
    for edge in self.edges:
      edge.reset()
    self.paint_memory.reset()
    self.last_time = None
    self.inactive_time = 0.0

  def update(self, output, x, speed, frame_time=None, lateral_active=True):
    dt = DT
    if frame_time is not None:
      if self.last_time is not None:
        dt = frame_time - self.last_time
        if not 0.0 < dt <= 0.15:
          self.reset()
          dt = DT
      self.last_time = frame_time
    self.inactive_time = 0.0 if lateral_active else self.inactive_time + dt
    try:
      lines = np.asarray(output['lane_lines'][0, :, :, 0], dtype=float)
      edges = np.asarray(output['road_edges'][0, :, :, 0], dtype=float)
      stds = np.asarray(output['road_edges_stds'][0, :, 0, 0], dtype=float)
      probs = np.asarray(output['lane_lines_prob'][0, 1::2], dtype=float)
      if lines.shape != (4, len(x)) or edges.shape != (2, len(x)) or stds.shape != (2,) or probs.shape != (4,):
        raise ValueError('invalid boundary arrays')
      self.paint_memory.update(lines[1:3], probs[1:3], x, speed, dt, self.inactive_time < EDGE_DISENGAGE_TIME - 1e-6)
      for side, candidate in enumerate(self.edges):
        if candidate.memory_from_paint and (self.paint_memory.width is None or
                                           abs(candidate.memory_median - self.paint_memory.width) > EDGE_STABLE_HISTORY_SPREAD):
          candidate.reset()
        self.paint_memory.seed(candidate, side, lines[2 - side], probs[1 + side], probs[2 - side], edges[side], stds[side], x, speed, dt)
        if self.inactive_time >= EDGE_DISENGAGE_TIME - 1e-6:
          if candidate.held or candidate.memory_guided:
            candidate.reset()
          candidate.forget_memory()
        candidate.update(lines[2 - side], edges[side], probs[2 - side], stds[side], x, speed, dt)
        if self.inactive_time >= EDGE_DISENGAGE_TIME - 1e-6:
          candidate.forget_memory()
        candidate.inner_line.update(candidate, lines[2 - side], lines[1 + side], edges[side], probs[2 - side], probs[1 + side],
                                    stds[side], x, speed, dt, self.inactive_time < EDGE_DISENGAGE_TIME - 1e-6, self.paint_memory)
    except (KeyError, IndexError, TypeError, ValueError):
      self.reset()
