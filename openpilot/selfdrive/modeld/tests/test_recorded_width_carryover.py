"""Recorded geometry regressions through the complete LanePolicy caller.

Fixtures contain only relevant model geometry and control gates. Published
curvature is post-policy, so these tests establish eligibility and boundary
constraints, not counterfactual steering commands or closed-loop performance.
"""
import gzip
import json
from pathlib import Path

import numpy as np
import pytest

from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.lane_policy import LanePolicy
from openpilot.selfdrive.modeld.road_edge_policy import EDGE_ANCHOR_SUPPORT


X = np.asarray(ModelConstants.X_IDXS)
NEAR = (X >= 0.0) & (X <= 10.0)
HORIZON = (X >= 0.0) & (X <= 55.0)


def recorded_rows(name):
  with gzip.open(Path(__file__).parent / 'fixtures' / f'{name}.json.gz', 'rt') as stream:
    fixture = json.load(stream)
  assert fixture['format'] == 'lane_policy_geometry_v2'
  policy = LanePolicy()
  policy._mode = lambda reason: None
  previous = [None, None]
  rows = []
  for sec, stamp, speed, curvature, flags, probs, stds, desire, lines, edges, positions in fixture['frames']:
    enabled, lateral_active, steering_pressed, blinkers = flags
    plan = np.zeros((1, len(X), ModelConstants.PLAN_WIDTH))
    plan[0, :, :2] = positions
    lane_lines = np.zeros((1, 4, len(X), 1))
    road_edges = np.zeros((1, 2, len(X), 1))
    for side in (0, 1):
      lane_lines[0, side + 1, :, 0] = np.pad(lines[side], (0, len(X) - len(lines[side])), mode='edge')
      road_edges[0, side, :, 0] = np.pad(edges[side], (0, len(X) - len(edges[side])), mode='edge')
    probabilities = np.zeros((1, 8))
    probabilities[0, [3, 5]] = probs
    model = dict(plan=plan, lane_lines=lane_lines, road_edges=road_edges, lane_lines_prob=probabilities,
                 road_edges_stds=np.asarray(stds)[None, :, None, None], desire_state=np.asarray(desire)[None])
    policy.update(model, curvature, speed, blinkers, enabled, lateral_active, steering_pressed, stamp)
    snapshots = []
    for side, edge in enumerate(policy.road_edges.edges):
      opposite = lane_lines[0, 2 - side, :, 0]
      if edge.available:
        assert edge.y is not None and np.isfinite(edge.y).all(), sec
        assert np.min(edge.direction * edge.y[NEAR]) >= 0.3 - 1e-6, sec
        assert np.max(edge.direction * opposite[NEAR]) <= -0.3 + 1e-6, sec
        assert np.min(edge.direction * (edge.y - opposite)[HORIZON]) >= 3.0 - 1e-6, sec
        assert np.max(edge.direction * (edge.y - road_edges[0, side, :, 0])[HORIZON]) <= 1e-6, sec
      if edge.anchor_guided:
        assert probs[1 - side] >= 0.85 and stds[side] <= 1.0, sec
        assert edge.memory_support >= EDGE_ANCHOR_SUPPORT - 1e-6, sec
        assert not edge.memory_from_paint and not edge.anchor_blocked, sec
        prior = previous[side]
        if prior is None or not prior['anchor']:
          assert probs[1 - side] >= 0.92, sec
        if prior is not None and prior['memory'] == edge.memory_width:
          assert edge.memory_time >= prior['age'], sec  # An inferred boundary cannot renew itself.
      applied = None if edge.y is None else float(np.min(edge.direction * (edge.y - opposite)[HORIZON]))
      snapshots.append(dict(available=edge.available, memory=edge.memory_width, age=edge.memory_time,
                            anchor=edge.anchor_guided, guided=edge.memory_guided, blocked=edge.anchor_blocked,
                            anchor_width=edge.anchor_width, applied=applied, preview=edge.memory_y is not None))
    if not enabled or blinkers or sum(desire[3:5]) > 0.10:
      assert not policy.active and all(not edge['available'] and edge['memory'] is None for edge in snapshots), sec
    rows.append(dict(sec=sec, active=policy.active and enabled and lateral_active and not steering_pressed,
                     internal_active=policy.active, lat=lateral_active, steer=steering_pressed,
                     paint=policy.edge_side is None, edges=snapshots))
    previous = snapshots
  return rows


def nearest(rows, sec):
  return min(rows, key=lambda row: abs(row['sec'] - sec))


def test_recorded_intersection_retains_width_without_steering_through_lost_paint():
  rows = recorded_rows('intersection_width_carryover')
  before = nearest(rows, 181.55)['edges'][1]
  assert before['anchor'] and before['available']
  assert before['memory'] == pytest.approx(4.27061, abs=1e-4)
  assert before['applied'] == pytest.approx(3.80231, abs=1e-4)

  dip = nearest(rows, 182.15)['edges'][1]
  assert not dip['available'] and not dip['preview'] and dip['memory'] is not None
  paint = nearest(rows, 185.65)
  assert paint['paint'] and paint['edges'][1]['blocked'] and not paint['edges'][1]['available']
  assert paint['edges'][1]['memory'] is not None  # Short auxiliary paint suspends reuse, not private retention.
  assert all(not row['edges'][1]['available'] for row in rows if 187.80 <= row['sec'] < 191.79)

  # The returning edge stays qualified after the first early return. This
  # catches the old median mismatch and the later absolute-width mismatch.
  returning = [row for row in rows if 191.79 <= row['sec'] < 200.0]
  assert returning and all(row['edges'][1]['available'] for row in returning)
  assert nearest(rows, 191.80)['edges'][1]['guided']
  assert nearest(rows, 193.25)['edges'][1]['applied'] == pytest.approx(3.80231, abs=1e-4)
  assert all(row['active'] == (row['lat'] and not row['steer']) for row in returning)
  assert not nearest(rows, 200.01)['edges'][1]['available']  # Later inward change still rejects.
  assert nearest(rows, 202.11)['edges'][1]['memory'] is None  # Subsequent signal still resets history.


def test_recorded_observed_return_does_not_regress_after_remembered_reacquisition():
  rows = recorded_rows('remembered_width_return')
  # An earlier candidate reactivated from old memory, then reset fresh
  # qualification and lost 14 frames that the baseline kept active here.
  returning = [row for row in rows if 200.29 <= row['sec'] <= 201.05]
  assert len(returning) >= 14
  assert all(row['active'] and row['edges'][1]['available'] for row in returning)
