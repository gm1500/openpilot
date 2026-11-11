"""A weak inward line constrains a qualified curb; it never grants admission."""
import json
from pathlib import Path

import numpy as np
import pytest

from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.lane_policy import LanePolicy
from openpilot.selfdrive.modeld.road_edge_policy import DT, RoadEdgePolicy
from openpilot.selfdrive.modeld.tests.test_road_edge_policy import model_output, tick

X = np.asarray(ModelConstants.X_IDXS)


def inward_line(side=1, probability=0.2, gap=0.4, **kwargs):
  m = model_output(width=4.0, left_prob=0.99 if side else probability, right_prob=probability if side else 0.99, **kwargs)
  m['lane_lines'][0, 1 + side, :, 0] -= (-1 if side == 0 else 1) * gap
  return m


def update(p, m, count=1, speed=15.0, **kwargs):
  for _ in range(count):
    p.update(m, X, speed, **kwargs)


@pytest.mark.parametrize('side', [0, 1])
def test_inward_constraint_smooths_but_does_not_change_curb_width_or_admission(side):
  p = RoadEdgePolicy()
  m = inward_line(side)
  update(p, m, 14)
  e = p.edges[side]
  assert not e.available and e.inner_line.width is None
  update(p, m)
  assert e.available
  previous = 4.0
  for _ in range(80):
    actual = e.direction * (e.y - m['lane_lines'][0, 2 - side, :, 0])
    assert 0.0 <= previous - actual[0] <= 0.20 * DT + 1e-8
    np.testing.assert_allclose(actual, actual[0])
    previous = actual[0]
    update(p, m)
  assert previous == pytest.approx(3.6)
  assert e.width == pytest.approx(4.0) and e.memory_width == pytest.approx(4.0)
  assert p.paint_memory.width is None
  moved = inward_line(side, offset=0.2, curvature=0.0004)
  update(p, moved)
  np.testing.assert_allclose(e.y, moved['lane_lines'][0, 2 - side, :, 0] + e.direction * 3.6)


@pytest.mark.parametrize('problem', ['outside', 'too_far', 'narrow', 'crossing', 'bowed', 'speculative', 'no_anchor', 'nan', 'uncertain_edge'])
def test_unreliable_inner_line_cannot_constrain(problem):
  p = RoadEdgePolicy()
  m = inward_line()
  if problem == 'outside':
    m = inward_line(gap=-0.3)
  elif problem == 'too_far':
    m = inward_line(gap=0.9)
  elif problem == 'narrow':
    m = inward_line(gap=1.1)
  elif problem == 'crossing':
    m['lane_lines'][0, 2, 4, 0] += 0.5
  elif problem == 'bowed':
    m['lane_lines'][0, 2, :, 0] += 0.3 * np.sin(X / 10.0)
  elif problem == 'speculative':
    m = inward_line(probability=0.04)
  elif problem == 'no_anchor':
    m['lane_lines_prob'][0, 3] = 0.8
  elif problem == 'nan':
    m['lane_lines'][0, 2, 5, 0] = np.nan
  elif problem == 'uncertain_edge':
    m['road_edges_stds'][:] = 0.8
  update(p, m, 100)
  assert p.edges[1].inner_line.width is None
  if p.edges[1].available:
    np.testing.assert_allclose(p.edges[1].y, m['road_edges'][0, 1, :, 0])


def test_dips_pause_earning_and_disjoint_or_changing_guesses_do_not_qualify():
  p = RoadEdgePolicy()
  update(p, inward_line(), 10)
  update(p, inward_line(probability=0.07), 4)
  assert p.edges[1].inner_line.width is None
  update(p, inward_line(), 5)
  assert p.edges[1].inner_line.width == pytest.approx(3.6)
  p.reset()
  for _ in range(8):
    update(p, inward_line(), 8)
    update(p, inward_line(probability=0.07), 6)
  assert p.edges[1].inner_line.width is None
  p.reset()
  for i in range(100):
    update(p, inward_line(gap=0.2 if i % 2 else 0.6))
  assert p.edges[1].inner_line.width is None


@pytest.mark.parametrize('speed,hold_frames', [(0.0, 100), (25.0, 80)])
def test_confidence_loss_holds_bounded_width_then_releases_smoothly(speed, hold_frames):
  p = RoadEdgePolicy()
  update(p, inward_line(), 80, speed=speed)
  e = p.edges[1]
  previous = e.y.copy()
  missing = inward_line(probability=0.0)
  update(p, missing, hold_frames, speed=speed)
  assert e.inner_line.width == pytest.approx(3.6)
  np.testing.assert_allclose(e.y, previous)
  update(p, missing, speed=speed)
  assert e.inner_line.width is None
  assert e.y[0] - previous[0] == pytest.approx(0.2 * DT)
  previous = e.y.copy()
  for _ in range(50):
    update(p, missing, speed=speed)
    assert 0 <= e.y[0] - previous[0] <= 0.2 * DT + 1e-8
    previous = e.y.copy()
  assert e.inner_line.applied_width is None
  np.testing.assert_allclose(e.y, missing['road_edges'][0, 1, :, 0])


def test_raw_narrowing_wins_and_opening_hold_cannot_be_extended_by_weak_line():
  p = RoadEdgePolicy()
  update(p, inward_line(gap=0.15), 80)
  narrow = inward_line(gap=0.15)
  narrow['road_edges'][0, 1, :, 0] -= 0.25
  update(p, narrow)
  assert p.edges[1].y[0] == pytest.approx(narrow['road_edges'][0, 1, 0, 0])
  p.reset()
  update(p, inward_line(), 80, speed=10.0)
  opening = inward_line()
  opening['road_edges'][0, 1, :, 0] += 2.0
  update(p, opening, 40, speed=10.0)
  e = p.edges[1]
  assert e.available and e.held
  assert e.inner_line.age == pytest.approx(2.0)
  assert e.y[0] == pytest.approx(inward_line()['lane_lines'][0, 2, 0, 0])
  update(p, opening, speed=10.0)
  assert not e.available  # inner evidence never extends active road-edge hold
  assert e.memory_y is not None


@pytest.mark.parametrize('reset', ['blinker', 'off', 'lane_change', 'lost_anchor', 'stale', 'disengage', 'anchor_jump'])
def test_policy_resets_discard_the_constraint(reset):
  p = LanePolicy()
  tick(p, inward_line(), 80)
  assert p.road_edges.edges[1].inner_line.width is not None
  m = inward_line()
  options = {}
  count = 1
  if reset == 'blinker':
    options['blinkers_active'] = True
  elif reset == 'off':
    options['lane_policy_enabled'] = False
  elif reset == 'lane_change':
    m['desire_state'][0, 3] = 0.5
  elif reset == 'lost_anchor':
    m['lane_lines_prob'][0, 3] = 0.6
  elif reset == 'stale':
    tick(p, m, frame_time=10.0)
    options['frame_time'] = 11.0
  elif reset == 'disengage':
    options['lateral_active'] = False
    count = 100
  elif reset == 'anchor_jump':
    m['lane_lines'][0, 1, :, 0] += 0.3
    m['lane_lines'][0, 2, :, 0] += 0.3
    m['road_edges'][0, 1, :, 0] += 0.3
  tick(p, m, count, **options)
  assert p.road_edges.edges[1].inner_line.width is None


def test_two_confident_painted_lines_still_win():
  p = LanePolicy()
  tick(p, inward_line(), 80)
  assert p.active and p.edge_side == 1
  tick(p, inward_line(probability=0.99), 20)
  assert p.active and p.edge_side is None


def test_recorded_inner_line_narrows_curb_target_without_changing_availability():
  frames = json.loads((Path(__file__).parent / 'fixtures' / 'inward_line_curb.json').read_text())['frames']
  p, unconstrained = RoadEdgePolicy(), RoadEdgePolicy()
  constrained = []
  for stamp, speed, probs, std, opposite, line, edge in frames:
    m = model_output(left_prob=probs[0], right_prob=probs[1], edge_std=std)
    m['lane_lines'][0, 1, :, 0] = opposite
    m['lane_lines'][0, 2, :, 0] = line
    m['road_edges'][0, 1, :, 0] = edge
    update(p, m, speed=speed, frame_time=stamp)
    # Same observed curb and anchor, but no inward-line evidence.
    m['lane_lines'][0, 2, :, 0] = edge
    update(unconstrained, m, speed=speed, frame_time=stamp)
    e = p.edges[1]
    assert e.available == unconstrained.edges[1].available
    assert e.held == unconstrained.edges[1].held
    if e.available:
      shift = 0.5 * (e.width - (e.y[0] - opposite[0]))
      constrained.append(shift)
      assert 3 <= e.y[0] - opposite[0] <= e.width + 1e-8
  assert len(constrained) >= 80
  assert np.median(constrained[-40:]) > 0.10
