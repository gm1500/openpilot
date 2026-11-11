"""Confidence pauses and transfer of trusted paint width into an inward cap."""
import json
from pathlib import Path

import numpy as np
import pytest

from openpilot.selfdrive.modeld.lane_policy import LanePolicy
from openpilot.selfdrive.modeld.road_edge_policy import DT, PaintWidthMemory, RoadEdgePolicy
from openpilot.selfdrive.modeld.tests.test_road_edge_policy import model_output, tick
from openpilot.selfdrive.modeld.tests.test_road_edge_inner_line import X, inward_line, update


def learn(memory, count, probability=0.99, width=3.6, dt=DT):
  m = model_output(width=width, left_prob=probability, right_prob=probability)
  for _ in range(count):
    memory.update(m['lane_lines'][0, 1:3, :, 0], np.array([probability, probability]), X, 10.0, dt, True)


def test_paint_confidence_pause_preserves_evidence_but_earns_no_observations():
  p = PaintWidthMemory()
  learn(p, 10)
  learn(p, 4, probability=0.89)
  assert len(p.history) == 10 and p.width is None
  learn(p, 4)
  assert p.width is None
  learn(p, 1)
  assert p.width == pytest.approx(3.6)
  learn(p, 4, probability=0.89)
  assert p.age == pytest.approx(0.2)  # weak frames cannot renew memory


@pytest.mark.parametrize('issue', ['long_dip', 'weak_dip', 'changed_width', 'sparse', 'stale'])
def test_paused_paint_evidence_cannot_accumulate_without_continuity(issue):
  p = PaintWidthMemory()
  learn(p, 10)
  if issue == 'long_dip':
    learn(p, 6, probability=0.89)
  elif issue == 'weak_dip':
    learn(p, 1, probability=0.84)
  elif issue == 'changed_width':
    learn(p, 1, probability=0.89, width=4.0)
  elif issue == 'sparse':
    for _ in range(10):
      learn(p, 4, probability=0.89)
      learn(p, 1)
  elif issue == 'stale':
    learn(p, 1, probability=0.89, dt=1.3)
  learn(p, 4)
  assert p.width is None


def test_nonfinite_paint_cannot_teach_width():
  p = RoadEdgePolicy()
  m = model_output(right_prob=0.99)
  m['lane_lines'][0, 2, 3, 0] = np.nan
  update(p, m, 30)
  assert p.paint_memory.width is None


def prepared(side=1):
  p = RoadEdgePolicy()
  update(p, inward_line(side, probability=0.99, edge_std=2), 30, speed=10.0)
  update(p, inward_line(side, probability=0.04), 30, speed=10.0)
  assert p.paint_memory.width == pytest.approx(3.6)
  assert p.edges[side].available and p.edges[side].inner_line.width is None
  return p


@pytest.mark.parametrize('side', [0, 1])
def test_recent_paint_width_confirms_inward_constraint_after_six_fresh_matches(side):
  p = prepared(side)
  m = inward_line(side)
  update(p, m, 5, speed=10.0)
  e = p.edges[side]
  assert e.inner_line.width is None
  update(p, m, speed=10.0)
  assert e.inner_line.from_paint and e.inner_line.width == pytest.approx(3.6)
  assert e.inner_line.age == pytest.approx(p.paint_memory.age)
  assert e.inner_line.distance == pytest.approx(p.paint_memory.distance)
  actual = e.direction * (e.y - m['lane_lines'][0, 2 - side, :, 0])
  np.testing.assert_allclose(actual, 4.0 - 0.20 * DT)
  # Only a full independent inward qualification can renew the reference.
  update(p, m, 9, speed=10.0)
  assert not e.inner_line.from_paint and e.inner_line.age == 0
  assert p.paint_memory.age == pytest.approx(2.25)


@pytest.mark.parametrize('issue', ['different_width', 'far_shape', 'outside_curb', 'weak_line', 'weak_anchor', 'expired', 'changed_reference'])
def test_paint_width_does_not_excuse_current_inward_geometry(issue):
  p = prepared()
  m = inward_line()
  if issue == 'different_width':
    m['lane_lines'][0, 2, :, 0] += 0.25
  elif issue == 'far_shape':
    m['lane_lines'][0, 2, 17, 0] += 0.25
  elif issue == 'outside_curb':
    m['lane_lines'][0, 2, :, 0] += 0.5
  elif issue == 'weak_line':
    m['lane_lines_prob'][0, 5] = 0.09
  elif issue == 'weak_anchor':
    m['lane_lines_prob'][0, 3] = 0.91
  elif issue == 'expired':
    update(p, inward_line(probability=0.04), 68, speed=10.0)
  elif issue == 'changed_reference':
    strong = inward_line(probability=0.99)
    strong['lane_lines'][0, 2, :, 0] += 0.5
    update(p, strong, speed=10.0)
    assert p.paint_memory.width is None
  update(p, m, 6, speed=10.0)
  assert not p.edges[1].inner_line.from_paint
  assert p.edges[1].inner_line.width is None


def test_transfer_does_not_keep_expired_paint_width_alive():
  p = prepared()
  update(p, inward_line(), 6, speed=10.0)
  e = p.edges[1]
  assert e.inner_line.from_paint
  update(p, inward_line(probability=0.04), 64, speed=10.0)
  assert p.paint_memory.width is not None
  assert e.inner_line.from_paint
  update(p, inward_line(probability=0.04), speed=10.0)
  assert p.paint_memory.width is None
  assert e.inner_line.width is None and not e.inner_line.from_paint


def test_changed_paint_source_invalidates_transferred_constraint_and_unwinds_it():
  p = prepared()
  update(p, inward_line(), 12, speed=10.0)
  e = p.edges[1]
  assert e.inner_line.from_paint
  previous = e.y.copy()
  changed = inward_line(probability=0.99)
  changed['lane_lines'][0, 2, :, 0] += 0.5
  update(p, changed, speed=10.0)
  assert p.paint_memory.width is None and e.inner_line.width is None
  assert not e.inner_line.from_paint
  np.testing.assert_allclose(e.y - previous, 0.20 * DT)


def test_matching_inner_line_does_not_admit_an_unqualified_curb():
  p = RoadEdgePolicy()
  update(p, inward_line(probability=0.99, edge_std=2), 30)
  update(p, inward_line(edge_std=0.6), 6)
  assert p.edges[1].inner_line.from_paint
  assert not p.edges[1].available  # still needs the curb's 30-frame qualification


@pytest.mark.parametrize('side', [0, 1])
def test_confidence_dip_unwinds_inward_output_without_a_curbward_jump(side):
  p = RoadEdgePolicy()
  m = inward_line(side)
  update(p, m, 80)
  e = p.edges[side]
  previous = e.y.copy()
  anchor_index = 3 if side else 5
  m['lane_lines_prob'][0, anchor_index] = 0.8
  for _ in range(10):
    update(p, m)
    assert e.available and e.inner_line.width is None
    np.testing.assert_allclose(e.direction * (e.y - previous), 0.2 * DT)
    previous = e.y.copy()
  # Loss below ordinary observed-paint retention still releases the whole edge.
  m['lane_lines_prob'][0, anchor_index] = 0.69
  update(p, m)
  assert not e.available and e.inner_line.applied_width is None


def test_paint_assisted_constraint_uses_memory_yellow_and_active_side_thickness():
  from openpilot.cereal import log
  from openpilot.selfdrive.ui.onroad.lane_policy_style import active_edge, boundary_colors, EDGE_YELLOW

  p = LanePolicy()
  tick(p, inward_line(probability=0.99, edge_std=2), 30)
  tick(p, inward_line(probability=0.04), 30)
  tick(p, inward_line(), 6)
  assert p.active and p.edge_side == 1 and p.road_edges.edges[1].inner_line.from_paint
  state = log.ModelDataV2.new_message().lanePolicy
  p.fill_state(state, True, True, False)
  assert state.edges[1].held and active_edge(state, 1, True)
  assert boundary_colors(state, 1, True) == (None, EDGE_YELLOW)
  assert not active_edge(state, 0, True) and not active_edge(state, 1, False)


def test_recorded_painted_width_learning_survives_short_confidence_dips():
  frames = json.loads((Path(__file__).parent / 'fixtures' / 'paint_width_confidence_dips.json').read_text())['frames']
  p = PaintWidthMemory()
  first = None
  previous = None
  for stamp, speed, probs, left, right in frames:
    p.update(np.array([left, right]), np.array(probs), X, speed, DT if previous is None else stamp - previous, True)
    previous = stamp
    if p.width is not None and first is None:
      first = stamp
  assert first is not None and first < 2.3
  assert p.width is not None and 3.7 < p.width < 4.1
