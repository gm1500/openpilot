"""Paint-derived edge memory: transfer, geometric rejection and expiration."""

import numpy as np
import pytest

from openpilot.selfdrive.modeld.lane_policy import LanePolicy
from openpilot.selfdrive.modeld.tests.test_road_edge_policy import model_output
from openpilot.selfdrive.modeld.constants import ModelConstants


def tick(p, m, n=1, **kw):
  options = {'lane_policy_enabled': True, 'lateral_active': True}
  options.update(kw)
  for _ in range(n):
    p.update(m, 0.001, 10.0, **options)


def learned():
  p = LanePolicy()
  tick(p, model_output(right_prob=0.99, edge_std=2), 30)
  assert p.road_edges.paint_memory.width == pytest.approx(3.6)
  assert not p.road_edges.edges[1].available
  return p


def test_matching_return_uses_paint_width_and_follows_live_curve():
  p = learned()
  m = model_output(curvature=0.0005, edge_std=0.6)
  tick(p, m, 5)
  assert not p.road_edges.edges[1].available
  tick(p, m)
  e = p.road_edges.edges[1]
  assert p.active and p.edge_side == 1 and e.memory_guided
  np.testing.assert_allclose(e.y, m['lane_lines'][0, 1, :, 0] + 3.6)
  assert e.memory_time >= 0.3 - 1e-6


@pytest.mark.parametrize(
  'issue',
  ['shoulder', 'median_only', 'shape_after_seed', 'narrow', 'wide_lane', 'lost_anchor', 'jump_anchor', 'expired', 'lane_change', 'blinker', 'disengaged'],
)
def test_bad_transitions_do_not_get_accelerated_acquisition(issue):
  p = learned()
  m = model_output(edge_std=0.6)
  if issue == 'shoulder':
    m['road_edges'][0, 1, :, 0] += 0.6
  elif issue == 'median_only':
    # Median remains the learned width, but far edge peels away.
    m['road_edges'][0, 1, 17, 0] += 0.4
  elif issue == 'shape_after_seed':
    tick(p, m, 1)
    m['road_edges'][0, 1, 17, 0] += 0.4
  elif issue == 'narrow':
    m['road_edges'][0, 1, :, 0] -= 0.7
  elif issue == 'wide_lane':
    p = LanePolicy()
    tick(p, model_output(width=4.8, right_prob=0.99, edge_std=2), 30)
    assert p.road_edges.paint_memory.width is None
  elif issue == 'lost_anchor':
    m['lane_lines_prob'][0, 3] = 0.1
  elif issue == 'jump_anchor':
    m['lane_lines'][0, 1, :, 0] -= 0.4
  elif issue == 'expired':
    tick(p, model_output(edge_std=2), 101)
  elif issue == 'lane_change':
    q = model_output()
    q['desire_state'][0, 3] = 0.5
    tick(p, q)
  elif issue == 'blinker':
    tick(p, m, blinkers_active=True)
  elif issue == 'disengaged':
    tick(p, model_output(right_prob=0.99, edge_std=2), 101, lateral_active=False)
  tick(p, m, 6)
  assert not p.road_edges.edges[1].available


def test_raw_existing_width_can_lag_a_new_lane_width():
  p = LanePolicy()
  tick(p, model_output(width=3.6, right_prob=0.99, edge_std=2), 30)
  tick(p, model_output(width=4.2, right_prob=0.99, edge_std=2), 20)
  assert 3.65 < p.width < 3.67  # ~0.54 m behind the newly observed width
  print('width_after_one_second', p.width)


def test_widening_or_shoulders_do_not_extend_saved_paint_width():
  p = learned()
  m = model_output(edge_std=2)
  tick(p, m, 99)
  assert p.road_edges.paint_memory.width is not None
  tick(p, m, 2)
  assert p.road_edges.paint_memory.width is None


@pytest.mark.parametrize('side', [0, 1])
def test_paint_memory_uses_current_curve_and_publishes_yellow_while_assisting(side):
  from openpilot.cereal import log
  from openpilot.selfdrive.ui.onroad.lane_policy_style import active_edge, boundary_colors, EDGE_YELLOW, PAINT_GREEN

  p = learned()
  m = model_output(left_prob=0.99 if side else 0.1, right_prob=0.1 if side else 0.99, curvature=0.0005, edge_std=0.6)
  tick(p, m, 6)
  e = p.road_edges.edges[side]
  assert e.memory_from_paint and e.memory_guided and p.edge_side == side
  np.testing.assert_allclose(e.y, m['lane_lines'][0, 2 - side, :, 0] + (-1 if side == 0 else 1) * e.width)
  state = log.ModelDataV2.new_message().lanePolicy
  p.fill_state(state, True, True, False)
  assert state.edges[side].held and active_edge(state, side, True)
  assert boundary_colors(state, side, True) == (None, EDGE_YELLOW)
  assert not active_edge(state, side, False)
  tick(p, m, 24)
  p.fill_state(state, True, True, False)
  assert not e.memory_from_paint and not e.memory_guided
  assert boundary_colors(state, side, True) == (None, PAINT_GREEN)


def test_paint_memory_requires_shape_agreement_after_engagement_too():
  p = learned()
  m = model_output(edge_std=0.6)
  tick(p, m, 6)
  assert p.road_edges.edges[1].memory_guided
  m['road_edges'][0, 1, 17, 0] += 0.4
  tick(p, m)
  assert not p.road_edges.edges[1].available
  assert p.road_edges.edges[1].memory_width is None
  # Same painted reference cannot repeatedly re-seed a rejected edge.
  m['road_edges'][0, 1, 17, 0] -= 0.4
  tick(p, m, 6)
  assert not p.road_edges.edges[1].available


def test_seed_inherits_age_and_expires_during_reacquisition():
  p = learned()
  tick(p, model_output(edge_std=2), 97)
  assert p.road_edges.paint_memory.age == pytest.approx(4.85)
  tick(p, model_output(edge_std=0.6), 6)
  assert p.road_edges.paint_memory.width is None
  assert p.road_edges.edges[1].memory_width is None
  assert not p.road_edges.edges[1].available


def test_painted_reference_expires_at_distance_limit():
  p = learned()
  m = model_output(edge_std=2)
  for _ in range(66):
    p.update(m, 0.001, 30.0, lane_policy_enabled=True, lateral_active=True)
  assert p.road_edges.paint_memory.width is not None
  p.update(m, 0.001, 30.0, lane_policy_enabled=True, lateral_active=True)
  assert p.road_edges.paint_memory.width is None


def test_confident_width_change_discards_old_paint_prior_before_relearning():
  p = learned()
  m = model_output(width=4.2, right_prob=0.99, edge_std=2)
  tick(p, m)
  assert p.road_edges.paint_memory.width is None
  tick(p, m, 14)
  assert p.road_edges.paint_memory.width == pytest.approx(4.2)
  assert p.width < 3.7  # original one-line filter remains independent


@pytest.mark.parametrize('probability,width', [(0.91, 3.6), (0.99, 2.8), (0.99, 4.8)])
def test_unqualified_painted_lanes_cannot_teach_edge_memory(probability, width):
  p = LanePolicy()
  tick(p, model_output(width=width, right_prob=probability, edge_std=2), 40)
  assert p.road_edges.paint_memory.width is None


def test_frame_gap_clears_both_paint_reference_and_seeded_edge():
  p = learned()
  m = model_output(edge_std=0.6)
  tick(p, m, 6, frame_time=None)
  tick(p, m, frame_time=10.0)
  tick(p, m, frame_time=10.5)
  assert p.road_edges.paint_memory.width is None
  assert p.road_edges.edges[1].memory_width is None


def test_recorded_wider_edge_is_not_accelerated_by_narrower_painted_width():
  import json
  from pathlib import Path

  fixture = json.loads((Path(__file__).parent / 'fixtures' / 'paint_width_edge_mismatch.json').read_text())
  p = LanePolicy()
  saw_reference = saw_mismatch = False
  for stamp, speed, prob, missing_prob, std, blinkers, desire, paint, missing, edge in fixture['frames']:
    m = model_output(left_prob=missing_prob, right_prob=prob, edge_std=2)
    m['road_edges_stds'][0, 0, :, 0] = std
    for target, values in [(m['lane_lines'][0, 2, :, 0], paint), (m['lane_lines'][0, 1, :, 0], missing), (m['road_edges'][0, 0, :, 0], edge)]:
      target[:] = np.pad(values, (0, 33 - len(values)), mode='edge')
    m['desire_state'][0, 3] = desire
    p.update(m, 0.001, speed, lane_policy_enabled=True, lateral_active=True, blinkers_active=blinkers, frame_time=stamp)
    prior = p.road_edges.paint_memory.width
    if prior is not None:
      saw_reference = True
      assert 3.3 < prior < 3.5
    if 5.9 <= stamp <= 6.2:
      saw_mismatch = True
      assert prior is not None and prob >= 0.92 and missing_prob < 0.7
      x = np.asarray(ModelConstants.X_IDXS)
      widths = m['lane_lines'][0, 2, :, 0] - m['road_edges'][0, 0, :, 0]
      assert np.median(widths[(x >= 5) & (x <= 55)]) - prior > 0.8
      assert not p.road_edges.edges[0].available and not p.road_edges.edges[0].memory_from_paint
  assert saw_reference and saw_mismatch
