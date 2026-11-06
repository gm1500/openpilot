from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.lane_policy import DT, LanePolicy, anchor_correction, soften_correction
from openpilot.selfdrive.modeld.road_edge_policy import RoadEdgePolicy
from openpilot.selfdrive.ui.onroad.lane_policy_style import boundary_colors, EDGE_ORANGE, EDGE_YELLOW, PAINT_BLUE, PAINT_GREEN


def model_output(offset=0.3, width=3.6, left_prob=0.99, right_prob=0.1, curvature=0.0, edge_std=0.3):
  x = np.asarray(ModelConstants.X_IDXS)
  path = 0.5 * curvature * x ** 2
  plan = np.zeros((1, len(x), ModelConstants.PLAN_WIDTH))
  plan[0, :, 0], plan[0, :, 1] = x, path
  lines = np.zeros((1, 4, len(x), 2))
  lines[0, 1, :, 0], lines[0, 2, :, 0] = path + offset - width / 2, path + offset + width / 2
  probs = np.zeros((1, 8))
  probs[0, 3], probs[0, 5] = left_prob, right_prob
  return {'plan': plan, 'lane_lines': lines, 'lane_lines_prob': probs,
          'road_edges': lines[:, 1:3].copy(), 'road_edges_stds': np.full((1, 2, len(x), 2), edge_std),
          'desire_state': np.zeros((1, 8))}


def tick(policy, output, count=1, **kwargs):
  result = 0.0
  options = {'lane_policy_enabled': True}
  options.update(kwargs)
  for _ in range(count):
    result = policy.update(output, 0.001, 20.0, **options)
  return result


@pytest.mark.parametrize('side', [0, 1])
def test_stable_edge_and_one_strong_line_keep_original_centering_gain(side):
  p = LanePolicy()
  m = model_output(left_prob=0.99 if side else 0.1, right_prob=0.1 if side else 0.99)
  tick(p, m, 14)
  assert not p.road_edges.edges[side].available
  result = tick(p, m, 100)
  assert p.active and p.edge_side == side
  center = m['lane_lines'][0, 1:3, :, 0].mean(axis=0)
  expected = anchor_correction(m, center, np.asarray(ModelConstants.X_IDXS), 20.0)[0]
  assert result - 0.001 == pytest.approx(soften_correction(expected))
  assert p.width is None  # edges must not teach the painted-line width memory


@pytest.mark.parametrize('width', [2.8, 4.9, 7.2])
def test_width_bounds_reject_narrow_space_and_multiple_lanes(width):
  p = LanePolicy()
  tick(p, model_output(width=width), 80)
  assert not p.active and not any(e.available for e in p.road_edges.edges)


def test_two_edges_without_reliable_paint_do_not_arm():
  p = LanePolicy()
  tick(p, model_output(left_prob=0.2, right_prob=0.2), 80)
  assert not p.active


def test_width_filter_does_not_lag_vehicle_motion_or_curve_shape():
  p = RoadEdgePolicy()
  x = np.asarray(ModelConstants.X_IDXS)
  for _ in range(20):
    p.update(model_output(), x, 15.0)
  moved = model_output(offset=0.1, curvature=0.001)
  p.update(moved, x, 15.0)
  np.testing.assert_allclose(p.edges[1].y, moved['road_edges'][0, 1, :, 0], atol=1e-10)


def test_driveway_hold_is_bounded_and_narrowing_drops_immediately():
  p = LanePolicy()
  tick(p, model_output(), 40)
  opening = model_output()
  opening['road_edges'][0, 1, :, 0] += 2.0
  old_width = p.road_edges.edges[1].width
  tick(p, opening, 3)
  assert p.active and p.road_edges.edges[1].held
  assert p.road_edges.edges[1].width == old_width
  tick(p, opening, 6)
  assert not p.active
  tick(p, model_output(), 40)
  narrowed = model_output()
  narrowed['road_edges'][0, 1, :, 0] -= 1.0
  tick(p, narrowed)
  assert not p.active and not p.road_edges.edges[1].available


def test_noisy_edge_cannot_accumulate_disjoint_good_frames():
  p = LanePolicy()
  for _ in range(5):
    tick(p, model_output(), 10)
    tick(p, model_output(edge_std=0.9))
  assert not p.active


def test_unstable_width_resets_admission():
  p = LanePolicy()
  for _ in range(10):
    tick(p, model_output(width=3.4), 7)
    tick(p, model_output(width=4.1), 7)
  assert not p.active


@pytest.mark.parametrize('issue', ['blinker', 'lane_change', 'missing_anchor', 'nan', 'off'])
def test_existing_policy_gates_win(issue):
  p = LanePolicy()
  m = model_output()
  tick(p, m, 40)
  options = {}
  if issue == 'blinker':
    options['blinkers_active'] = True
  elif issue == 'lane_change':
    m['desire_state'][0, 3] = 0.5
  elif issue == 'missing_anchor':
    m['lane_lines_prob'][0, 3] = 0.1
  elif issue == 'nan':
    m['road_edges'][0, 1, 3, 0] = np.nan
  else:
    options['lane_policy_enabled'] = False
  assert tick(p, m, **options) == 0.001
  assert not p.active


def test_painted_recovery_does_not_switch_on_probability_chatter():
  p = LanePolicy()
  tick(p, model_output(), 40)
  for prob in [0.71, 0.69] * 30:
    tick(p, model_output(right_prob=prob))
    assert p.edge_side == 1
  tick(p, model_output(right_prob=0.99), 14)
  assert p.edge_side == 1
  tick(p, model_output(right_prob=0.99))
  assert p.edge_side is None and p.active


def test_time_gap_discards_edge_history():
  p = LanePolicy()
  m = model_output()
  for n in range(40):
    tick(p, m, frame_time=100 + n * DT)
  assert p.active
  tick(p, m, frame_time=105.0)
  assert not p.active


def test_return_to_paint_requires_consecutive_strong_frames():
  p = LanePolicy()
  tick(p, model_output(), 40)
  for _ in range(3):
    tick(p, model_output(right_prob=0.99), 10)
    tick(p, model_output(right_prob=0.1))
    assert p.edge_side == 1
  tick(p, model_output(right_prob=0.99), 15)
  assert p.edge_side is None


def test_curve_pause_exactly_three_frames_not_restarted_by_chatter():
  p = LanePolicy()
  p.motion_bias = 0.04
  p._guard_correction(0.0001, 0.001)
  for value in (-0.0001, 0.0001, -0.0001):
    assert p._guard_correction(value, 0.001) == (0.0, True)
  assert p.curve_release_frames == 0
  assert p._guard_correction(-0.0001, 0.001) == (-0.0001, False)
  assert p.motion_bias == 0.04


def test_small_sign_noise_does_not_retrigger_curve_pause():
  p = LanePolicy()
  p._guard_correction(0.0001, 0.001)
  for correction in [-0.000015, 0.000015] * 20:
    assert not p._guard_correction(correction, 0.001)[1]


def test_curve_pause_keeps_latest_direction_without_an_extra_pause():
  p = LanePolicy()
  p._guard_correction(0.0001, 0.001)
  for value in (-0.0001, 0.0001, 0.0001):
    assert p._guard_correction(value, 0.001) == (0.0, True)
  assert p._guard_correction(0.0001, 0.001) == (0.0001, False)


def test_two_real_lines_keep_centering_and_existing_temporary_one_line_hold():
  p = LanePolicy()
  m = model_output(right_prob=0.99)
  result = tick(p, m, 100)
  center = m['lane_lines'][0, 1:3, :, 0].mean(axis=0)
  expected = anchor_correction(m, center, np.asarray(ModelConstants.X_IDXS), 20.0)[0]
  assert result - 0.001 == pytest.approx(soften_correction(expected))
  assert p.edge_side is None and p.width == pytest.approx(3.6)
  del m['road_edges']
  m['lane_lines_prob'][0, 5] = 0.1
  tick(p, m, 10)
  assert p.active and p.holding_line and p.edge_side is None
  tick(p, m, 12)
  assert not p.active


def test_real_turn_reversal_releases_opposing_correction():
  p = LanePolicy()
  p._track_turn(0.001)
  p._track_turn(-0.001)
  assert p._guard_correction(0.0001, -0.001) == (0.0, True)


def test_published_filtered_geometry_and_side_sources():
  from openpilot.cereal import log
  p = LanePolicy()
  tick(p, model_output(), 40)
  msg = log.ModelDataV2.new_message()
  p.fill_state(msg.lanePolicy, True, True, False)
  assert msg.lanePolicy.active
  assert str(msg.lanePolicy.leftSource) == 'painted'
  assert str(msg.lanePolicy.rightSource) == 'roadEdge'
  np.testing.assert_allclose(msg.lanePolicy.edges[1].y, p.road_edges.edges[1].y)
  p.fill_state(msg.lanePolicy, True, True, True)
  assert not msg.lanePolicy.active


def test_orange_candidate_yellow_used_painted_blue_green():
  state = NS(valid=True, enabled=True, active=True, leftSource='painted', rightSource='roadEdge',
             edges=[NS(available=True), NS(available=True)])
  assert boundary_colors(state, 0, True) == (PAINT_GREEN, None)
  assert boundary_colors(state, 1, True) == (None, EDGE_YELLOW)
  assert boundary_colors(state, 0, False) == (PAINT_BLUE, None)
  assert boundary_colors(state, 1, False) == (None, EDGE_ORANGE)
  state.enabled = False
  assert boundary_colors(state, 1, True) == (None, None)
