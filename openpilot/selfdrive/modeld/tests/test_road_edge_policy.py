import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.lane_policy import DT, LanePolicy, anchor_correction, soften_correction
from openpilot.selfdrive.modeld.road_edge_policy import RoadEdgePolicy
from openpilot.selfdrive.ui.onroad.lane_policy_style import active_edge, boundary_colors, EDGE_YELLOW, PAINT_BLUE, PAINT_GREEN


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


def test_noisy_inward_caps_do_not_ratchet_width_or_disqualify_a_steady_curb():
  p = RoadEdgePolicy()
  x = np.asarray(ModelConstants.X_IDXS)
  normal = model_output(width=4.0)
  for _ in range(20):
    p.update(normal, x, 15.0)
  dip = model_output(width=4.0)
  dip['road_edges'][0, 1, :8, 0] -= 0.35
  for _ in range(20):
    p.update(dip, x, 15.0)
    assert p.edges[1].available and not p.edges[1].held
    assert p.edges[1].width <= 3.65 + 1e-8
    p.update(normal, x, 15.0)
    assert p.edges[1].available and not p.edges[1].held
    assert p.edges[1].width == pytest.approx(4.0)


def test_recorded_steady_curb_keeps_continuity_after_qualification():
  # Five seconds of boundary observations; no GPS, images or vehicle identifiers.
  frames = json.loads((Path(__file__).parent / 'fixtures' / 'steady_curb.json').read_text())['frames']
  p = RoadEdgePolicy()
  x = np.asarray(ModelConstants.X_IDXS)
  for index, (stamp, speed, probability, std, paint, edge) in enumerate(frames):
    m = model_output(left_prob=probability, edge_std=std)
    m['lane_lines'][0, 1, :, 0] = np.pad(paint, (0, 33 - len(paint)), mode='edge')
    m['road_edges'][0, 1, :, 0] = np.pad(edge, (0, 33 - len(edge)), mode='edge')
    p.update(m, x, speed, stamp)
    if index >= 19:
      assert p.edges[1].available and not p.edges[1].held, stamp


def test_short_hold_pauses_arming_and_requires_fresh_recovery_before_engaging():
  p = LanePolicy()
  # Both painted lines are retained but cannot arm centering. The edge earns
  # its own qualification before the weak same-side paint disappears.
  tick(p, model_output(right_prob=0.8), 20)
  assert p.road_edges.edges[1].available and not p.active
  progress = p.arm_time
  tick(p, model_output(edge_std=0.9), 2)
  assert not p.active and p.arm_time == progress
  tick(p, model_output())
  assert not p.active  # qualification is shared, but recovery is not settled
  tick(p, model_output(edge_std=0.9))
  assert not p.active
  tick(p, model_output(), 2)
  assert not p.active
  tick(p, model_output())
  assert p.active


@pytest.mark.parametrize('probability', [0.69, 0.90])
def test_held_edge_cannot_start_centering_without_fresh_strong_paint(probability):
  p = LanePolicy()
  tick(p, model_output(right_prob=0.8), 20)
  tick(p, model_output(left_prob=probability, edge_std=0.9))
  assert not p.active and p.arm_time == 0.0


def test_held_boundary_also_moves_inside_an_observed_narrowing():
  p = LanePolicy()
  tick(p, model_output(width=4.0), 40)
  narrowed = model_output(width=4.0)
  narrowed['road_edges'][0, 1, :, 0] -= 0.5
  tick(p, narrowed)
  assert p.road_edges.edges[1].held
  assert p.road_edges.edges[1].width <= 3.5 + 1e-8
  tick(p, narrowed, 8)
  assert not p.active  # the large change must be qualified again


def test_driveway_hold_is_bounded_and_narrowing_drops_immediately():
  p = LanePolicy()
  tick(p, model_output(), 40)
  opening = model_output()
  opening['road_edges'][0, 1, :, 0] += 2.0
  old_width = p.road_edges.edges[1].width
  tick(p, opening, 3)
  assert p.active and p.road_edges.edges[1].held
  assert p.road_edges.edges[1].width == old_width
  tick(p, opening, 30)
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


def test_yellow_candidate_green_active_only_virtual_side_thickens():
  state = NS(valid=True, enabled=True, active=True, leftSource='painted', rightSource='roadEdge',
             edges=[NS(available=True), NS(available=True)])
  assert boundary_colors(state, 0, True) == (PAINT_GREEN, None)
  assert boundary_colors(state, 1, True) == (None, PAINT_GREEN)
  assert active_edge(state, 1, True) and not active_edge(state, 0, True)
  assert boundary_colors(state, 0, False) == (PAINT_BLUE, None)
  assert boundary_colors(state, 1, False) == (None, EDGE_YELLOW)
  assert not active_edge(state, 1, False)
  state.enabled = False
  assert boundary_colors(state, 1, True) == (None, None)


@pytest.mark.parametrize('side', [0, 1])
def test_established_opening_freezes_width_and_follows_current_paint(side):
  p = RoadEdgePolicy()
  x = np.asarray(ModelConstants.X_IDXS)
  m = model_output(left_prob=0.99 if side else 0.1, right_prob=0.1 if side else 0.99)
  for _ in range(30):
    p.update(m, x, 10.0)
  width = p.edges[side].width
  opening = model_output(offset=0.2, curvature=0.001,
                         left_prob=0.99 if side else 0.1, right_prob=0.1 if side else 0.99)
  opening['road_edges'][0, side, :, 0] += 2.0 * (-1 if side == 0 else 1)
  for _ in range(20):
    p.update(opening, x, 10.0)
    assert p.edges[side].available and p.edges[side].held
    assert p.edges[side].width == pytest.approx(width)
    expected = opening['lane_lines'][0, 2 - side, :, 0] + (-1 if side == 0 else 1) * width
    np.testing.assert_allclose(p.edges[side].y, expected)
  # The returning curb must agree for three consecutive frames before learning.
  for _ in range(2):
    p.update(m, x, 10.0)
    assert p.edges[side].held
  p.update(m, x, 10.0)
  assert p.edges[side].available and not p.edges[side].held


@pytest.mark.parametrize('speed,frames', [(0.0, 25), (10.0, 25), (30.0, 13)])
def test_opening_bridge_has_time_and_distance_limits(speed, frames):
  p = RoadEdgePolicy()
  x = np.asarray(ModelConstants.X_IDXS)
  for _ in range(30):
    p.update(model_output(), x, speed)
  opening = model_output()
  opening['road_edges'][0, 1, :, 0] += 2.0
  for _ in range(frames):
    p.update(opening, x, speed)
    assert p.edges[1].available
  p.update(opening, x, speed)
  assert not p.edges[1].available


@pytest.mark.parametrize('issue', ['weak_paint', 'lost_paint', 'jumping_paint', 'narrowing', 'uncertain_edge'])
def test_long_bridge_stops_when_support_deteriorates(issue):
  p = RoadEdgePolicy()
  x = np.asarray(ModelConstants.X_IDXS)
  for _ in range(30):
    p.update(model_output(), x, 10.0)
  opening = model_output()
  opening['road_edges'][0, 1, :, 0] += 2.0
  for _ in range(12):
    p.update(opening, x, 10.0)
  assert p.edges[1].available
  if issue in ('weak_paint', 'lost_paint'):
    opening['lane_lines_prob'][0, 3] = 0.84 if issue == 'weak_paint' else 0.69
  elif issue == 'jumping_paint':
    opening['lane_lines'][0, 1, :, 0] -= 0.4
  elif issue == 'narrowing':
    opening['road_edges'][0, 1, :, 0] -= 3.0
  else:
    opening['road_edges_stds'][0, 1, :, 0] = 1.1
  p.update(opening, x, 10.0)
  assert not p.edges[1].available


def test_intermittent_good_frames_cannot_refresh_an_opening_forever():
  p = RoadEdgePolicy()
  x = np.asarray(ModelConstants.X_IDXS)
  for _ in range(30):
    p.update(model_output(), x, 10.0)
  opening = model_output()
  opening['road_edges'][0, 1, :, 0] += 2.0
  for n in range(26):
    p.update(model_output() if n % 4 in (2, 3) else opening, x, 10.0)
  assert not p.edges[1].available


def test_confidence_dip_has_a_shorter_bridge_than_visible_outward_opening():
  p = RoadEdgePolicy()
  x = np.asarray(ModelConstants.X_IDXS)
  for _ in range(30):
    p.update(model_output(), x, 10.0)
  for _ in range(10):
    p.update(model_output(edge_std=0.9), x, 10.0)
    assert p.edges[1].available and p.edges[1].held
  p.update(model_output(edge_std=0.9), x, 10.0)
  assert not p.edges[1].available


def test_robust_width_window_ignores_one_old_outlier_but_requires_agreeing_latest_width():
  p = RoadEdgePolicy()
  x = np.asarray(ModelConstants.X_IDXS)
  for n in range(15):
    p.update(model_output(width=4.0 if n == 5 else 3.6), x, 10.0)
  assert p.edges[1].available and p.edges[1].width == pytest.approx(3.6)
  p.reset()
  for n in range(15):
    p.update(model_output(width=4.0 if n == 14 else 3.6), x, 10.0)
  assert not p.edges[1].available


def test_opening_cannot_qualify_without_an_observed_curb():
  p = LanePolicy()
  opening = model_output()
  opening['road_edges'][0, 1, :, 0] += 2.0
  tick(p, opening, 100)
  assert not p.active and not p.road_edges.edges[1].available


@pytest.mark.parametrize('name,start,end', [('curb_opening', 1.0, 5.65), ('curb_width_noise', 2.1, 11.0),
                                         ('curb_opening_night', 1.0, 3.7)])
def test_recorded_opening_and_noisy_curb_retain_qualified_boundary(name, start, end):
  frames = json.loads((Path(__file__).parent / 'fixtures' / f'{name}.json').read_text())['frames']
  p = RoadEdgePolicy()
  x = np.asarray(ModelConstants.X_IDXS)
  for stamp, speed, probability, std, paint, edge in frames:
    m = model_output(left_prob=probability, edge_std=std)
    m['lane_lines'][0, 1, :, 0] = np.pad(paint, (0, 33 - len(paint)), mode='edge')
    m['road_edges'][0, 1, :, 0] = np.pad(edge, (0, 33 - len(edge)), mode='edge')
    p.update(m, x, speed, stamp)
    if start <= stamp <= end:
      assert p.edges[1].available, stamp
    if name == 'curb_opening' and stamp >= 5.7:
      assert not p.edges[1].available, stamp  # paint support deteriorates after the opening


def test_long_bridge_cannot_earn_initial_engagement():
  p = LanePolicy()
  tick(p, model_output(right_prob=0.8), 20)
  assert p.road_edges.edges[1].available and not p.active
  progress = p.arm_time
  opening = model_output()
  opening['road_edges'][0, 1, :, 0] += 2.0
  # tick's fixed 20 m/s reaches 15 m here, still inside both bridge limits.
  tick(p, opening, 15)
  assert p.road_edges.edges[1].held and not p.active and p.arm_time == progress
  tick(p, model_output(), 3)
  assert p.active  # fresh recovery reuses the already-earned qualification


@pytest.mark.parametrize('side', [0, 1])
def test_edge_qualification_is_shared_without_a_second_centering_wait(side):
  p = LanePolicy()
  m = model_output(left_prob=0.99 if side else 0.1, right_prob=0.1 if side else 0.99)
  tick(p, m, 14)
  assert not p.active and not p.road_edges.edges[side].available
  tick(p, m)
  assert p.active and p.edge_side == side


@pytest.mark.parametrize('side', [0, 1])
def test_brief_paint_confidence_dip_pauses_but_does_not_earn_qualification(side):
  p = LanePolicy()
  strong = model_output(left_prob=0.99 if side else 0.1, right_prob=0.1 if side else 0.99)
  dip = {k: v.copy() for k, v in strong.items()}
  dip['lane_lines_prob'][0, 5 if side == 0 else 3] = 0.88
  tick(p, strong, 10)
  tick(p, dip, 4)
  assert not p.active and not p.road_edges.edges[side].available
  tick(p, strong, 4)
  assert not p.active  # fourteen strong observations, not eighteen
  tick(p, strong)
  assert p.active and p.edge_side == side


@pytest.mark.parametrize('issue', ['low_confidence', 'long_dip', 'width_step', 'anchor_step', 'anchor_heading', 'uncertain_edge'])
def test_paint_dip_does_not_bridge_deteriorating_acquisition_geometry(issue):
  p = LanePolicy()
  tick(p, model_output(), 10)
  dip = model_output(left_prob=0.88)
  count = 1
  if issue == 'low_confidence':
    dip['lane_lines_prob'][0, 3] = 0.84
  elif issue == 'long_dip':
    count = 6
  elif issue == 'width_step':
    dip['road_edges'][0, 1, :, 0] += 0.4
  elif issue == 'anchor_step':
    dip['lane_lines'][0, 1, :, 0] += 0.3
    dip['road_edges'][0, 1, :, 0] += 0.3
  elif issue == 'anchor_heading':
    # Same width and near position, but a changing painted-line direction.
    change = 0.015 * np.asarray(ModelConstants.X_IDXS)
    dip['lane_lines'][0, 1, :, 0] += change
    dip['road_edges'][0, 1, :, 0] += change
  else:
    dip['road_edges_stds'][0, 1, :, 0] = 0.6
  tick(p, dip, count)
  tick(p, model_output(), 5)
  assert not p.active and not p.road_edges.edges[1].available


def test_medium_paint_confidence_alone_never_earns_admission():
  p = LanePolicy()
  tick(p, model_output(left_prob=0.90), 200)
  assert not p.active and not any(e.available for e in p.road_edges.edges)


def test_scattered_strong_paint_observations_expire():
  p = LanePolicy()
  for _ in range(20):
    tick(p, model_output(), 2)
    tick(p, model_output(left_prob=0.88), 4)
  assert not p.active and not p.road_edges.edges[1].available


def test_confidence_pause_uses_elapsed_time_when_model_frames_drop():
  p = LanePolicy()
  for n in range(10):
    tick(p, model_output(), frame_time=n * DT)
  for n in range(3):
    tick(p, model_output(left_prob=0.88), frame_time=9 * DT + (n + 1) * 0.1)
  for n in range(5):
    tick(p, model_output(), frame_time=9 * DT + 0.3 + (n + 1) * DT)
  assert not p.active  # 0.30 seconds exhausted the pause, despite only 3 frames


@pytest.mark.parametrize('name,start,end', [('curb_shared_qualification', 2.3, 3.1), ('curb_paint_confidence_dip', 2.3, 7.1)])
def test_recorded_confidence_fluctuations_reuse_edge_qualification(name, start, end):
  frames = json.loads((Path(__file__).parent / 'fixtures' / f'{name}.json').read_text())['frames']
  p = LanePolicy()
  for stamp, probability, same_side_probability, std, paint, same_side, edge in frames:
    m = model_output(left_prob=probability, right_prob=same_side_probability, edge_std=std)
    m['road_edges_stds'][0, 0, :, 0] = 2.0
    m['lane_lines'][0, 1, :, 0] = np.pad(paint, (0, 33 - len(paint)), mode='edge')
    m['lane_lines'][0, 2, :, 0] = np.pad(same_side, (0, 33 - len(same_side)), mode='edge')
    m['road_edges'][0, 1, :, 0] = np.pad(edge, (0, 33 - len(edge)), mode='edge')
    tick(p, m, frame_time=stamp)
    if start <= stamp <= end:
      assert p.active and p.edge_side == 1, stamp


@pytest.mark.parametrize('speed,frames', [(0.0, 12), (30.0, 6)])
def test_dropped_model_frames_do_not_extend_bridge_time_or_distance(speed, frames):
  p = RoadEdgePolicy()
  x = np.asarray(ModelConstants.X_IDXS)
  for n in range(30):
    p.update(model_output(), x, speed, n * DT)
  opening = model_output()
  opening['road_edges'][0, 1, :, 0] += 2.0
  stamp = 29 * DT
  for _ in range(frames):
    stamp += 0.1
    p.update(opening, x, speed, stamp)
    assert p.edges[1].available
  stamp += 0.1
  p.update(opening, x, speed, stamp)
  assert not p.edges[1].available  # time or distance exhausted at half the nominal frame rate


def test_bridge_compares_narrow_corridor_end_to_its_previous_boundary_not_median():
  p = RoadEdgePolicy()
  x = np.asarray(ModelConstants.X_IDXS)
  m = model_output(width=4.2)
  m['road_edges'][0, 1, x < 15.0, 0] -= 0.4
  for _ in range(30):
    p.update(m, x, 10.0)
  assert p.edges[1].available and p.edges[1].width == pytest.approx(3.8)
  opening = {k: v.copy() for k, v in m.items()}
  opening['road_edges'][0, 1, x > 35.0, 0] += 2.0
  for _ in range(12):
    p.update(opening, x, 10.0)
    assert p.edges[1].available and p.edges[1].width == pytest.approx(3.8)
  # A true inward step is compared with the original cap, not a moving cap.
  opening['road_edges'][0, 1, x < 15.0, 0] -= 0.4
  p.update(opening, x, 10.0)
  assert not p.edges[1].available
