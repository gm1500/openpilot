"""Confidence must come from fresh paint, not held or corrected boundaries."""
from copy import deepcopy

import numpy as np
import pytest

from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.lane_policy import LanePolicy
from openpilot.selfdrive.modeld.road_edge_policy import DT, RoadEdgePolicy, WidthReference
from openpilot.selfdrive.modeld.tests.test_road_edge_policy import model_output

X = np.asarray(ModelConstants.X_IDXS)
HORIZON = X <= 55.0


def advance(policy, output, frames=1, speed=10.0, **kwargs):
  for _ in range(frames):
    policy.update(output, X, speed, **kwargs)


def trained(frames=180, width=3.6):
  p = RoadEdgePolicy()
  advance(p, model_output(offset=0.0, width=width, right_prob=0.99, edge_std=2.0), frames)
  assert p.width_reference.width == pytest.approx(width)
  return p


def wider(side=1, width=4.2, curvature=0.0, edge_std=0.3):
  m = model_output(offset=0.0, left_prob=0.99 if side else 0.0, right_prob=0.0 if side else 0.99,
                   curvature=curvature, edge_std=edge_std)
  direction = 1 if side else -1
  m['road_edges'][0, side, :, 0] = m['lane_lines'][0, 2 - side, :, 0] + direction * width
  return m


def output_width(p, m, side=1):
  e = p.edges[side]
  return e.direction * (e.y - m['lane_lines'][0, 2 - side, :, 0])


def test_fresh_duration_builds_confidence_but_holding_only_ages_it():
  short, long = trained(20), trained(180)
  assert short.width_reference.confidence < long.width_reference.confidence
  assert long.width_reference.confidence == pytest.approx(1.0)
  support = long.width_reference.support
  samples = list(long.width_reference.samples)
  last = long.width_reference.confidence
  for _ in range(40):
    advance(long, wider())
    assert long.width_reference.confidence < last
    last = long.width_reference.confidence
  assert long.width_reference.support == support
  assert list(long.width_reference.samples) == samples
  assert long.paint_memory.age == pytest.approx(2.0)


def test_qualified_edge_alone_never_teaches_independent_painted_reference():
  p = RoadEdgePolicy()
  advance(p, wider(), 200)
  assert p.edges[1].available
  assert p.width_reference.width is None and p.width_reference.support == 0.0
  np.testing.assert_allclose(output_width(p, wider()), 4.2)


def test_one_noisy_narrow_frame_cannot_ratchet_the_reference():
  p = trained()
  narrow = model_output(offset=0.0, width=3.0, right_prob=0.99, edge_std=2.0)
  advance(p, narrow)
  # Conflicting strong paint invalidates instead of becoming a permanent min.
  assert p.width_reference.width is None
  advance(p, model_output(offset=0.0, right_prob=0.99, edge_std=2.0), 20)
  assert p.width_reference.width == pytest.approx(3.6)
  assert p.width_reference.confidence < 0.5


def test_stable_lower_width_is_robust_and_can_relearn_a_real_widening():
  p = trained()
  for width in (3.60, 3.62, 3.58, 3.60) * 20:
    advance(p, model_output(offset=0.0, width=width, right_prob=0.99, edge_std=2.0))
  assert 3.59 <= p.width_reference.width <= 3.61
  advance(p, model_output(offset=0.0, width=4.2, right_prob=0.99, edge_std=2.0), 25)
  assert p.width_reference.width == pytest.approx(4.2)
  assert p.width_reference.confidence < 0.5


@pytest.mark.parametrize('side', [0, 1])
def test_wider_railing_is_weighted_without_changing_admission_or_learning(side):
  p = trained()
  m = wider(side)
  advance(p, m, 14)
  assert not p.edges[side].available
  advance(p, m)
  e = p.edges[side]
  assert e.available and e.width_weighted
  assert e.width == pytest.approx(4.2) and e.memory_width == pytest.approx(4.2)
  # Previous center error would be 30 cm. Strong independent evidence keeps
  # this synthetic, aligned case within 5 cm without changing correction gain.
  assert np.max(np.abs(output_width(p, m, side) - 3.6)) / 2.0 < 0.05
  assert p.width_reference.width == pytest.approx(3.6)
  assert p.paint_memory.age == pytest.approx(0.75)


def test_longer_confirmed_width_gets_more_weight():
  short, long = trained(20), trained(180)
  m = wider()
  advance(short, m, 15)
  advance(long, m, 15)
  assert output_width(long, m)[0] < output_width(short, m)[0] < 4.2


def test_e2e_support_can_reduce_edge_weight_but_never_becomes_the_target():
  aligned, outside = trained(), trained()
  m, far = wider(), wider()
  far['plan'][0, :, 1] += 1.0
  advance(aligned, m, 15)
  advance(outside, far, 15)
  assert 3.6 <= output_width(aligned, m)[0] < output_width(outside, far)[0] < 4.2


@pytest.mark.parametrize('bad_plan', ['absent', 'nan', 'backwards', 'short', 'late_start'])
def test_missing_or_invalid_e2e_cannot_artificially_favor_memory(bad_plan):
  m = wider()
  if bad_plan == 'absent':
    del m['plan']
  elif bad_plan == 'nan':
    m['plan'][0, 10, 1] = np.nan
  elif bad_plan == 'backwards':
    m['plan'][0, 10, 0] = -1.0
  elif bad_plan == 'short':
    m['plan'][0, :, 0] *= 0.1
  else:
    m['plan'][0, :, 0] += 10.0
  assert WidthReference._path_weight(m, X, m['lane_lines'][0, 1, :, 0], 1, 3.6, 4.2) == 1.0


@pytest.mark.parametrize('side', [0, 1])
def test_curve_geometry_moves_with_live_paint_and_retains_width(side):
  p = trained()
  m = wider(side)
  advance(p, m, 15)
  support = p.width_reference.support
  for curvature in np.linspace(0.0, 0.002, 20):
    m = wider(side, curvature=curvature)
    advance(p, m)
    assert p.edges[side].available and p.width_reference.support == support
    widths = output_width(p, m, side)
    np.testing.assert_allclose(widths, widths[0])
    assert 3.6 <= widths[0] < 3.8


@pytest.mark.parametrize('side', [0, 1])
@pytest.mark.parametrize('distance', [0.0, 18.75, 54.1875])
def test_inward_sample_and_inner_line_remain_hard_bounds(side, distance):
  p = trained()
  m = wider(side)
  advance(p, m, 20)
  e = p.edges[side]
  m['road_edges'][0, side, X == distance, 0] = m['lane_lines'][0, 2 - side, X == distance, 0] + e.direction * 3.4
  advance(p, m)
  assert e.available
  for y in (e.y, e.memory_y):
    assert np.max(e.direction * (y - m['road_edges'][0, side, :, 0])[HORIZON]) <= 1e-8
  assert p.width_reference.width == pytest.approx(3.6)


def test_expiry_releases_smoothly_without_extending_edge_availability():
  p = trained()
  m = wider()
  advance(p, m, 20)
  previous = output_width(p, m)[0]
  for _ in range(160):
    advance(p, m)
    current = output_width(p, m)[0]
    assert 0.0 <= current - previous <= 0.20 * DT + 1e-8
    previous = current
  assert p.width_reference.width is None
  assert not p.edges[1].width_weighted
  assert current == pytest.approx(4.2)


def test_intersection_opening_keeps_anchor_but_cannot_renew_memory_or_hold_forever():
  p = trained()
  advance(p, wider(), 20)
  opening = wider(width=6.0)
  previous_support = p.width_reference.support
  advance(p, opening, 20)
  assert p.edges[1].available and p.edges[1].held
  assert output_width(p, opening)[0] < 3.8
  assert p.width_reference.support == previous_support
  advance(p, opening, 22)
  assert not p.edges[1].available
  e = p.edges[1]
  assert e.y is None and e.memory_y is not None  # display only, never steering
  memory_width = e.direction * (e.memory_y - opening['lane_lines'][0, 1, :, 0])
  assert np.max(memory_width) < 3.9
  assert p.width_reference.support == previous_support


@pytest.mark.parametrize('problem', ['two_edges', 'wide', 'uncertain', 'narrow'])
def test_trusted_memory_and_matching_e2e_cannot_qualify_bad_boundaries(problem):
  p = trained()
  m = wider()
  if problem == 'two_edges':
    m['lane_lines_prob'][0, 3] = 0.1
  elif problem == 'wide':
    m = wider(width=6.0)
  elif problem == 'uncertain':
    m = wider(edge_std=0.9)
  else:
    m = wider(width=2.8)
  advance(p, m, 40)
  assert not any(e.available for e in p.edges)


@pytest.mark.parametrize('reason', ['lost_anchor', 'anchor_jump', 'time', 'distance', 'disengage', 'turn'])
def test_association_and_age_resets_clear_reference(reason):
  p = trained()
  m = wider()
  kwargs, count, speed = {}, 1, 10.0
  if reason == 'lost_anchor':
    m['lane_lines_prob'][0, 3] = 0.1
  elif reason == 'anchor_jump':
    m['lane_lines'][0, 1, :, 0] -= 0.3
  elif reason == 'time':
    count, speed = 101, 0.0
  elif reason == 'distance':
    count, speed = 68, 30.0
  elif reason == 'disengage':
    count, kwargs = 100, {'lateral_active': False}
  else:
    m['desire_state'][0, 1] = 0.5
  advance(p, m, count, speed, **kwargs)
  assert p.width_reference.width is None


@pytest.mark.parametrize('reason', ['blinker', 'lane_change', 'toggle', 'frame_gap'])
def test_lane_policy_resets_also_clear_weighted_target(reason):
  p = LanePolicy()
  p.road_edges = trained()
  m = wider()
  advance(p.road_edges, m, 20)
  kwargs = {'lane_policy_enabled': True, 'lateral_active': True}
  if reason == 'blinker':
    kwargs['blinkers_active'] = True
  elif reason == 'lane_change':
    m['desire_state'][0, 3] = 0.5
  elif reason == 'toggle':
    kwargs['lane_policy_enabled'] = False
  else:
    p.road_edges.last_time = 1.0
    kwargs['frame_time'] = 2.0
  p.update(m, 0.0, 10.0, **kwargs)
  assert p.road_edges.width_reference.width is None
  assert p.road_edges.edges[1].position_width is None


def test_two_painted_lines_keep_exact_original_center_and_correction():
  weighted, original = LanePolicy(), LanePolicy()
  weighted.road_edges = trained()
  advance(weighted.road_edges, wider(), 20)
  m = model_output(offset=0.3, right_prob=0.99)
  for _ in range(40):
    a = weighted.update(m, 0.001, 20.0, lane_policy_enabled=True, lateral_active=True)
    b = original.update(m, 0.001, 20.0, lane_policy_enabled=True, lateral_active=True)
  assert weighted.edge_side is None and original.edge_side is None
  assert a == pytest.approx(b)


def test_weighted_memory_uses_yellow_and_only_active_edge_is_thick():
  from openpilot.cereal import log
  from openpilot.selfdrive.ui.onroad.lane_policy_style import active_edge, boundary_colors, EDGE_YELLOW

  p = LanePolicy()
  p.road_edges = trained()
  m = wider()
  for _ in range(20):
    p.update(m, 0.0, 10.0, lane_policy_enabled=True, lateral_active=True)
  state = log.ModelDataV2.new_message().lanePolicy
  p.fill_state(state, True, True, False)
  assert state.active and state.edges[1].held
  assert active_edge(state, 1, True) and not active_edge(state, 0, True)
  assert boundary_colors(state, 1, True)[1] == EDGE_YELLOW
  assert not active_edge(state, 1, False)
  assert boundary_colors(state, 1, False)[1] == EDGE_YELLOW
  np.testing.assert_allclose(state.edges[1].y, p.road_edges.edges[1].y, atol=1e-6)


def test_path_consistency_compares_shared_geometry_on_curves_not_origin():
  straight = wider()
  curved = wider(curvature=0.002)
  for m in (straight, curved):
    weight = WidthReference._path_weight(m, X, m['lane_lines'][0, 1, :, 0], 1, 3.6, 4.2)
    assert weight == pytest.approx(0.1)
  crossing = deepcopy(straight)
  crossing['plan'][0, :, 1] = 0.04 * X
  assert WidthReference._path_weight(crossing, X, crossing['lane_lines'][0, 1, :, 0], 1, 3.6, 4.2) == 1.0
