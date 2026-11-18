"""A remembered scalar may position from live paint without becoming a new edge observation."""
from copy import deepcopy

import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.lane_policy import LanePolicy
from openpilot.selfdrive.modeld.road_edge_policy import RoadEdgePolicy
from openpilot.selfdrive.modeld.tests.test_road_edge_inner_line import inward_line
from openpilot.selfdrive.modeld.tests.test_road_edge_policy import model_output, tick

X = np.asarray(ModelConstants.X_IDXS)


def advance(policy, output, frames=1, speed=10.0, **kwargs):
  for _ in range(frames):
    policy.update(output, X, speed, **kwargs)


def learned(side=1, frames=80):
  policy = RoadEdgePolicy()
  normal = model_output(left_prob=0.99 if side else 0.0, right_prob=0.0 if side else 0.99)
  advance(policy, normal, frames)
  opening = deepcopy(normal)
  opening['road_edges'][0, side, :, 0] += 3.0 if side else -3.0
  return policy, normal, opening


@pytest.mark.parametrize('side', [0, 1])
def test_only_mature_fresh_history_can_position_across_a_long_opening(side):
  for frames, mature in ((30, False), (80, True)):
    p, _, opening = learned(side, frames)
    e = p.edges[side]
    width, support = e.anchor_width, e.memory_support
    advance(p, opening, 120)
    assert e.available == mature
    if mature:
      assert e.anchor_guided and e.observed_frames == 0
      assert e.memory_time == pytest.approx(6.0)
      assert e.memory_support == support
      np.testing.assert_allclose(e.y, opening['lane_lines'][0, 2 - side, :, 0] + e.direction * width)
    else:
      assert e.memory_width is None


@pytest.mark.parametrize('side', [0, 1])
@pytest.mark.parametrize('probability', [0.84, 0.60])
def test_confidence_dip_keeps_only_scalar_and_requires_fresh_strong_anchor(side, probability):
  p, _, opening = learned(side)
  advance(p, opening)
  e = p.edges[side]
  width = e.anchor_width
  weak = deepcopy(opening)
  weak['lane_lines_prob'][0, 5 if side == 0 else 3] = probability
  advance(p, weak, 30)
  assert not e.available and e.y is None and e.memory_y is None
  assert e.memory_width is not None and e.anchor_width == width
  assert e.inner_line.width is None
  advance(p, opening, 5)
  assert not e.available
  advance(p, opening)
  assert e.available and e.anchor_guided and e.observed_frames == 0
  assert e.memory_time == pytest.approx(1.85)


@pytest.mark.parametrize('side', [0, 1])
@pytest.mark.parametrize('speed,frames', [(0.0, 300), (25.0, 200)])
def test_mature_private_lease_expires_without_renewal_from_anchor_output(side, speed, frames):
  p, _, opening = learned(side)
  advance(p, opening, frames, speed)
  assert p.edges[side].available
  advance(p, opening, speed=speed)
  assert not p.edges[side].available and p.edges[side].memory_width is None


@pytest.mark.parametrize('side', [0, 1])
def test_current_inward_sample_caps_and_hard_narrowing_clears_memory(side):
  p, normal, opening = learned(side)
  e = p.edges[side]
  advance(p, opening)
  capped = deepcopy(opening)
  index = int(np.argmin(abs(X - 45.0)))
  capped['road_edges'][0, side, index, 0] = normal['road_edges'][0, side, index, 0] - e.direction * 0.2
  advance(p, capped)
  assert e.available and e.anchor_guided
  widths = e.direction * (e.y - capped['lane_lines'][0, 2 - side, :, 0])
  assert np.max(widths) == pytest.approx(3.4)
  capped['road_edges'][0, side, index, 0] -= e.direction * 0.2
  advance(p, capped)
  assert not e.available and e.memory_width is None


@pytest.mark.parametrize('side', [0, 1])
def test_live_anchor_and_virtual_boundary_must_both_bracket_ego(side):
  p, _, opening = learned(side)
  e = p.edges[side]
  # A slow, continuous shift cannot evade the virtual boundary's own bracket.
  for n in range(1, 40):
    shifted = deepcopy(opening)
    shifted['lane_lines'][0, 2 - side, :, 0] -= e.direction * n * 0.1
    advance(p, shifted)
    if e.memory_width is None:
      break
    assert e.y is None or np.min(e.direction * e.y[X <= 10]) >= 0.3
  assert e.memory_width is None and not e.available


@pytest.mark.parametrize('reason', ['anchor_jump', 'turn', 'stale', 'disengage', 'blinker', 'lane_change'])
def test_association_resets_also_clear_mature_private_memory(reason):
  p = LanePolicy()
  normal = model_output(right_prob=0.0)
  tick(p, normal, 80, lateral_active=True)
  assert p.road_edges.edges[1].mature_memory
  options = {'lateral_active': True}
  frames = 1
  if reason == 'anchor_jump':
    normal['lane_lines'][0, 1, :, 0] -= 0.4
  elif reason == 'turn':
    normal['desire_state'][0, 1] = 0.2
  elif reason == 'lane_change':
    normal['desire_state'][0, 3] = 0.2
  elif reason == 'blinker':
    options['blinkers_active'] = True
  elif reason == 'disengage':
    options['lateral_active'] = False
    frames = 100
  else:
    tick(p, normal, frame_time=10.0, lateral_active=True)
    options['frame_time'] = 10.5
  tick(p, normal, frames, **options)
  assert p.road_edges.edges[1].memory_width is None


@pytest.mark.parametrize('side', [0, 1])
def test_applied_inward_constraint_survives_expiry_and_reacquisition(side):
  p = RoadEdgePolicy()
  normal = inward_line(side)
  advance(p, normal, 100)
  e = p.edges[side]
  assert e.memory_width == pytest.approx(4.0)
  assert e.anchor_width == pytest.approx(3.6)
  opening = deepcopy(normal)
  opening['road_edges'][0, side, :, 0] += e.direction * 3.0
  opening['lane_lines_prob'][0, 3 if side == 0 else 5] = 0.0
  advance(p, opening, 120)
  assert e.inner_line.width is None and e.anchor_guided
  for boundary in (e.y, e.memory_y):
    np.testing.assert_allclose(e.direction * (boundary - opening['lane_lines'][0, 2 - side, :, 0]), 3.6)
  weak = deepcopy(opening)
  weak['lane_lines_prob'][0, 5 if side == 0 else 3] = 0.6
  advance(p, weak)
  assert e.y is None and e.memory_y is None
  returning = deepcopy(normal)
  returning['lane_lines_prob'][0, 3 if side == 0 else 5] = 0.0
  returning['road_edges_stds'][:] = 0.6
  returning['road_edges'][0, side, :, 0] -= e.direction * 0.25
  advance(p, returning, 6)
  assert e.available and e.memory_guided
  np.testing.assert_allclose(e.direction * (e.y - returning['lane_lines'][0, 2 - side, :, 0]), 3.6)
  assert e.memory_time == pytest.approx(6.35)


def test_mature_return_compares_conservative_width_instead_of_old_median():
  p = RoadEdgePolicy()
  normal = model_output(width=4.4, right_prob=0.0)
  normal['road_edges'][0, 1, X <= 5.0, 0] -= 0.15
  advance(p, normal, 80)
  e = p.edges[1]
  assert e.memory_width == pytest.approx(4.25)
  opening = deepcopy(normal)
  opening['road_edges'][0, 1, :, 0] += 2.0
  opening['lane_lines_prob'][0, 3] = 0.6
  advance(p, opening)
  returning = deepcopy(normal)
  returning['road_edges'][0, 1, :, 0] = returning['lane_lines'][0, 1, :, 0] + 4.17
  returning['road_edges_stds'][:] = 0.6
  advance(p, returning, 7)
  assert e.available and e.memory_guided
  assert e.width == pytest.approx(4.17)


def test_transient_paint_blocks_anchor_mode_then_qualified_paint_clears_corridor():
  p = LanePolicy()
  normal = model_output(width=4.2, right_prob=0.0)
  tick(p, normal, 80, lateral_active=True)
  opening = deepcopy(normal)
  opening['road_edges'][0, 1, :, 0] += 2.0
  tick(p, opening, lateral_active=True)
  assert p.road_edges.edges[1].anchor_guided
  opening['lane_lines_prob'][0, 5] = 0.99
  opening['lane_lines'][0, 2, :, 0] -= 0.6
  tick(p, opening, lateral_active=True)
  assert p.edge_side is None
  e = p.road_edges.edges[1]
  assert e.memory_width is not None and e.anchor_blocked and not e.available
  tick(p, opening, 14, lateral_active=True)
  assert e.memory_width is None


def test_anchor_mode_is_explicit_without_claiming_observed_edge_evidence():
  p = LanePolicy()
  normal = model_output(right_prob=0.0)
  tick(p, normal, 80, lateral_active=True)
  opening = deepcopy(normal)
  opening['road_edges'][0, 1, :, 0] += 2.0
  tick(p, opening, 50, lateral_active=True)
  e = p.road_edges.edges[1]
  assert p.active and e.anchor_guided and e.observed_frames == 0
  state = log.ModelDataV2.new_message().lanePolicy
  p.fill_state(state, True, True, False)
  assert state.active and state.rightSource == 'roadEdge'
  assert state.edges[1].available and state.edges[1].held and not state.edges[1].memoryOnly


def test_fragmented_returning_edges_cannot_renew_private_lease():
  p, normal, opening = learned()
  normal['road_edges_stds'][:] = 0.6
  for frame in range(301):
    advance(p, normal if frame % 8 < 6 else opening, speed=0.0)
  assert p.edges[1].memory_width is None
  assert not p.edges[1].available


def test_weak_frames_cannot_make_immature_history_mature():
  p, _, opening = learned(frames=30)
  support = p.edges[1].memory_support
  advance(p, opening, 60)
  assert p.edges[1].memory_support == support
  assert not p.edges[1].mature_memory and not p.edges[1].anchor_guided


@pytest.mark.parametrize('side', [0, 1])
def test_memory_guided_return_rejects_a_virtual_boundary_on_wrong_side_of_ego(side):
  p = RoadEdgePolicy()
  normal = inward_line(side)
  advance(p, normal, 100)
  e = p.edges[side]
  assert e.anchor_width == pytest.approx(3.6)
  opening = deepcopy(normal)
  opening['road_edges'][0, side, :, 0] += e.direction * 3.0
  advance(p, opening, 120)
  returning = deepcopy(normal)
  returning['lane_lines'][0, 2 - side, :, 0] = -e.direction * 3.75
  returning['road_edges'][0, side, :, 0] = e.direction * 0.4
  returning['road_edges_stds'][:] = 0.6
  returning['lane_lines_prob'][0, 3 if side else 5] = 0.6
  advance(p, returning)
  returning['lane_lines_prob'][0, 3 if side else 5] = 0.99
  for _ in range(6):
    advance(p, returning)
    assert e.memory_y is None
    assert not e.available
  assert e.memory_width is None


@pytest.mark.parametrize('frames,mature', [(63, False), (64, True)])
def test_maturity_requires_full_qualified_support_after_initial_arm(frames, mature):
  p, _, opening = learned(frames=frames)
  assert p.edges[1].mature_memory == mature
  advance(p, opening, 45)
  assert p.edges[1].anchor_guided == mature


def test_moderate_confidence_can_retain_but_cannot_start_anchor_mode():
  p, _, opening = learned()
  e = p.edges[1]
  weak = deepcopy(opening)
  weak['lane_lines_prob'][0, 3] = 0.90
  advance(p, weak, 45)
  assert not e.anchor_guided and not e.available
  advance(p, opening, 6)
  assert e.anchor_guided
  age = e.memory_time
  advance(p, weak, 20)
  assert e.anchor_guided and e.available
  assert e.memory_time == pytest.approx(age + 1.0)
  weak['lane_lines_prob'][0, 3] = 0.84
  advance(p, weak)
  assert not e.available and e.memory_y is None


def test_outward_observation_variation_does_not_drop_frozen_memory_reacquisition():
  p, normal, opening = learned()
  weak = deepcopy(opening)
  weak['lane_lines_prob'][0, 3] = 0.6
  advance(p, weak)
  normal['road_edges_stds'][:] = 0.6
  advance(p, normal, 6)
  e = p.edges[1]
  width, age = e.width, e.memory_time
  assert e.memory_guided
  normal['road_edges'][0, 1, :, 0] += 0.25
  advance(p, normal, 7)
  assert e.available and e.memory_guided
  assert e.width == pytest.approx(width)
  assert e.memory_time == pytest.approx(age + 0.35)


def test_qualified_paint_conflict_cannot_leave_memory_dependent_boundary_active():
  p, normal, opening = learned()
  opening['lane_lines_prob'][0, 3] = 0.6
  advance(p, opening)
  normal['road_edges_stds'][:] = 0.6
  advance(p, normal, 6)
  e = p.edges[1]
  assert e.memory_guided
  paint = deepcopy(normal)
  paint['lane_lines_prob'][0, 5] = 0.99
  paint['lane_lines'][0, 2, :, 0] -= 0.4
  advance(p, paint, 15)
  assert e.memory_width is None and not e.available and not e.memory_guided
  opening['lane_lines_prob'][0, 3] = 0.99
  advance(p, opening)
  assert not e.available
