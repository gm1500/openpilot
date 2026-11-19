"""Behavioral coverage for paint-only centering on the Cheddar model path."""
import copy
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.lane_policy import (
  ARM_TIME, CURVE_THRESHOLD, DT, ENGAGE_STEP, MAX_CORRECTION, LanePolicy,
  anchor_correction, curve_reversal_frames, is_enabled,
)


def perception(offset=0.25, width=3.6, probability=.99, curvature=0., horizon=100.):
  x = np.asarray(ModelConstants.X_IDXS)
  center = offset + .5 * curvature * x**2
  lanes = np.zeros((1, 4, len(x), 2))
  lanes[0, :, :, 0] = center + np.array([-1.5, -.5, .5, 1.5])[:, None] * width
  plan = np.zeros((1, len(x), ModelConstants.PLAN_WIDTH))
  plan_x = np.linspace(0., horizon, len(x))
  plan[0, :, 0] = plan_x
  plan[0, :, 1] = .5 * curvature * plan_x**2
  probs = np.zeros((1, 8))
  probs[0, 3] = probs[0, 5] = probability
  return {'lane_lines': lanes, 'lane_lines_prob': probs, 'plan': plan, 'desire_state': np.zeros((1, 8))}


def step(policy, data, frames=1, **kwargs):
  result = 0.
  curvature, speed = kwargs.pop('e2e_curvature', 0.), kwargs.pop('v_ego', 20.)
  for _ in range(frames):
    result = policy.update(data, curvature, speed,
                           lane_policy_enabled=True, **kwargs)
  return result


@pytest.mark.parametrize('offset', [-.4, -.15, .15, .4])
def test_centers_toward_painted_midpoint_with_bounded_entry(offset):
  policy = LanePolicy()
  data = perception(offset)
  result = step(policy, data, frames=round(ARM_TIME / DT) - 1)
  assert result == 0.
  assert not policy.active and policy.blending
  result = step(policy, data)
  assert policy.active and not policy.blending
  assert 0. < abs(result) <= ENGAGE_STEP + 1e-12
  assert result * offset > 0.
  result = step(policy, data, frames=100)
  assert 0. < abs(result) <= MAX_CORRECTION
  assert result * offset > 0.


def test_centered_lane_does_not_add_steering():
  policy = LanePolicy()
  assert step(policy, perception(0.), frames=100) == pytest.approx(0., abs=1e-12)


@pytest.mark.parametrize('side', [3, 5])
def test_missing_paint_discards_alignment_without_width_carryover(side):
  policy = LanePolicy()
  data = perception(.3)
  step(policy, data, frames=300, lateral_active=True)
  assert policy.active
  data['lane_lines_prob'][0, side] = .2
  e2e = .00025
  assert step(policy, data, e2e_curvature=e2e) == e2e
  assert not policy.active and not policy.blending
  assert policy.correction == 0.
  assert not hasattr(policy, 'width')
  assert not hasattr(policy, 'road_edges')
  # A newly visible line must earn a fresh confidence window.
  data['lane_lines_prob'][0, side] = .99
  step(policy, data, frames=round(ARM_TIME / DT) - 1)
  assert not policy.active
  step(policy, data)
  assert policy.active


def test_one_line_never_arms_even_with_visible_road_edges():
  policy = LanePolicy()
  data = perception()
  data['lane_lines_prob'][0, 5] = .1
  data['road_edges'] = np.zeros((1, 2, ModelConstants.IDX_N, 2))
  data['road_edges'][0, :, :, 0] = np.array([-1.8, 1.8])[:, None]
  for _ in range(200):
    assert step(policy, data) == 0.
    assert not policy.active


def test_road_edge_inputs_cannot_change_painted_lane_result():
  data = perception()
  extreme = copy.deepcopy(data)
  extreme['road_edges'] = np.full((1, 2, ModelConstants.IDX_N, 2), np.nan)
  a, b = LanePolicy(), LanePolicy()
  for _ in range(40):
    assert step(a, data) == step(b, extreme)


@pytest.mark.parametrize('width', [1.8, 2.65, 5.15, 6.0])
def test_implausible_lane_width_rejects_centering(width):
  policy = LanePolicy()
  assert step(policy, perception(width=width), frames=50) == 0.
  assert not policy.active


def test_width_divergence_rejects_a_merge_shape():
  policy = LanePolicy()
  data = perception()
  data['lane_lines'][0, 2, :, 0] += .05 * np.asarray(ModelConstants.X_IDXS)
  assert step(policy, data, frames=50) == 0.
  assert not policy.active


def test_retention_confidence_is_lower_than_fresh_admission():
  data = perception(probability=.8)
  policy = LanePolicy()
  assert step(policy, data, frames=40) == 0.
  data['lane_lines_prob'][0, [3, 5]] = .99
  step(policy, data, frames=20)
  assert policy.active
  data['lane_lines_prob'][0, [3, 5]] = .8
  assert step(policy, data) != 0.
  assert policy.active


@pytest.mark.parametrize('override', ['blinkers_active', 'lane_change_intent', 'disabled'])
def test_model_fallback_releases_centering_immediately(override):
  policy = LanePolicy()
  data = perception()
  step(policy, data, frames=40)
  assert policy.active
  if override == 'lane_change_intent':
    data['desire_state'][0, 3] = .25
  result = policy.update(data, .001, 20., blinkers_active=override == 'blinkers_active',
                         lane_policy_enabled=override != 'disabled')
  assert result == .001
  assert not policy.active and not policy.blending


@pytest.mark.parametrize('bad', ['nan_line', 'nan_prob', 'short_plan', 'backward_plan', 'missing_plan'])
def test_invalid_perception_returns_original_model_curvature(bad):
  policy = LanePolicy()
  data = perception()
  if bad == 'nan_line':
    data['lane_lines'][0, 1, :, 0] = np.nan
  elif bad == 'nan_prob':
    data['lane_lines_prob'][0, 3] = np.nan
  elif bad == 'short_plan':
    data = perception(horizon=15.)
  elif bad == 'backward_plan':
    data['plan'][0, :, 0] *= -1.
  else:
    data.pop('plan')
  for _ in range(40):
    assert policy.update(data, .0005, 20., lane_policy_enabled=True) == .0005
  assert not policy.active


def test_raw_model_geometry_is_never_modified():
  data = perception()
  before = copy.deepcopy(data)
  step(LanePolicy(), data, frames=50)
  for name in data:
    np.testing.assert_array_equal(data[name], before[name])


@pytest.mark.parametrize('speed,frames', [(0., 6), (50. / 3.6, 3), (100. / 3.6, 1), (130. / 3.6, 1)])
def test_speed_dependent_curve_pause(speed, frames):
  assert curve_reversal_frames(speed) == frames
  policy = LanePolicy()
  policy._guard_correction(.0001, CURVE_THRESHOLD * 2, speed)
  paused = []
  for _ in range(frames):
    value, guarded = policy._guard_correction(-.0001, CURVE_THRESHOLD * 2, speed)
    paused.append(value)
    assert guarded
  assert paused == [0.] * frames
  value, guarded = policy._guard_correction(-.0001, CURVE_THRESHOLD * 2, speed)
  assert value == -.0001 and not guarded


def test_curve_pause_is_not_extended_by_speed_or_target_changes():
  policy = LanePolicy()
  policy._guard_correction(.0001, .001, 0.)
  assert policy._guard_correction(-.0001, .001, 0.)[0] == 0.
  assert policy.curve_release_frames == 5
  for index in range(5):
    assert policy._guard_correction((1 if index % 2 else -1) * .0001, .001, 40.)[0] == 0.
    assert policy.curve_release_frames == 4 - index
  assert policy._guard_correction(-.0001, .001, 40.) == (-.0001, False)


def test_live_paint_target_preserves_existing_curved_path_shape():
  data = perception(offset=.2, curvature=.001)
  x = np.asarray(ModelConstants.X_IDXS)
  center = data['lane_lines'][0, 1:3, :, 0].mean(axis=0)
  correction, _, _ = anchor_correction(data, center, x, 20.)
  assert 0. < correction <= MAX_CORRECTION


def test_hud_state_only_reports_live_paint_and_actual_engagement():
  policy = LanePolicy()
  step(policy, perception(), frames=20)
  state = SimpleNamespace()
  policy.fill_state(state, True, True, False)
  assert state.valid and state.enabled and state.active
  assert state.leftSource == state.rightSource == 'painted'
  policy.fill_state(state, True, True, True)
  assert not state.active
  policy.fill_state(state, False, True, False)
  assert not state.active and state.leftSource == state.rightSource == 'none'


@pytest.mark.parametrize('value,expected', [(None, True), (b'1', True), (b'0', False), (True, True), (False, False)])
def test_default_and_explicit_enable(value, expected):
  assert is_enabled(value) is expected
