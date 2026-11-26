from dataclasses import replace
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest

from openpilot.selfdrive.ui.onroad.model_stop import StopMarker
from openpilot.selfdrive.ui.onroad.stop_bar_animation import (
  LANE_EDGES, StopBarPresentation, lane_edges_at_stop, stop_bar_fade_allowed,
)
from openpilot.selfdrive.ui.tests.test_model_stop import SM, model


def road(left=-1.8, right=1.8, curve=0.):
  prediction = model(decel=0.)
  prediction.position.y = curve * prediction.position.x**2
  prediction.laneLines = [NS(x=prediction.position.x.copy(), y=prediction.position.y + offset,
                             z=prediction.position.z.copy()) for offset in (-5.4, left, right, 5.4)]
  prediction.laneLineProbs = [0., 1., 1., 0.]
  return prediction


def advance(presentation, marker, prediction, now, **kwargs):
  options = {'now': now, 'started_frame': 1, 'engaged': True, 'model_time': now}
  options.update(kwargs)
  presentation.update(marker, prediction, **options)


def settled(presentation, marker, prediction):
  for index in range(41):
    advance(presentation, marker, prediction, 100. + index * .05)


@pytest.fixture
def marker():
  return StopMarker((31.52, 0., 0.), 0., 30., label='STOP TARGET')


@pytest.fixture
def healthy_sm():
  sm = SM(road())
  sm['selfdriveState'].enabled = True
  sm['carState'].gasPressed = False
  sm['carState'].brakePressed = False
  sm['radarState'] = NS(leadOne=NS(present=False), leadTwo=NS(present=False))
  sm['longitudinalPlan'] = NS(stopTarget=NS(active=False, approaching=False), e2eStopActive=False)
  sm['extrinsicsCalibration'] = NS(calStatus='calibrated')
  sm.valid = dict.fromkeys(sm, True)
  sm.recv_frame = dict.fromkeys(sm, 10)
  sm.stamp(100.)
  return sm


def test_current_target_is_immediate_while_visibility_fades(marker):
  presentation, prediction = StopBarPresentation(), road()
  advance(presentation, marker, prediction, 100.)
  assert presentation.marker is marker
  assert 0. < presentation.opacity < 1.

  closer = replace(marker, position=(13.52, 0., 0.), distance=12., label='BRAKE HOLD')
  advance(presentation, closer, prediction, 100.05)
  assert presentation.marker is closer
  assert presentation.marker.position == (13.52, 0., 0.)
  assert presentation.marker.distance == 12.
  assert presentation.marker.label == 'BRAKE HOLD'
  assert marker.position == (31.52, 0., 0.)
  assert marker.distance == 30.


def test_edges_ease_without_easing_target(marker):
  presentation = StopBarPresentation()
  advance(presentation, marker, road(), 100.)
  closer = replace(marker, position=(13.52, 0., 0.), distance=12.)
  advance(presentation, closer, road(-2.8, 2.8), 100.2)
  np.testing.assert_allclose(presentation.edges, (-2.3, 2.3))
  assert presentation.marker is closer
  advance(presentation, closer, road(-2.8, 2.8), 100.4)
  np.testing.assert_allclose(presentation.edges, (-2.55, 2.55))


def test_lane_edges_use_position_and_curved_path_not_label_distance(marker):
  prediction = road(curve=.002)
  prediction.laneLines[1].y -= .01 * prediction.position.x
  prediction.laneLines[2].y += .02 * prediction.position.x
  assert lane_edges_at_stop(marker, prediction) == pytest.approx((-1.8 - .01 * 31.52, 1.8 + .02 * 31.52))


@pytest.mark.parametrize('probability', [0., .5, .50001, 1., float('nan')])
def test_lane_probability_threshold_and_independent_fallback(marker, probability):
  prediction = road(-2.5, 2.5)
  prediction.laneLineProbs[1:3] = [probability, 0.]
  expected_left = -2.5 if probability > .5 else -1.8
  assert lane_edges_at_stop(marker, prediction) == pytest.approx((expected_left, 1.8))


@pytest.mark.parametrize('failure', ['missing', 'nonfinite', 'reversed', 'crossed', 'unknown_position'])
def test_unusable_lane_edges_fall_back(marker, failure):
  prediction = road()
  if failure == 'missing':
    prediction.laneLines = []
  elif failure == 'nonfinite':
    prediction.laneLines[1].y[:] = np.nan
    prediction.laneLines[2].y[:] = np.nan
  elif failure == 'reversed':
    for line in prediction.laneLines:
      line.x = line.x[::-1]
  elif failure == 'crossed':
    prediction = road(2., -2.)
  else:
    marker = replace(marker, position=None)
  assert lane_edges_at_stop(marker, prediction) == LANE_EDGES


def test_engagement_fades_independently_of_visibility(marker):
  reference, changing, prediction = StopBarPresentation(), StopBarPresentation(), road()
  for presentation in (reference, changing):
    advance(presentation, marker, prediction, 100.)
  advance(reference, marker, prediction, 100.25)
  advance(changing, marker, prediction, 100.25, engaged=False)
  assert changing.opacity / reference.opacity == pytest.approx(.8)
  advance(reference, marker, prediction, 100.5)
  advance(changing, marker, prediction, 100.5)
  assert changing.opacity / reference.opacity == pytest.approx(.9)

  disengaged = StopBarPresentation()
  advance(disengaged, marker, prediction, 100., engaged=False)
  first = StopBarPresentation()
  advance(first, marker, prediction, 100.)
  assert disengaged.opacity / first.opacity == pytest.approx(.6)


def test_source_opacity_reduction_applies_once_immediately(marker):
  presentation, reference, prediction = StopBarPresentation(), StopBarPresentation(), road()
  for display in (presentation, reference):
    settled(display, marker, prediction)
  reduced = replace(marker, trajectory_only=True, opacity=.25)
  advance(presentation, reduced, prediction, 102.05)
  advance(reference, marker, prediction, 102.05)
  assert presentation.marker is reduced
  assert presentation.opacity / reference.opacity == pytest.approx(.25)


def test_ordinary_withdrawal_fades_then_cannot_resurrect(marker):
  presentation, prediction = StopBarPresentation(), road()
  settled(presentation, marker, prediction)
  before = presentation.opacity
  advance(presentation, None, prediction, 102.1, allow_fade_out=True)
  assert presentation.marker is marker
  assert presentation.opacity == pytest.approx(before / 2)
  for index in range(1, 21):
    advance(presentation, None, prediction, 102.1 + index * .05, allow_fade_out=True)
  assert presentation.marker is None
  assert presentation.opacity == 0.
  advance(presentation, None, prediction, 103.15, allow_fade_out=True)
  assert presentation.marker is None


@pytest.mark.parametrize('kind', ['ordinary', 'held', 'trajectory'])
def test_disallowed_withdrawal_clears_immediately(marker, kind):
  marker = replace(marker, held=kind == 'held', trajectory_only=kind == 'trajectory')
  presentation, prediction = StopBarPresentation(), road()
  settled(presentation, marker, prediction)
  advance(presentation, None, prediction, 102.05, allow_fade_out=kind != 'ordinary')
  assert presentation.marker is None
  assert presentation.opacity == 0.


@pytest.mark.parametrize('failure', ['new_drive', 'time_reverse', 'time_gap', 'model_reverse', 'nonfinite_time'])
def test_discontinuity_clears_cached_target(marker, failure):
  presentation, prediction = StopBarPresentation(), road()
  settled(presentation, marker, prediction)
  options = {'allow_fade_out': True}
  now = 102.05
  if failure == 'new_drive':
    options['started_frame'] = 11
  elif failure == 'time_reverse':
    now = 101.
  elif failure == 'time_gap':
    now = 102.31
  elif failure == 'model_reverse':
    options['model_time'] = 101.
  else:
    now = float('nan')
  advance(presentation, None, prediction, now, **options)
  assert presentation.marker is None
  assert presentation.opacity == 0.


def test_unknown_distance_and_position_are_not_invented(marker):
  marker = replace(marker, position=None, distance=None)
  presentation = StopBarPresentation()
  settled(presentation, marker, road())
  assert presentation.marker is marker
  assert presentation.marker.position is None
  assert presentation.marker.distance is None


def test_fade_gate_accepts_only_ordinary_confirmed_target(healthy_sm, marker):
  assert stop_bar_fade_allowed(healthy_sm, 1, 100.1, marker)
  for previous in (None, replace(marker, held=True), replace(marker, trajectory_only=True)):
    assert not stop_bar_fade_allowed(healthy_sm, 1, 100.1, previous)


def test_calibration_uses_its_slower_publication_cadence(healthy_sm, marker):
  healthy_sm.recv_time['extrinsicsCalibration'] = 99.5
  healthy_sm.logMonoTime['extrinsicsCalibration'] = 99_500_000_000
  assert stop_bar_fade_allowed(healthy_sm, 1, 100.1, marker)


@pytest.mark.parametrize('service', ['modelV2', 'carState', 'selfdriveState', 'radarState',
                                    'longitudinalPlan', 'extrinsicsCalibration'])
@pytest.mark.parametrize('failure', ['missing', 'invalid', 'old_drive', 'stale_receive', 'stale_publish',
                                    'future_receive', 'future_publish'])
def test_fade_gate_rejects_unusable_envelopes(healthy_sm, marker, service, failure):
  if failure == 'missing':
    del healthy_sm.valid[service]
  elif failure == 'invalid':
    healthy_sm.valid[service] = False
  elif failure == 'old_drive':
    healthy_sm.recv_frame[service] = 0
  elif failure == 'stale_receive':
    healthy_sm.recv_time[service] = 98.
  elif failure == 'stale_publish':
    healthy_sm.logMonoTime[service] = 98_000_000_000
  elif failure == 'future_receive':
    healthy_sm.recv_time[service] = 101.
  else:
    healthy_sm.logMonoTime[service] = 101_000_000_000
  assert not stop_bar_fade_allowed(healthy_sm, 1, 100.1, marker)


def test_renderer_preserves_current_marker_and_clears_before_calibration_early_return(healthy_sm, marker, monkeypatch):
  from openpilot.selfdrive.ui.onroad import model_renderer

  healthy_sm.updated = dict.fromkeys(healthy_sm, False)
  healthy_sm.updated['carParams'] = False
  healthy_sm['extrinsicsCalibration'].height = []
  state = NS(sm=healthy_sm, started_frame=1, status=model_renderer.UIStatus.ENGAGED, is_metric=True,
             received_since_started=lambda service: healthy_sm.recv_frame[service] >= 1)
  monkeypatch.setattr(model_renderer, 'ui_state', state)
  monkeypatch.setattr(model_renderer, 'gui_app', NS(font=lambda weight: None))
  monkeypatch.setattr(model_renderer.time, 'monotonic', lambda: 100.1)
  draw = Mock()
  monkeypatch.setattr(model_renderer, 'draw_stop_bar', draw)

  renderer = model_renderer.ModelRenderer.__new__(model_renderer.ModelRenderer)
  renderer._model_stop = NS(marker=marker, update=Mock())
  renderer._stop_bar = StopBarPresentation()
  renderer._path = NS(raw_points=np.array([healthy_sm['modelV2'].position.x,
                                         healthy_sm['modelV2'].position.y,
                                         healthy_sm['modelV2'].position.z]).T)
  renderer._longitudinal_control = False
  renderer._transform_dirty = False
  renderer._draw_lane_lines = Mock()
  renderer._draw_path = Mock()
  rect = model_renderer.rl.Rectangle(0., 0., 1920., 1080.)

  renderer._render(rect)
  assert draw.call_args.args[0] is marker
  assert draw.call_args.kwargs['path'] is renderer._path.raw_points
  assert draw.call_args.kwargs['opacity'] == renderer._stop_bar.opacity

  closer = replace(marker, position=(13.52, 0., 0.), distance=12.)
  renderer._model_stop.marker = closer
  monkeypatch.setattr(model_renderer.time, 'monotonic', lambda: 100.15)
  renderer._render(rect)
  assert draw.call_args.args[0] is closer

  renderer._model_stop.marker = None
  monkeypatch.setattr(model_renderer.time, 'monotonic', lambda: 100.2)
  renderer._render(rect)
  assert draw.call_args.args[0] is closer  # ordinary fresh withdrawal fades
  draw.reset_mock()
  healthy_sm['radarState'].leadOne.present = True
  monkeypatch.setattr(model_renderer.time, 'monotonic', lambda: 100.25)
  renderer._render(rect)
  assert renderer._stop_bar.marker is None
  draw.assert_not_called()

  renderer._model_stop.marker = closer
  healthy_sm['radarState'].leadOne.present = False
  renderer._render(rect)
  assert renderer._stop_bar.marker is closer
  draw.reset_mock()
  renderer._draw_path.reset_mock()
  healthy_sm.recv_frame['extrinsicsCalibration'] = 0
  monkeypatch.setattr(model_renderer.time, 'monotonic', lambda: 100.3)
  renderer._render(rect)
  assert renderer._stop_bar.marker is None
  assert renderer._stop_bar.opacity == 0.
  draw.assert_not_called()
  renderer._draw_path.assert_not_called()


@pytest.mark.parametrize('failure', ['reverse', 'neutral', 'gas', 'negative_speed', 'nonfinite_speed',
                                    'lead_one', 'lead_two', 'uncalibrated', 'bad_velocity', 'bad_path',
                                    'reversed_path', 'bad_orientation', 'missing_orientation'])
def test_fade_gate_rejects_unsafe_or_invalid_fresh_inputs(healthy_sm, marker, failure):
  car, prediction = healthy_sm['carState'], healthy_sm['modelV2']
  if failure in ('reverse', 'neutral'):
    car.gearShifter = failure
  elif failure == 'gas':
    car.gasPressed = True
  elif failure == 'negative_speed':
    car.vEgo = -1.
  elif failure == 'nonfinite_speed':
    car.vEgo = float('nan')
  elif failure == 'lead_one':
    healthy_sm['radarState'].leadOne.present = True
  elif failure == 'lead_two':
    healthy_sm['radarState'].leadTwo.present = True
  elif failure == 'uncalibrated':
    healthy_sm['extrinsicsCalibration'].calStatus = 'uncalibrated'
  elif failure == 'bad_velocity':
    prediction.velocity.x[3] = np.nan
  elif failure == 'bad_path':
    prediction.position.y[3] = np.nan
  elif failure == 'reversed_path':
    prediction.position.x = prediction.position.x[::-1]
  elif failure == 'bad_orientation':
    prediction.orientation.z[3] = np.nan
  else:
    prediction.orientation.y = []
  assert not stop_bar_fade_allowed(healthy_sm, 1, 100.1, marker)
