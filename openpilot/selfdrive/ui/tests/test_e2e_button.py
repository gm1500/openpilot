from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import pytest
import raylib_nb as rl

from openpilot.common.params import Params
from openpilot.selfdrive.ui.onroad import e2e_button as button
from openpilot.selfdrive.ui.onroad.gap_lane_gesture import GapLaneGesture
from openpilot.system.ui.lib.application import MouseEvent, MousePos


def event(t, kind='down', slot=0, inside=True):
  return MouseEvent(MousePos(40, 40) if inside else MousePos(400, 400), slot,
                    kind == 'press', kind == 'release', kind != 'release', t)


@pytest.fixture
def widget():
  now = [100.]
  sm = type('SM', (dict,), {})(
    selfdriveState=NS(experimentalMode=False, alwaysOnLateralSupported=True, alwaysOnLateralActive=False,
                      alwaysOnLateralSuspended=False),
    carControl=NS(longActive=False, latActive=False),
    carState=NS(vEgo=15., standstill=False))
  sm.valid = dict.fromkeys(sm, True)
  sm.recv_frame = dict.fromkeys(sm, 10)
  sm.recv_time = dict.fromkeys(sm, now[0])
  sm.logMonoTime = dict.fromkeys(sm, int(now[0] * 1e9))
  values = {'ExperimentalMode': False, 'AlwaysOnLateral': False}
  params = Mock()
  params.get_bool.side_effect = values.__getitem__
  params.get.side_effect = lambda key, **kwargs: values[key]
  ui = NS(sm=sm, started=True, started_frame=1, map_cruise_supported=True,
          experimental_mode=False, always_on_lateral=False, params=params, CP=NS(minSteerSpeed=0., steerAtStandstill=False))

  def select(name, attr, state):
    values[name] = state
    setattr(ui, attr, state)
  ui.set_experimental_mode = Mock(side_effect=lambda value: select('ExperimentalMode', 'experimental_mode', value))
  ui.set_always_on_lateral = Mock(side_effect=lambda value: select('AlwaysOnLateral', 'always_on_lateral', value))
  app, device = NS(mouse_events=[]), NS(awake=True)
  with patch.object(button, 'ui_state', ui), patch.object(button, 'gui_app', app), patch.object(button, 'device', device), \
       patch.object(button.time, 'monotonic', side_effect=lambda: now[0]):
    obj = button.E2EButton()
    obj.set_rect(rl.Rectangle(0, 0, 192, 192))

    def frame(t, *events):
      now[0] = t
      app.mouse_events = list(events)
      obj._update_state()
      obj._process_mouse_events()
    yield NS(button=obj, ui=ui, sm=sm, frame=frame, device=device, now=now, values=values)


def test_tap_changes_only_e2e_and_quick_second_tap_uses_saved_preference(widget):
  widget.frame(100.1, event(100., 'press'), event(100.1, 'release'))
  widget.ui.set_experimental_mode.assert_called_once_with(True)
  widget.ui.set_always_on_lateral.assert_not_called()
  widget.frame(100.3, event(100.2, 'press'), event(100.3, 'release'))
  assert [c.args[0] for c in widget.ui.set_experimental_mode.call_args_list] == [True, False]


def test_hold_changes_only_aol_once_and_consumes_late_release(widget):
  widget.frame(100., event(100., 'press'))
  widget.frame(100.7)
  widget.frame(104.)
  widget.frame(104.1, event(104.1, 'release'))
  widget.ui.set_always_on_lateral.assert_called_once_with(True)
  widget.ui.set_experimental_mode.assert_not_called()
  assert widget.button.is_pressed  # release cannot trigger camera/menu action
  widget.frame(104.4)
  assert not widget.button.is_pressed


def test_batched_long_press_release_is_not_an_e2e_tap(widget):
  widget.frame(101., event(100., 'press'), event(100.8, 'release'))
  widget.ui.set_always_on_lateral.assert_called_once_with(True)
  widget.ui.set_experimental_mode.assert_not_called()


def test_first_hold_turns_default_on_aol_off(widget, tmp_path):
  widget.ui.params = Params(str(tmp_path))
  assert widget.ui.params.get('AlwaysOnLateral') is None
  assert widget.ui.params.get('AlwaysOnLateral', return_default=True) is True
  widget.frame(101., event(100., 'press'), event(100.8, 'release'))
  widget.ui.set_always_on_lateral.assert_called_once_with(False)
  widget.ui.set_experimental_mode.assert_not_called()


@pytest.mark.parametrize('cancel', ['offroad', 'hide', 'sleep', 'disable', 'leave', 'multi_touch'])
def test_interrupted_contact_cannot_toggle_after_return(widget, cancel):
  widget.frame(100., event(100., 'press'))
  if cancel == 'offroad':
    widget.ui.started = False
  elif cancel == 'hide':
    widget.button.hide_event()
  elif cancel == 'sleep':
    widget.device.awake = False
  elif cancel == 'disable':
    widget.button.set_enabled(False)
  elif cancel == 'leave':
    widget.frame(100.1, event(100.1, inside=False))
  else:
    widget.frame(100.1, event(100.1, 'press', slot=1))
    widget.frame(100.2, event(100.2, 'release', slot=1))
  widget.frame(101.)
  widget.ui.started = widget.device.awake = True
  widget.button.set_enabled(True)
  widget.frame(102., event(102., 'release'))
  widget.ui.set_experimental_mode.assert_not_called()
  widget.ui.set_always_on_lateral.assert_not_called()


def test_unsupported_longitudinal_does_not_toggle_e2e(widget):
  widget.ui.map_cruise_supported = False
  widget.frame(100.1, event(100., 'press'), event(100.1, 'release'))
  widget.ui.set_experimental_mode.assert_not_called()


def test_new_drive_cannot_complete_previous_drives_contact(widget):
  widget.frame(100., event(100., 'press'))
  widget.ui.started_frame = 20  # HUD need not have rendered during the offroad interval
  widget.frame(150., event(150.))
  widget.frame(150.1, event(150.1, 'release'))
  widget.ui.set_experimental_mode.assert_not_called()
  widget.ui.set_always_on_lateral.assert_not_called()


def test_preference_never_fakes_active_control(widget):
  widget.ui.experimental_mode = widget.ui.always_on_lateral = True
  state = button.read_button_state(widget.ui, 100.)
  assert state.requested and not state.active
  assert state.lateral_supported and not state.lateral_active
  widget.sm['selfdriveState'].experimentalMode = True
  widget.sm['selfdriveState'].alwaysOnLateralActive = True
  state = button.read_button_state(widget.ui, 100.)
  assert not state.active and not state.lateral_active
  widget.sm['carControl'].longActive = widget.sm['carControl'].latActive = True
  state = button.read_button_state(widget.ui, 100.)
  assert state.active and state.lateral_active
  widget.ui.experimental_mode = False
  assert button.read_button_state(widget.ui, 100.).active and not button.read_button_state(widget.ui, 100.).requested


@pytest.mark.parametrize('invalid', ['invalid', 'old_frame', 'stale_receive', 'stale_publish', 'future_publish', 'offroad'])
def test_stale_control_does_not_show_aol_active(widget, invalid):
  widget.ui.always_on_lateral = True
  widget.sm['selfdriveState'].alwaysOnLateralActive = True
  widget.sm['carControl'].latActive = True
  if invalid == 'invalid':
    widget.sm.valid['carControl'] = False
  elif invalid == 'old_frame':
    widget.sm.recv_frame['carControl'] = 0
  elif invalid == 'stale_receive':
    widget.sm.recv_time['carControl'] = 99.
  elif invalid == 'stale_publish':
    widget.sm.logMonoTime['carControl'] = int(99. * 1e9)
  elif invalid == 'future_publish':
    widget.sm.logMonoTime['carControl'] = int(101. * 1e9)
  else:
    widget.ui.started = False
  state = button.read_button_state(widget.ui, 100.)
  assert not state.lateral_active and not state.fresh


def test_unsupported_and_cancel_suspension_are_visible(widget):
  widget.ui.always_on_lateral = True
  widget.sm['selfdriveState'].alwaysOnLateralSupported = False
  assert not button.read_button_state(widget.ui, 100.).lateral_supported
  widget.sm['selfdriveState'].alwaysOnLateralSupported = True
  widget.sm['selfdriveState'].alwaysOnLateralSuspended = True
  assert button.read_button_state(widget.ui, 100.).lateral_suspended


def test_hold_progress_matches_trigger_and_resets_on_interruption():
  gesture = GapLaneGesture()
  assert gesture.progress(100.) == 0.
  gesture.event(event(100., 'press'), True)
  assert gesture.progress(100.3) == pytest.approx(.5)
  assert gesture.progress(100.7) == 1.
  assert gesture.advance(100.7) == 'hold'
  assert gesture.progress(105.) == 1.
  gesture.event(event(105., inside=False), False)
  assert gesture.progress(105.) == 0.


@pytest.mark.parametrize('fields, hands', [
  ({'lateral_requested': False}, None),
  ({}, (255, 255, 255, 255)),
  ({'lateral_active': True}, (13, 248, 122, 255)),
  ({'lateral_suspended': True, 'lateral_active': True}, (255, 152, 0, 255)),
  ({'fresh': False, 'lateral_active': True}, (170, 180, 190, 178)),
  ({'lateral_supported': False, 'lateral_active': True}, (170, 180, 190, 178)),
])
def test_display_requires_acknowledged_support_and_activity(fields, hands):
  state = button.E2EButtonState(**({'fresh': True, 'lateral_supported': True, 'lateral_requested': True, 'lateral_available': True} | fields))
  background, hand_color, _ = button.button_colors(state)
  assert (None if hand_color is None else (hand_color.r, hand_color.g, hand_color.b, hand_color.a)) == hands
  assert (background.r, background.g, background.b, background.a) == (0, 0, 0, 166)


@pytest.mark.parametrize('fields, visible, outline', [
  ({'lateral_requested': False}, False, False),
  ({'fresh': False}, True, True),
  ({'lateral_supported': False}, True, True),
  ({'lateral_available': False}, True, True),
  ({}, True, False),
  ({'lateral_active': True}, True, False),
  ({'lateral_suspended': True, 'lateral_available': False}, True, False),
])
def test_renderer_uses_outline_for_unavailable_and_no_hands_when_off(fields, visible, outline):
  state = button.E2EButtonState(**({'fresh': True, 'lateral_supported': True, 'lateral_requested': True, 'lateral_available': True} | fields))
  app = NS(texture=Mock(return_value=NS(width=144, height=144)))
  with patch.object(button, 'gui_app', app), patch.object(button.rl, 'draw_circle'), \
       patch.object(button.rl, 'draw_texture_ex'), patch.object(button, 'draw_steering_hands') as hands:
    button.draw_e2e_button(rl.Rectangle(0, 0, 192, 192), state)
  assert hands.call_count == int(visible)
  if visible:
    assert hands.call_args.kwargs["outline"] == outline


@pytest.mark.parametrize('minimum, speed, standstill, steer_at_standstill, available', [
  (0., .29, False, False, False),
  (0., .3, False, False, False),
  (0., .301, False, False, True),
  (5., 4.99, False, False, False),
  (5., 5., False, False, False),
  (5., 5.01, False, False, True),
  (5., -5., False, False, False),
  (0., 1., True, False, False),
  (0., 0., True, True, True),
  (5., 3., False, True, True),
  (0., float('nan'), False, True, False),
])
def test_steering_availability_matches_controlsd_boundary(widget, minimum, speed, standstill, steer_at_standstill, available):
  widget.ui.always_on_lateral = True
  widget.ui.CP.minSteerSpeed = minimum
  widget.ui.CP.steerAtStandstill = steer_at_standstill
  widget.sm['carState'].vEgo = speed
  widget.sm['carState'].standstill = standstill
  widget.sm['selfdriveState'].alwaysOnLateralActive = widget.sm['carControl'].latActive = True
  state = button.read_button_state(widget.ui, 100.)
  assert state.lateral_available == available
  assert state.lateral_active == available
  assert state.hands_outline == (not available)


@pytest.mark.parametrize('invalid', ['invalid', 'old_frame', 'stale_receive', 'stale_publish', 'future_publish', 'missing_params'])
def test_unreliable_vehicle_state_cannot_show_ready_or_active(widget, invalid):
  widget.ui.always_on_lateral = True
  widget.sm['selfdriveState'].alwaysOnLateralActive = widget.sm['carControl'].latActive = True
  if invalid == 'invalid':
    widget.sm.valid['carState'] = False
  elif invalid == 'old_frame':
    widget.sm.recv_frame['carState'] = 0
  elif invalid == 'stale_receive':
    widget.sm.recv_time['carState'] = 99.
  elif invalid == 'stale_publish':
    widget.sm.logMonoTime['carState'] = int(99. * 1e9)
  elif invalid == 'future_publish':
    widget.sm.logMonoTime['carState'] = int(101. * 1e9)
  else:
    widget.ui.CP = None
  state = button.read_button_state(widget.ui, 100.)
  assert not state.lateral_available and not state.lateral_active and state.hands_outline


def test_acknowledged_pause_remains_orange_below_steering_speed(widget):
  widget.ui.always_on_lateral = True
  widget.sm['carState'].vEgo = 0.
  widget.sm['carState'].standstill = True
  widget.sm['selfdriveState'].alwaysOnLateralSuspended = True
  state = button.read_button_state(widget.ui, 100.)
  assert not state.lateral_available and not state.hands_outline
  _, hands, _ = button.button_colors(state)
  assert (hands.r, hands.g, hands.b) == (255, 152, 0)


def test_outlined_hands_never_fill_and_keep_fading_forearm_ends():
  with patch.object(button.rl, 'draw_line_ex') as lines, patch.object(button.rl, 'draw_triangle_fan') as fills:
    button.draw_steering_hands(rl.Vector2(96, 96), 1., rl.Color(170, 180, 190, 178), outline=True)
  fills.assert_not_called()
  assert len(lines.call_args_list) > 40
  assert 0 < lines.call_args_list[-1].args[-1].a < 10
