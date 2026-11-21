from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import pytest
import raylib_nb as rl

from openpilot.selfdrive.ui.onroad import e2e_button as button
from openpilot.system.ui.lib.application import MouseEvent, MousePos


def event(t, kind='down', slot=0, inside=True):
  return MouseEvent(MousePos(40, 40) if inside else MousePos(400, 400), slot,
                    kind == 'press', kind == 'release', kind != 'release', t)


@pytest.fixture
def widget():
  now = [100.]
  sm = type('SM', (dict,), {})(
    selfdriveState=NS(experimentalMode=False), carControl=NS(longActive=False))
  sm.valid = dict.fromkeys(sm, True)
  sm.recv_frame = dict.fromkeys(sm, 10)
  sm.recv_time = dict.fromkeys(sm, now[0])
  sm.logMonoTime = dict.fromkeys(sm, int(now[0] * 1e9))
  values = {'ExperimentalMode': False}
  params = Mock()
  params.get_bool.side_effect = values.__getitem__
  params.get.side_effect = lambda key, **kwargs: values[key]
  ui = NS(sm=sm, started=True, started_frame=1, map_cruise_supported=True,
          experimental_mode=False, params=params)

  def select(name, attr, state):
    values[name] = state
    setattr(ui, attr, state)
  ui.set_experimental_mode = Mock(side_effect=lambda value: select('ExperimentalMode', 'experimental_mode', value))
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
  widget.frame(100.3, event(100.2, 'press'), event(100.3, 'release'))
  assert [c.args[0] for c in widget.ui.set_experimental_mode.call_args_list] == [True, False]


def test_hold_leaves_e2e_unchanged_and_consumes_late_release(widget):
  widget.frame(100., event(100., 'press'))
  widget.frame(100.7)
  widget.frame(104.)
  widget.frame(104.1, event(104.1, 'release'))
  widget.ui.set_experimental_mode.assert_not_called()
  assert widget.button.is_pressed  # release cannot trigger camera/menu action
  widget.frame(104.4)
  assert not widget.button.is_pressed


def test_batched_long_press_release_is_not_an_e2e_tap(widget):
  widget.frame(101., event(100., 'press'), event(100.8, 'release'))
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


def test_preference_never_fakes_active_control(widget):
  widget.ui.experimental_mode = True
  assert button.read_button_state(widget.ui, 100.).requested
  assert not button.read_button_state(widget.ui, 100.).active
  widget.sm['selfdriveState'].experimentalMode = True
  assert not button.read_button_state(widget.ui, 100.).active
  widget.sm['carControl'].longActive = True
  assert button.read_button_state(widget.ui, 100.).active
  widget.ui.experimental_mode = False
  assert button.read_button_state(widget.ui, 100.).active
  assert not button.read_button_state(widget.ui, 100.).requested


@pytest.mark.parametrize('invalid', ['invalid', 'old_frame', 'stale_receive', 'stale_publish', 'future_publish', 'offroad'])
def test_stale_control_does_not_show_e2e_active(widget, invalid):
  widget.sm['selfdriveState'].experimentalMode = True
  widget.sm['carControl'].longActive = True
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
  assert not state.active and not state.fresh


@pytest.mark.parametrize('requested,active,icon', [(False, False, 'chffr_wheel.png'),
                                                 (True, False, 'experimental.png'),
                                                 (False, True, 'experimental.png')])
def test_renderer_preserves_original_mode_icon(requested, active, icon):
  state = button.E2EButtonState(requested=requested, active=active, fresh=True)
  app = NS(texture=Mock(return_value=NS(width=144, height=144)))
  with patch.object(button, 'gui_app', app), patch.object(button.rl, 'draw_circle'), patch.object(button.rl, 'draw_texture_ex'):
    button.draw_e2e_button(rl.Rectangle(0, 0, 192, 192), state)
  app.texture.assert_called_once_with('icons/' + icon, 144, 144)
