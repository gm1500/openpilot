from types import SimpleNamespace as NS
from unittest.mock import Mock

import raylib_nb as rl

from openpilot.selfdrive.ui.onroad import speed_limit as ui
from openpilot.selfdrive.ui.onroad.lane_policy_style import boundary_colors, PAINT_GREEN


def test_candidate_display_has_no_independent_warning_and_pulse_tracks_controller(monkeypatch):
  state = NS(map_cruise_state='active', map_cruise_enabled=True, map_cruise_supported=True, map_cruise_pulsing=False,
             map_cruise_e2e=False, engaged=True, speed_limit=80/3.6, speed_limit_is_advisory=False,
             speed_limit_preview=100/3.6, speed_limit_preview_advisory=False, is_metric=True,
             sm={'mapSpeedLimit': NS(positionEstimated=False, zoneType='none')})
  monkeypatch.setattr(ui, 'ui_state', state)
  draw = Mock()
  monkeypatch.setattr(ui, 'draw_speed_limit', draw)
  now = [100.]
  monkeypatch.setattr(ui.rl, 'get_time', lambda: now[0])
  button = ui.SpeedLimitButton()
  rect = rl.Rectangle(0, 0, 180, 204)
  button._render(rect)
  assert draw.call_args.args[1] == 100/3.6
  assert draw.call_args.args[6] is None  # preview alone never starts a warning
  now[0] = 103.
  button._render(rect)
  assert draw.call_args.args[6] is None
  state.map_cruise_pulsing = True
  button._render(rect)
  assert draw.call_args.args[6] == 0.  # controller adjustment is shown immediately
  now[0] += .25
  button._render(rect)
  assert draw.call_args.args[6] == .25
  state.speed_limit_preview = 60/3.6
  now[0] = 104.
  button._render(rect)
  assert draw.call_args.args[1] == 60/3.6
  assert draw.call_args.args[6] == 0.
  # Cancellation takes effect immediately even for a newly displayed limit.
  state.map_cruise_pulsing = False
  now[0] += .1
  button._render(rect)
  assert draw.call_args.args[6] is None
  state.speed_limit_preview = 50/3.6
  button._render(rect)
  assert draw.call_args.args[1] == 50/3.6
  assert draw.call_args.args[6] is None


def test_no_limit_icon_has_no_maximum_or_units_and_candidate_exits_icon(monkeypatch):
  texts = []
  monkeypatch.setattr(ui.gui_app, 'font', lambda *args: rl.Font())
  monkeypatch.setattr(ui.gui_app, 'texture', lambda *args: NS(width=100, height=100))
  monkeypatch.setattr(ui, 'measure_text_cached', lambda *args: rl.Vector2(100, 30))
  for name in ('draw_rectangle_rounded', 'draw_rectangle_rounded_lines_ex', 'draw_texture_ex', 'draw_rectangle_rec'):
    monkeypatch.setattr(ui.rl, name, lambda *args: None)
  monkeypatch.setattr(ui.rl, 'draw_text_ex', lambda font, value, *args: texts.append(value))
  ui.draw_speed_limit(rl.Rectangle(0, 0, 180, 204), None, 105, True, rl.WHITE, e2e_override=True)
  assert 'OVERRIDE' in texts and 'NO MAP LIMIT' in texts
  assert not {'MAXIMUM', 'km/h', 'SPEED LIMIT', 'mph'} & set(texts)


def test_painted_policy_never_needs_edge_or_memory_telemetry():
  state = NS(valid=True, enabled=True, active=True, leftSource='painted', rightSource='none')
  assert boundary_colors(state, 0, True) == (PAINT_GREEN, None)
  assert boundary_colors(state, 1, True) == (None, None)
