import pyray as rl

from openpilot.common.params import Params
from openpilot.selfdrive.ui.layouts.settings import slc as ui
from openpilot.system.ui.widgets import DialogResult, list_view


def test_schedule_controls_persist_weekday_and_keep_manual_download_actions(monkeypatch, tmp_path):
  params = Params(str(tmp_path/'params'))
  dialogs = []
  monkeypatch.setattr(ui, 'Params', lambda: params)
  monkeypatch.setattr(ui.gui_app, 'font', lambda *args: rl.Font())
  monkeypatch.setattr(ui.gui_app, 'push_widget', dialogs.append)
  monkeypatch.setattr(list_view, 'measure_text_cached', lambda font, text, size: rl.Vector2(len(text)*20, 40))
  layout = ui.SLCLayout()
  schedule = next(row for row in layout.items if row.title == 'Automatic map updates').action_item
  layout._update_state()
  assert schedule.is_button_selected(0)
  assert not layout.update_day.is_visible
  schedule.select_button(2)
  layout.last_refresh = 0.
  layout._update_state()
  assert layout.update_day.is_visible
  layout.select_weekday()
  dialog = dialogs[-1]
  assert len(dialog.options) == 7 and dialog.current == 'Monday'
  dialog.selection = 'Sunday'
  dialog._callback(DialogResult.CONFIRM)
  assert params.get('OfflineMapUpdateWeekday') == 6
  assert layout.weekday_label() == 'Sunday'
  schedule.select_button(1)
  layout.last_refresh = 0.
  layout._update_state()
  assert not layout.update_day.is_visible
  layout.request('update')
  assert params.get('OfflineMapRequest')['action'] == 'update'
  assert params.get('OfflineMapRequest')['region'] == 'nation.CA'
  assert params.get('OfflineMapUpdateSchedule') == 1

