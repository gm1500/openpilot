"""Speed-limit preferences and persistent regional map management."""
from datetime import datetime
import time

from openpilot.common.params import Params
from openpilot.selfdrive.car.slc_settings import SLCSettings
from openpilot.selfdrive.mapd.offline_maps import region_catalog, region_name
from openpilot.selfdrive.mapd.map_schedule import WEEKDAYS
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog
from openpilot.system.ui.widgets.list_view import ListItem, MultipleButtonAction, button_item, text_item, toggle_item
from openpilot.system.ui.widgets.scroller_tici import Scroller


class SettingButtons(MultipleButtonAction):
  def __init__(self, params, key, labels):
    super().__init__(labels, button_width=250)
    self.params, self.key = params, key

  def is_button_selected(self, index):
    return self.params.get(self.key, return_default=True) == index

  def select_button(self, index):
    if self.enabled:
      self.params.put(self.key, index, block=True)


class OffsetButtons(MultipleButtonAction):
  def __init__(self, params):
    self.params = params
    super().__init__(['-', self.value_label, '+'], button_width=180)

  def value_label(self):
    setting = SLCSettings.read(self.params)
    unit = '%' if setting.offset_type else 'km/h' if self.params.get_bool('IsMetric') else 'mph'
    return f'{setting.offset_value:+d} {unit}'

  def is_button_selected(self, index):
    return index == 1

  def select_button(self, index):
    if self.enabled and index != 1:
      current = SLCSettings.read(self.params).offset_value
      self.params.put('SlcOffsetValue', max(-30, min(30, current + (-5 if index == 0 else 5))), block=True)


class SLCLayout(Widget):
  def __init__(self):
    super().__init__()
    self.params = Params()
    self.status = {}
    self.last_refresh = 0.
    self.enable = toggle_item('Speed Limit Control',
      'Use the mapped speed as the cruise target on supported cars. The onroad SLC button controls the same setting.',
      initial_state=self.params.get('MapCruiseEnabled', return_default=True),
      callback=lambda value: self.params.put_bool('MapCruiseEnabled', value, block=True))
    self.advance = toggle_item('Advanced speed limit changes',
      'Predict mapped speed limit changes up to 2 seconds ahead. Begin slowing for lower limits early and apply higher limits ' +
      'when entering the new road segment. Requires a confident road match; existing override settings still apply.',
      initial_state=self.params.get('SlcAdvancedSpeedLimit', return_default=True),
      callback=lambda value: self.params.put_bool('SlcAdvancedSpeedLimit', value, block=True))
    self.region_buttons = MultipleButtonAction(['North America', 'Country'], button_width=310, callback=self.select_region)
    self.region_buttons.is_button_selected = lambda i: self.region().startswith(('region.', 'nation.')[i])
    self.actions = MultipleButtonAction(['Download', 'Update', 'Delete'], button_width=240, callback=self.map_action)
    self.actions.is_button_selected = lambda i: False
    self.cancel = button_item('Map download', 'CANCEL', callback=lambda: self.request('cancel'))
    self.update_day = button_item('Weekly update day', self.weekday_label,
      'If the device is off or away from Wi-Fi, update at the next parked Wi-Fi connection.', callback=self.select_weekday)
    self.items = [
      self.enable,
      self.advance,
      ListItem('New speed limit', action_item=SettingButtons(self.params, 'SlcChangePolicy', ['Automatic', 'Lower only', 'SET to accept']),
        description='Automatic applies qualified new limits. Lower only never raises the current target automatically. ' +
                    'SET to accept waits for an explicit map SET or re-engagement with SET. Ordinary +/- buttons remain manual.'),
      ListItem('After manual override', action_item=SettingButtons(self.params, 'SlcResumePolicy', ['Next limit', 'SET only']),
        description='Next limit allows SLC to resume at a changed road limit, subject to the rule above. ' +
                    'SET only preserves your manual choice until you explicitly select the map target again.'),
      ListItem('Offset type', action_item=SettingButtons(self.params, 'SlcOffsetType',
        [lambda: 'km/h' if self.params.get_bool('IsMetric') else 'mph', 'Percent']),
        description='Choose a fixed speed offset or a percentage of the posted limit.'),
      ListItem('Speed offset', action_item=OffsetButtons(self.params),
        description='Adjust in steps of 5, from -30 to +30. Zero uses the posted limit. Only the cruise target changes; ' +
                    'the displayed road limit stays unchanged. Existing cruise bounds still apply.'),
      ListItem('Map area', action_item=self.region_buttons),
      text_item('Selected area', self.selected_label),
      ListItem('Offline maps', action_item=self.actions,
        description='Download once and keep maps across reboots. Installed maps supply speed limits when live map data is unavailable. ' +
                    'Live data still supplies richer junction information. Large regions can require several GB. ' +
                    'Downloads run while parked on an unmetered connection. Turn off Cellular Metered in network settings to allow cellular.'),
      self.cancel,
      ListItem('Automatic map updates', action_item=SettingButtons(self.params, 'OfflineMapUpdateSchedule', ['Manual', 'Daily', 'Weekly']),
        description='Refresh all fully downloaded areas while parked on unmetered Wi-Fi. ' +
                    'Daily runs once per calendar day; Weekly runs on the selected weekday, or the next available connection. ' +
                    'Uses your saved local timezone, or UTC until it is known. Manual is the default.'),
      self.update_day,
      text_item('Download status', self.progress_label,
                lambda: self.status.get('error') or 'Progress counts validated map groups. Download resumes completed groups after interruption.'),
      text_item('Map storage', lambda: f'{self.status.get("installed_bytes", 0) / 1024**3:.2f} GB',
        'Keeps at least 5 GB and 10% of storage free for recordings. Interrupted downloads retain completed map groups.'),
      text_item('Last update', self.date_label,
        'This is the last completed download date, not the age of every road in the source database. Update refreshes the selected area.'),
      text_item('Map data', 'OpenStreetMap contributors',
        'Open Database License: openstreetmap.org/copyright. Regional packages provided by map-data.pfeifer.dev.'),
    ]
    self.scroller = self._child(Scroller(self.items, spacing=0, line_separator=True))

  def region(self):
    return self.params.get('OfflineMapRegion', return_default=True) or 'nation.CA'

  def selected_label(self):
    return region_name(self.region())

  def select_region(self, index):
    if index == 0:
      self.params.put('OfflineMapRegion', 'region.NA', block=True)
      return
    options = {v['full_name']: f'nation.{k}' for k,v in region_catalog()['nation'].items()}
    def selected(result):
      if result == DialogResult.CONFIRM and dialog.selection in options:
        self.params.put('OfflineMapRegion', options[dialog.selection], block=True)
    dialog = MultiOptionDialog('Choose country', sorted(options), current=self.selected_label(), callback=selected)
    gui_app.push_widget(dialog)

  def request(self, action):
    self.params.put('OfflineMapRequest', {'action': action, 'region': self.region(), 'id': time.time_ns()}, block=True)

  def weekday_label(self):
    day = self.params.get('OfflineMapUpdateWeekday', return_default=True)
    return WEEKDAYS[day] if day in range(7) else WEEKDAYS[0]

  def select_weekday(self):
    def selected(result):
      if result == DialogResult.CONFIRM and dialog.selection in WEEKDAYS:
        self.params.put('OfflineMapUpdateWeekday', WEEKDAYS.index(dialog.selection), block=True)
    dialog = MultiOptionDialog('Weekly update day', list(WEEKDAYS), current=self.weekday_label(), callback=selected)
    gui_app.push_widget(dialog)

  def map_action(self, index):
    action = ('download', 'update', 'delete')[index]
    name = self.selected_label()
    if action == 'delete':
      message = f'Delete downloaded maps for {name}? Map groups shared with other installed areas are kept.'
    else:
      message = (f'{"Update" if action == "update" else "Download"} {name}? This may use several GB and will run while parked on an unmetered connection. ' +
                 'Cellular is allowed when Cellular Metered is off.')
    gui_app.push_widget(ConfirmDialog(message, action.capitalize(), callback=lambda result:
                                     self.request(action) if result == DialogResult.CONFIRM else None))

  def progress_label(self):
    phase = self.status.get('phase', 'Starting map manager')
    if self.status.get('error'):
      return phase
    total, completed = self.status.get('total', 0), self.status.get('completed', 0)
    return f'{phase}: {completed}/{total}' if total else phase

  def date_label(self):
    info = self.status.get('regions', {}).get(self.region(), {})
    stamp = info.get('updated_at')
    if stamp:
      return datetime.fromtimestamp(stamp).strftime('%Y-%m-%d')
    return 'Partial download' if info else 'Not downloaded'

  def _update_state(self):
    if time.monotonic()-self.last_refresh < .5:
      return
    self.last_refresh = time.monotonic()
    self.status = self.params.get('OfflineMapStatus') or {}
    busy = self.status.get('active', False) or bool(self.params.get('OfflineMapRequest'))
    self.region_buttons.set_enabled(not busy)
    self.actions.set_enabled(not busy)
    self.cancel.set_visible(busy)
    self.update_day.set_visible(self.params.get('OfflineMapUpdateSchedule', return_default=True) == 2)
    self.enable.action_item.set_state(self.params.get('MapCruiseEnabled', return_default=True))
    self.advance.action_item.set_state(self.params.get('SlcAdvancedSpeedLimit', return_default=True))

  def _render(self, rect):
    self.scroller.render(rect)

  def show_event(self):
    super().show_event()
    self.scroller.show_event()
