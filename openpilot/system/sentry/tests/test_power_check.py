from pathlib import Path

import zstandard

from openpilot.cereal import messaging
from openpilot.tools.lib.logreader import LogReader
from tools.sentry_power_check import read_current_params, boot_snapshot, print_report


def test_reads_only_power_settings_without_changing_them(tmp_path):
  (tmp_path/'ForcePowerDown').write_text('1')
  (tmp_path/'SentryEnabled').write_text('1')
  (tmp_path/'AccessToken').write_text('do-not-print')
  before = {p.name: p.read_bytes() for p in tmp_path.iterdir()}
  snapshot = read_current_params(tmp_path)
  assert snapshot['ForcePowerDown'] == '1'
  assert snapshot['DoShutdown'] == '(unset)'
  assert 'AccessToken' not in snapshot
  assert {p.name: p.read_bytes() for p in tmp_path.iterdir()} == before


def test_saved_boot_snapshot_survives_cleared_current_flags_and_filters_credentials(tmp_path):
  msg = messaging.new_message('initData')
  entries = msg.initData.init('params').init('entries', 3)
  for entry, key, value in zip(entries, ('DoShutdown', 'LastManagerExitReason', 'AccessToken'),
                                (b'1', b'DoShutdown at previous boot', b'do-not-print'), strict=True):
    entry.key, entry.value = key, value
  path = tmp_path/'boot.zst'
  path.write_bytes(zstandard.ZstdCompressor().compress(msg.to_bytes()))
  assert read_current_params(tmp_path)['DoShutdown'] == '(unset)'
  assert boot_snapshot(path, LogReader) == {'DoShutdown': '1', 'LastManagerExitReason': 'DoShutdown at previous boot'}


def test_report_does_not_disclose_location_and_handles_absent_bootlogs(tmp_path, capsys):
  runtime = tmp_path/'runtime.json'
  runtime.write_text('{"session": true, "phase": "Battery cutoff", "location": {"latitude": 53}}')
  print_report(tmp_path, runtime, tmp_path/'missing', count=3)
  output = capsys.readouterr().out
  assert 'Battery cutoff' in output and 'No saved bootlogs' in output
  assert 'latitude' not in output
  assert runtime.exists() and not Path(tmp_path/'missing').exists()
