from datetime import datetime, UTC
from pathlib import Path
import os
import shlex
import shutil
import subprocess
from types import SimpleNamespace

import pytest

from openpilot.common.params import Params
from openpilot.system.sentry import clock


def test_offline_timezone_persists_and_handles_dst_without_gps(tmp_path):
  params = Params(str(tmp_path/'params'))
  current = clock.SentryClock(params)
  current.update({'latitude': 53.5, 'longitude': -113.5}, 100.)
  current._worker.join(timeout=30)
  assert not current._worker.is_alive()
  assert params.get('SentryTimeZone') == 'America/Edmonton'
  restarted = clock.SentryClock(params)
  # A garage start can use the timezone even when no parking coordinates or GPS
  # fix are available. The current clock, not the checkpoint date, sets the date.
  for month, offset in ((1, -7*3600), (7, -6*3600)):
    wall = datetime(2026, month, 11, 8, 0, tzinfo=UTC).timestamp()
    restarted.update(None, 101.)
    assert restarted.snapshot(wall) == ('America/Edmonton', offset)
  assert restarted._worker is None


def test_timezone_failure_is_utc_and_cannot_stop_capture(tmp_path, monkeypatch):
  params = Params(str(tmp_path/'params'))
  params.put('SentryTimeZone', 'invalid/zone', block=True)
  current = clock.SentryClock(params)
  assert current.snapshot(10000.) == ('', 0)
  assert clock.time_status(params).endswith(' UTC')
  monkeypatch.setattr(clock.cloudlog, 'exception', lambda *args: None)

  def fail(**kwargs):
    raise ValueError('optional timezone data unavailable')

  current._finder = SimpleNamespace(timezone_at=fail)
  current.update({'latitude': 53.5, 'longitude': -113.5}, 100.)
  worker = current._worker
  worker.join(timeout=2)
  assert current.snapshot(10000.) == ('', 0)
  current.update({'latitude': 53.5, 'longitude': -113.5}, 101.)
  assert current._worker is worker  # Back off failed lookups, never busy-loop.


def test_native_watermark_timestamp_and_utc_fallback(tmp_path):
  compiler = shlex.split(os.environ.get('CXX', 'c++'))
  if not compiler or not shutil.which(compiler[0]):
    pytest.skip('native timestamp check requires a C++ compiler')
  root = Path(__file__).resolve().parents[4]
  source, binary = tmp_path/'time.cc', tmp_path/'time'
  source.write_text('''
#include <cassert>
#include <climits>
#include "openpilot/system/sentry/timestamp.h"
int main() {
  assert(sentry_timestamp(1791705600, false, -21600) == "2026-10-11 08:00:00 UTC");
  assert(sentry_timestamp(1791705600, true, -21600) == "2026-10-11 02:00:00 -0600");
  assert(sentry_timestamp(1791705600, true, 19800) == "2026-10-11 13:30:00 +0530");
  assert(sentry_timestamp(1791705600, true, -1800) == "2026-10-11 07:30:00 -0030");
  assert(sentry_timestamp(1791705600, true, INT_MIN) == "2026-10-11 08:00:00 UTC");
}
''')
  subprocess.run([*compiler, '-std=c++17', '-Wall', '-Werror', '-I'+str(root), str(source), '-o', str(binary)], check=True)
  subprocess.run([str(binary)], check=True)
