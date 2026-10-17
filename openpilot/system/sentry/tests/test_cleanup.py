import os
from pathlib import Path
import signal

import pytest

from tools import sentry_clear_recordings as cleanup
from tools.sentry_clear_recordings import completed_segments, delete_segments, module_process


def segment(root, name, lock=False):
  path = root/name
  path.mkdir()
  (path/'qlog.zst').write_bytes(b'old data')
  if lock:
    (path/'sentry.lock').touch()
  return path


def test_cleanup_keeps_active_locked_and_unknown_directories(tmp_path):
  old = segment(tmp_path, '00000001--0123456789--0')
  legacy = segment(tmp_path, '2026-10-01--12-30-00--5')
  active_route = '00000002--0123456789'
  active = segment(tmp_path, active_route+'--0')
  locked = segment(tmp_path, '00000003--0123456789--1', lock=True)
  locked_earlier = segment(tmp_path, '00000003--0123456789--0')
  boot = segment(tmp_path, 'boot')
  crash = segment(tmp_path, 'crash')
  unknown = segment(tmp_path, 'other-data')
  assert set(completed_segments(tmp_path, {active_route})) == {old, legacy}
  assert delete_segments(tmp_path, {active_route}) == 2
  assert not old.exists() and not legacy.exists()
  for path in (active, locked, locked_earlier, boot, crash, unknown):
    assert (path/'qlog.zst').read_bytes() == b'old data'


def test_cleanup_does_not_follow_symlinks(tmp_path):
  root = tmp_path/'routes'
  root.mkdir()
  outside = segment(tmp_path, 'outside')
  (root/'00000001--0123456789--0').symlink_to(outside, target_is_directory=True)
  old = segment(root, '00000002--0123456789--0')
  (old/'external').symlink_to(outside, target_is_directory=True)
  assert delete_segments(root, set()) == 1
  assert (outside/'qlog.zst').exists()
  assert (root/'00000001--0123456789--0').is_symlink()


def test_module_process_checks_exact_module(monkeypatch):
  monkeypatch.setattr(Path, 'read_bytes', lambda _: b'python\0-m\0openpilot.system.athena.manage_athenad\0')
  assert module_process(os.getpid(), 'openpilot.system.athena.manage_athenad')
  assert not module_process(os.getpid(), 'openpilot.system.athena.athenad')


def test_queue_cannot_be_saved_back_and_supervisor_resumes_if_reboot_fails(monkeypatch):
  actions = []

  class Params:
    def get(self, key):
      assert key == 'AthenadPid'
      return 101

    def put(self, key, value, block=False):
      actions.append(('put', key, value, block))

  monkeypatch.setattr(cleanup, 'module_process', lambda pid, module: (pid, module) in
                      ((101, cleanup.ATHENA_MANAGER), (102, cleanup.ATHENA)))
  monkeypatch.setattr(Path, 'read_text', lambda _: 'State:\tT (stopped)\n')
  monkeypatch.setattr(Path, 'iterdir', lambda _: iter([Path('/proc/101'), Path('/proc/102')]))
  monkeypatch.setattr(cleanup.os, 'kill', lambda pid, sig: actions.append(('kill', pid, sig)))
  monkeypatch.setattr(cleanup.os, 'sync', lambda: actions.append(('sync',)))

  def failed_reboot(args, check):
    actions.append(('reboot', args, check))
    raise RuntimeError('test reboot failure')

  monkeypatch.setattr(cleanup.subprocess, 'run', failed_reboot)
  with pytest.raises(RuntimeError, match='test reboot failure'):
    cleanup.clear_requests_and_reboot(Params())
  assert actions == [
    ('kill', 101, signal.SIGSTOP), ('kill', 102, signal.SIGKILL),
    ('put', 'AthenadUploadQueue', [], True), ('sync',),
    ('reboot', ['sudo', '-n', 'reboot'], True), ('kill', 101, signal.SIGCONT),
  ]
