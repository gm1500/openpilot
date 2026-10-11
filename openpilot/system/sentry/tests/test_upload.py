from types import SimpleNamespace

import pytest

from openpilot.common.params import Params
from openpilot.system.loggerd import uploader as upload
from openpilot.system.loggerd.xattr_cache import getxattr, setxattr


@pytest.fixture
def uploader(tmp_path, monkeypatch):
  params = Params(str(tmp_path/'params'))
  monkeypatch.setattr(upload, 'Params', lambda: params)
  monkeypatch.setattr(upload, 'Api', lambda _: None)
  root = tmp_path/'routes'
  root.mkdir()
  return upload.Uploader('test-device', str(root))


def event_files(uploader):
  from pathlib import Path
  directory = Path(uploader.root)/'00000001--0123456789--0'
  directory.mkdir()
  (directory/'.sentry').write_text('1')
  for name in ('qlog.zst', 'rlog.zst', 'qcamera.ts', 'fcamera.hevc', 'ecamera.hevc', 'dcamera.hevc'):
    (directory/name).write_bytes(b'footage')
  return directory


def test_auto_upload_all_cameras_after_unlock_and_resume_after_restart(uploader, monkeypatch):
  directory = event_files(uploader)
  lock = directory/'sentry.lock'
  lock.touch()
  assert uploader.next_file_to_upload(False) is None
  lock.unlink()
  order = []
  monkeypatch.setattr(upload.Uploader, 'do_upload', lambda self, key, fn: (
    order.append(key.rsplit('/', 1)[-1]) or SimpleNamespace(status_code=200, request=SimpleNamespace(headers={'Content-Length': '7'}))))
  assert uploader.step(upload.NetworkType.wifi, False)
  assert uploader.step(upload.NetworkType.wifi, False)
  # Restart after preview/log upload. Remaining files are discovered from disk.
  restarted = upload.Uploader('test-device', uploader.root)
  while restarted.step(upload.NetworkType.wifi, False):
    pass
  assert order == ['qlog.zst', 'qcamera.ts', 'rlog.zst', 'fcamera.hevc', 'ecamera.hevc', 'dcamera.hevc']
  assert restarted.next_file_to_upload(False) is None
  for name in order:
    assert getxattr(str(directory/name), upload.UPLOAD_ATTR_NAME) == upload.UPLOAD_ATTR_VALUE


def test_full_video_waits_for_unmetered_and_does_not_upload_ordinary_routes(uploader):
  directory = event_files(uploader)
  setxattr(str(directory/'qlog.zst'), upload.UPLOAD_ATTR_NAME, upload.UPLOAD_ATTR_VALUE)
  assert uploader.next_file_to_upload(True) is None
  assert {name for name, _, _ in uploader.list_upload_files(True)}.isdisjoint(upload.SENTRY_UPLOAD_PRIORITY)
  setxattr(str(directory/'qcamera.ts'), upload.UPLOAD_ATTR_NAME, upload.UPLOAD_ATTR_VALUE)
  assert uploader.next_file_to_upload(False)[0] == 'rlog.zst'
  (directory/'.sentry').unlink()
  assert uploader.next_file_to_upload(False) is None


@pytest.mark.parametrize('status', [401, 403, 412, 500, None])
def test_refused_or_failed_upload_remains_pending(uploader, monkeypatch, status):
  directory = event_files(uploader)
  fn = str(directory/'ecamera.hevc')

  def response(key, filename):
    if status is None:
      raise TimeoutError('network unavailable')
    return SimpleNamespace(status_code=status)

  monkeypatch.setattr(uploader, 'do_upload', response)
  assert not uploader.upload('ecamera.hevc', 'route/ecamera.hevc', fn, upload.NetworkType.wifi, False)
  assert getxattr(fn, upload.UPLOAD_ATTR_NAME) is None
  assert any(candidate[2] == fn for candidate in uploader.list_upload_files(False))
