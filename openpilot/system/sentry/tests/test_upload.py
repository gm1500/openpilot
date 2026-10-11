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


def event_files(uploader, segment=0):
  from pathlib import Path
  directory = Path(uploader.root)/f'00000001--0123456789--{segment}'
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


@pytest.mark.parametrize('status', [401, 403, 412, 429, 500])
def test_sentry_upload_url_refusal_retains_http_status(uploader, monkeypatch, status):
  directory = event_files(uploader)
  response = SimpleNamespace(status_code=status, text='{"error":"upload refused"}')
  uploader.api = SimpleNamespace(get=lambda *args, **kwargs: response, get_token=lambda: 'test-token')

  def unexpected_put(*args, **kwargs):
    pytest.fail('A refused upload URL must not issue a file PUT')

  monkeypatch.setattr(upload.requests, 'put', unexpected_put)
  assert uploader.do_upload('route/ecamera.hevc', str(directory/'ecamera.hevc')) is response


@pytest.mark.parametrize('status', [401, 403, 412, 500, None])
def test_rejected_file_does_not_block_remaining_event_segments(uploader, monkeypatch, status):
  directories = [event_files(uploader, segment) for segment in range(4)]
  for directory in directories:
    for name in ('qlog.zst', 'qcamera.ts'):
      setxattr(str(directory/name), upload.UPLOAD_ATTR_NAME, upload.UPLOAD_ATTR_VALUE)
  # Match the reported route: segment zero is already on Connect, while the
  # other three segments still need all their full logs and camera files.
  for name in upload.SENTRY_UPLOAD_PRIORITY:
    if (directories[0]/name).exists():
      setxattr(str(directories[0]/name), upload.UPLOAD_ATTR_NAME, upload.UPLOAD_ATTR_VALUE)
  failed = str(directories[1]/'rlog.zst')
  attempts = []

  def response(key, filename):
    attempts.append(filename)
    if filename == failed:
      if status is None:
        raise TimeoutError('network unavailable')
      return SimpleNamespace(status_code=status)
    return SimpleNamespace(status_code=200, request=SimpleNamespace(headers={'Content-Length': '7'}))

  monkeypatch.setattr(uploader, 'do_upload', response)
  assert uploader.step(upload.NetworkType.wifi, False) is False
  for _ in range(11):
    assert uploader.step(upload.NetworkType.wifi, False) is True
  expected = {str(directory/name) for directory in directories[1:]
              for name in ('rlog.zst', 'fcamera.hevc', 'ecamera.hevc', 'dcamera.hevc')}
  assert set(attempts) == expected
  assert len(attempts) == len(expected)
  assert getxattr(failed, upload.UPLOAD_ATTR_NAME) is None
  assert uploader.next_file_to_upload(False)[2] == failed
  # A later accepted retry completes the route, without falsely marking a
  # server refusal as uploaded or needing the recorder to save another event.
  monkeypatch.setattr(uploader, 'do_upload', lambda *args: (
    SimpleNamespace(status_code=201, request=SimpleNamespace(headers={'Content-Length': '7'}))))
  assert uploader.step(upload.NetworkType.wifi, False) is True
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
