import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.common.params import Params
from openpilot.system.loggerd import uploader as upload
from openpilot.system.loggerd.xattr_cache import getxattr, setxattr

FULL_FILES = ('rlog.zst', 'fcamera.hevc', 'ecamera.hevc', 'dcamera.hevc')


@pytest.fixture
def uploader(tmp_path, monkeypatch):
  params = Params(str(tmp_path/'params'))
  params.put_bool('IsOffroad', True, block=True)
  monkeypatch.setattr(upload, 'Params', lambda: params)
  monkeypatch.setattr(upload, 'Api', lambda _: None)
  monkeypatch.setattr(upload.Paths, 'persist_root', lambda: str(tmp_path/'persist'))
  root = tmp_path/'routes'
  root.mkdir()
  return upload.Uploader('test-device', str(root))


def event_files(uploader, segment=0):
  directory = Path(uploader.root)/f'00000001--0123456789--{segment}'
  directory.mkdir()
  (directory/'.sentry').write_text('1')
  for name in ('qlog.zst', 'qcamera.ts', *FULL_FILES):
    (directory/name).write_bytes(b'footage')
  return directory


@pytest.mark.parametrize('metered', [False, True])
@pytest.mark.parametrize('sentry', [False, True])
def test_only_preview_and_qlog_upload_full_files_remain_local_after_restart(uploader, monkeypatch, metered, sentry):
  directory = event_files(uploader)
  if not sentry:
    (directory/'.sentry').unlink()
  lock = directory/'sentry.lock'
  lock.touch()
  assert uploader.next_file_to_upload(metered) is None
  lock.unlink()
  order = []
  monkeypatch.setattr(upload.Uploader, 'do_upload', lambda self, key, fn: (
    order.append(key.rsplit('/', 1)[-1]) or SimpleNamespace(status_code=200, request=SimpleNamespace(headers={'Content-Length': '7'}))))
  assert uploader.step(upload.NetworkType.wifi, metered)
  restarted = upload.Uploader('test-device', uploader.root)
  while restarted.step(upload.NetworkType.wifi, metered):
    pass
  assert order == (['qlog.zst'] if metered else ['qlog.zst', 'qcamera.ts'])
  for name in order:
    assert getxattr(str(directory/name), upload.UPLOAD_ATTR_NAME) == upload.UPLOAD_ATTR_VALUE
  # Viewing a route permits the stock metered preview policy, never full files.
  restarted.params.put('AthenadRecentlyViewedRoutes', 'test-device|00000001--0123456789', block=True)
  while restarted.step(upload.NetworkType.wifi, metered):
    pass
  assert order == ['qlog.zst', 'qcamera.ts']
  assert restarted.next_file_to_upload(False) is None
  for name in FULL_FILES:
    assert (directory/name).read_bytes() == b'footage'
    assert getxattr(str(directory/name), upload.UPLOAD_ATTR_NAME) is None


def test_old_pending_full_files_do_not_block_new_event_previews(uploader, monkeypatch):
  old = event_files(uploader)
  for name in ('qlog.zst', 'qcamera.ts', 'fcamera.hevc'):
    setxattr(str(old/name), upload.UPLOAD_ATTR_NAME, upload.UPLOAD_ATTR_VALUE)
  new = event_files(uploader, segment=1)
  order = []
  monkeypatch.setattr(uploader, 'do_upload', lambda key, fn: (
    order.append(key) or SimpleNamespace(status_code=200, request=SimpleNamespace(headers={'Content-Length': '7'}))))
  while uploader.step(upload.NetworkType.wifi, False):
    pass
  assert order == [new.name+'/qlog.zst', new.name+'/qcamera.ts']
  assert all((old/name).exists() for name in FULL_FILES)


@pytest.mark.parametrize('metered', [False, True])
def test_gap_minute_uploads_only_its_qlog_once(uploader, monkeypatch, metered):
  directory = Path(uploader.root)/'00000001--0123456789--1'
  directory.mkdir()
  (directory/'.sentry').write_text('1')
  (directory/'qlog.zst').write_bytes(b'gap metadata')
  lock = directory/'sentry.lock'
  lock.touch()
  assert uploader.next_file_to_upload(metered) is None
  lock.unlink()
  uploaded = []
  monkeypatch.setattr(uploader, 'do_upload', lambda key, fn: (
    uploaded.append(key) or SimpleNamespace(status_code=200, request=SimpleNamespace(headers={'Content-Length': '12'}))))
  assert uploader.step(upload.NetworkType.wifi, metered)
  assert uploaded == [directory.name+'/qlog.zst']
  assert uploader.next_file_to_upload(metered) is None
  restarted = upload.Uploader('test-device', uploader.root)
  assert restarted.next_file_to_upload(False) is None, 'absent video must not cause upload attempts'
  assert {f.name for f in directory.iterdir()} == {'.sentry', 'qlog.zst'}


@pytest.mark.parametrize('status', [401, 403, 412, 429, 500])
def test_sentry_preview_url_refusal_retains_http_status(uploader, monkeypatch, status):
  directory = event_files(uploader)
  response = SimpleNamespace(status_code=status, text='{"error":"upload refused"}')
  uploader.api = SimpleNamespace(get=lambda *args, **kwargs: response, get_token=lambda: 'test-token')
  monkeypatch.setattr(upload.requests, 'put', lambda *args, **kwargs: pytest.fail('Refused URL must not cause a PUT'))
  assert uploader.do_upload(directory.name+'/qcamera.ts', str(directory/'qcamera.ts')) is response


@pytest.mark.parametrize('sentry', [False, True])
def test_preview_keeps_ordinary_device_upload_endpoint(uploader, monkeypatch, sentry):
  directory = event_files(uploader)
  if not sentry:
    (directory/'.sentry').unlink()
  calls = []

  def get(endpoint, **kwargs):
    calls.append((endpoint, kwargs))
    return SimpleNamespace(status_code=200, text=json.dumps({'url': 'https://storage.example/preview', 'headers': {}}))

  uploader.api = SimpleNamespace(get=get, get_token=lambda: 'test-device-token')
  monkeypatch.setattr(upload.requests, 'post', lambda *args, **kwargs: pytest.fail('Do not request batch upload URLs'))
  monkeypatch.setattr(upload.requests, 'put', lambda *args, **kwargs: SimpleNamespace(status_code=200))
  assert uploader.do_upload(directory.name+'/qcamera.ts', str(directory/'qcamera.ts')).status_code == 200
  assert calls == [('v1.4/test-device/upload_url/',
                    {'timeout': 10, 'path': directory.name+'/qcamera.ts', 'access_token': 'test-device-token'})]


@pytest.mark.parametrize('status', [401, 403, 412, 500, None])
def test_failed_preview_does_not_block_remaining_event_segments(uploader, monkeypatch, status):
  clock = [100.]
  monkeypatch.setattr(upload.time, 'monotonic', lambda: clock[0])
  directories = [event_files(uploader, segment) for segment in range(3)]
  failed = str(directories[0]/'qlog.zst')
  attempts = []

  def response(key, filename):
    attempts.append(filename)
    clock[0] += .1
    if filename == failed:
      if status is None:
        raise TimeoutError('network unavailable')
      return SimpleNamespace(status_code=status)
    return SimpleNamespace(status_code=200, request=SimpleNamespace(headers={'Content-Length': '7'}))

  monkeypatch.setattr(uploader, 'do_upload', response)
  assert uploader.step(upload.NetworkType.wifi, False) is False
  for _ in range(5):
    assert uploader.step(upload.NetworkType.wifi, False) is True
  expected = {str(directory/name) for directory in directories for name in ('qlog.zst', 'qcamera.ts')}
  assert set(attempts) == expected
  assert len(attempts) == len(expected)
  assert getxattr(failed, upload.UPLOAD_ATTR_NAME) is None
  if status in (401, 403, 412):
    # The five successful uploads above must not reset this file's cooldown.
    assert uploader.next_file_to_upload(False) is None
    clock[0] = uploader.sentry_denied_until[failed] - .1
    assert uploader.next_file_to_upload(False) is None
    clock[0] = uploader.sentry_denied_until[failed]
  assert uploader.next_file_to_upload(False)[2] == failed
  def recovered(*args):
    clock[0] += .1
    return SimpleNamespace(status_code=201, request=SimpleNamespace(headers={'Content-Length': '7'}))
  monkeypatch.setattr(uploader, 'do_upload', recovered)
  assert uploader.step(upload.NetworkType.wifi, False) is True
  assert uploader.next_file_to_upload(False) is None
  assert all((directory/name).exists() for directory in directories for name in FULL_FILES)


def test_uploader_removes_only_obsolete_sentry_account_credential(uploader):
  directory = Path(upload.Paths.persist_root())/'comma'
  directory.mkdir(parents=True)
  credential = directory/'sentry_connect.json'
  credential.write_text('old-secret')
  device_key = directory/'id_rsa'
  device_key.write_text('device-key')
  upload.Uploader('test-device', uploader.root)
  assert not credential.exists()
  assert device_key.read_text() == 'device-key'


def test_credential_cleanup_failure_does_not_stop_preview_uploads(uploader, monkeypatch):
  def denied(path, **kwargs):
    raise PermissionError('read-only persist')
  monkeypatch.setattr(Path, 'unlink', denied)
  restarted = upload.Uploader('test-device', uploader.root)
  directory = event_files(restarted)
  assert restarted.next_file_to_upload(False)[2] == str(directory/'qlog.zst')


def test_driving_holds_sentry_and_resumes_offroad_without_blocking_regular_files(uploader):
  sentry = event_files(uploader)
  regular = event_files(uploader, 1)
  (regular/'.sentry').unlink()
  uploader.params.put_bool('IsOffroad', False, block=True)
  assert uploader.next_file_to_upload(False)[2] == str(regular/'qlog.zst')
  for name in ('qlog.zst', 'qcamera.ts'):
    setxattr(str(regular/name), upload.UPLOAD_ATTR_NAME, upload.UPLOAD_ATTR_VALUE)
  assert uploader.next_file_to_upload(False) is None
  assert all(getxattr(str(sentry/name), upload.UPLOAD_ATTR_NAME) is None for name in ('qlog.zst', 'qcamera.ts', *FULL_FILES))
  uploader.params.put_bool('IsOffroad', True, block=True)
  assert uploader.next_file_to_upload(False)[2] == str(sentry/'qlog.zst')


def test_sentry_inflight_upload_defers_when_driving_starts_without_marking_failed(uploader, monkeypatch):
  directory = event_files(uploader)
  filename = directory/'qlog.zst'
  filename.write_bytes(b'x' * 32768)
  uploader.api = SimpleNamespace(get=lambda *a, **kw: SimpleNamespace(status_code=200,
    text='{"url":"https://example.invalid/upload","headers":{}}'), get_token=lambda: 'test')

  def put(url, *, data, **kwargs):
    assert data.read(16384) == b'x' * 16384
    uploader.params.put_bool('IsOffroad', False, block=True)
    data.read(16384)
    pytest.fail('Onroad transition must stop further Sentry bytes')

  monkeypatch.setattr(upload.requests, 'put', put)
  assert uploader.step(upload.NetworkType.wifi, False) is None
  assert getxattr(str(filename), upload.UPLOAD_ATTR_NAME) is None
  assert not uploader.sentry_failed and not uploader.sentry_denied_until
