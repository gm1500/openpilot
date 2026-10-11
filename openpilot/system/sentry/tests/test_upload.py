import json
from types import SimpleNamespace

import pytest

from openpilot.common.params import Params
from openpilot.system.loggerd import uploader as upload
from openpilot.system.loggerd.xattr_cache import getxattr, setxattr
from openpilot.system.sentry import connect


@pytest.fixture(autouse=True)
def isolate_account(tmp_path, monkeypatch):
  monkeypatch.setattr(connect, 'credential_path', lambda: tmp_path/'persist'/'sentry_connect.json')
  monkeypatch.setattr(connect, 'STATUS_PATH', tmp_path/'upload_status.json')


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
  monkeypatch.setattr(upload.requests, 'post', lambda *args, **kwargs: response)

  def unexpected_put(*args, **kwargs):
    pytest.fail('A refused upload URL must not issue a file PUT')

  monkeypatch.setattr(upload.requests, 'put', unexpected_put)
  assert uploader.do_upload('route/ecamera.hevc', str(directory/'ecamera.hevc')) is response


@pytest.mark.parametrize('name', ['rlog.zst', 'fcamera.hevc', 'ecamera.hevc', 'dcamera.hevc'])
@pytest.mark.parametrize('account', [False, True])
def test_sentry_full_files_request_connect_batch_urls(uploader, monkeypatch, name, account):
  directory = event_files(uploader)
  key, fn = directory.name+'/'+name, str(directory/name)
  uploader.api = SimpleNamespace(get_token=lambda: 'test-device-token')
  monkeypatch.setattr(upload, 'account_token', lambda dongle: 'test-account-token' if account else None)
  signed_url = 'https://storage.example/authorized-upload'
  posts, puts = [], []

  def post(url, **kwargs):
    posts.append((url, kwargs))
    return SimpleNamespace(status_code=200, text=json.dumps([{'url': signed_url}]))

  def put(url, **kwargs):
    puts.append((url, kwargs['data'].read(), kwargs['headers']))
    # Merely obtaining a URL must not mark the file uploaded.
    assert getxattr(fn, upload.UPLOAD_ATTR_NAME) is None
    return SimpleNamespace(status_code=201, request=SimpleNamespace(headers={'Content-Length': '7'}))

  monkeypatch.setattr(upload.requests, 'post', post)
  monkeypatch.setattr(upload.requests, 'put', put)
  assert uploader.upload(name, key, fn, upload.NetworkType.wifi, False)
  assert len(posts) == 1
  url, kwargs = posts[0]
  assert url == connect.API_HOST+'/v1/test-device/upload_urls/'
  assert kwargs['json'] == {'paths': [key], 'expiry_days': 1}
  assert kwargs['headers']['Authorization'] == ('JWT test-account-token' if account else 'JWT test-device-token')
  assert kwargs['allow_redirects'] is False
  assert puts == [(signed_url, b'footage', {'x-ms-blob-type': 'BlockBlob'})]
  assert getxattr(fn, upload.UPLOAD_ATTR_NAME) == upload.UPLOAD_ATTR_VALUE
  assert connect.upload_status() == 'Last file uploaded'


def test_pending_videos_resume_with_verified_account_without_restart(uploader, monkeypatch):
  directory = event_files(uploader)
  for name in ('qlog.zst', 'qcamera.ts'):
    setxattr(str(directory/name), upload.UPLOAD_ATTR_NAME, upload.UPLOAD_ATTR_VALUE)
  uploader.api = SimpleNamespace(get_token=lambda: 'device-token')

  def post(url, **kwargs):
    owner = kwargs['headers']['Authorization'] == 'JWT owner-token'
    return SimpleNamespace(status_code=200 if owner else 412,
                           text=json.dumps([{'url': 'https://storage.example/upload'}]) if owner else '{}')

  monkeypatch.setattr(connect.requests, 'post', post)
  monkeypatch.setattr(connect.requests, 'get', lambda *args, **kwargs: SimpleNamespace(
    status_code=200, json=lambda: {'dongle_id': 'test-device', 'is_owner': True}))
  puts = []
  def put(url, **kwargs):
    assert 'Authorization' not in kwargs['headers']
    puts.append(kwargs['data'].read())
    return SimpleNamespace(status_code=201, request=SimpleNamespace(headers={'Content-Length': '7'}))
  monkeypatch.setattr(connect.requests, 'put', put)

  assert uploader.step(upload.NetworkType.wifi, False) is False
  assert connect.upload_status() == 'Connect rejected request (412)'
  assert not puts
  connect.authorize_account('owner-token', 'test-device', directory.name+'/fcamera.hevc')
  assert not puts  # Setup only validates authorization; it never transfers video.
  assert uploader.step(upload.NetworkType.wifi, True) is None
  for _ in range(4):
    assert uploader.step(upload.NetworkType.wifi, False) is True
  assert puts == [b'footage']*4
  assert uploader.next_file_to_upload(False) is None


@pytest.mark.parametrize('status', [401, 403, 412])
def test_account_refusal_keeps_file_pending_without_device_fallback(uploader, monkeypatch, status):
  directory = event_files(uploader)
  monkeypatch.setattr(upload, 'account_token', lambda dongle: 'owner-token')
  uploader.api = SimpleNamespace(get_token=lambda: pytest.fail('Do not fall back after account refusal'))
  monkeypatch.setattr(connect.requests, 'post', lambda *args, **kwargs: SimpleNamespace(status_code=status))
  monkeypatch.setattr(connect.requests, 'put', lambda *args, **kwargs: pytest.fail('Refused URL must never cause a PUT'))
  fn = str(directory/'ecamera.hevc')
  assert not uploader.upload('ecamera.hevc', directory.name+'/ecamera.hevc', fn, upload.NetworkType.wifi, False)
  assert getxattr(fn, upload.UPLOAD_ATTR_NAME) is None
  assert connect.upload_status() == f'Connect rejected request ({status})'


@pytest.mark.parametrize('payload', [[], {}, [{'url': ''}], [{'error': 'denied'}], [{'url': 'a'}, {'url': 'b'}]])
def test_malformed_sentry_upload_url_is_not_transferred_or_marked_uploaded(uploader, monkeypatch, payload):
  directory = event_files(uploader)
  fn = str(directory/'fcamera.hevc')
  uploader.api = SimpleNamespace(get_token=lambda: 'test-device-token')
  monkeypatch.setattr(upload.requests, 'post', lambda *args, **kwargs: SimpleNamespace(status_code=200, text=json.dumps(payload)))
  monkeypatch.setattr(upload.requests, 'put', lambda *args, **kwargs: pytest.fail('Malformed response must not cause a PUT'))
  assert not uploader.upload('fcamera.hevc', directory.name+'/fcamera.hevc', fn, upload.NetworkType.wifi, False)
  assert getxattr(fn, upload.UPLOAD_ATTR_NAME) is None


@pytest.mark.parametrize('sentry', [False, True])
def test_preview_keeps_ordinary_upload_endpoint(uploader, monkeypatch, sentry):
  directory = event_files(uploader)
  if not sentry:
    (directory/'.sentry').unlink()
  calls = []
  monkeypatch.setattr(upload, 'account_token', lambda dongle: pytest.fail('Preview must not use the account token'))

  def get(endpoint, **kwargs):
    calls.append((endpoint, kwargs))
    return SimpleNamespace(status_code=200, text=json.dumps({'url': 'https://storage.example/preview', 'headers': {}}))

  uploader.api = SimpleNamespace(get=get, get_token=lambda: 'test-token')
  monkeypatch.setattr(upload.requests, 'post', lambda *args, **kwargs: pytest.fail('Preview must not use batch API'))
  monkeypatch.setattr(upload.requests, 'put', lambda *args, **kwargs: SimpleNamespace(status_code=200))
  assert uploader.do_upload(directory.name+'/qcamera.ts', str(directory/'qcamera.ts')).status_code == 200
  assert calls[0][0] == 'v1.4/test-device/upload_url/'


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
