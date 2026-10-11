import json
from types import SimpleNamespace

import pytest

from openpilot.system.sentry import connect

KEY = '00000001--0123456789--0/fcamera.hevc'


@pytest.fixture(autouse=True)
def isolated_account(tmp_path, monkeypatch):
  monkeypatch.setattr(connect, 'credential_path', lambda: tmp_path/'private'/'sentry_connect.json')
  monkeypatch.setattr(connect, 'STATUS_PATH', tmp_path/'status.json')
  monkeypatch.setattr(connect.requests, 'put', lambda *args, **kwargs: pytest.fail('Setup must not upload footage'))


def mock_owner(monkeypatch, *, owner=True, dongle='test-device', status=200, upload_status=200, payload=None):
  monkeypatch.setattr(connect.requests, 'get', lambda *args, **kwargs: SimpleNamespace(
    status_code=status, json=lambda: {'dongle_id': dongle, 'is_owner': owner}))
  monkeypatch.setattr(connect.requests, 'post', lambda *args, **kwargs: SimpleNamespace(
    status_code=upload_status, text=json.dumps(payload if payload is not None else [{'url': 'https://storage.example/upload'}])))


def test_account_is_verified_then_saved_privately_for_only_this_device_and_host(monkeypatch):
  calls = []
  def get(url, **kwargs):
    calls.append((url, kwargs))
    return SimpleNamespace(status_code=200, json=lambda: {'dongle_id': 'test-device', 'is_owner': True})
  mock_owner(monkeypatch)
  monkeypatch.setattr(connect.requests, 'get', get)
  connect.authorize_account('private-account-token', 'test-device', KEY)
  assert calls[0][0] == connect.API_HOST+'/v1.1/devices/test-device/'
  assert calls[0][1]['headers']['Authorization'] == 'JWT private-account-token'
  assert calls[0][1]['allow_redirects'] is False
  assert connect.credential_path().stat().st_mode & 0o777 == 0o600
  assert connect.account_token('test-device') == 'private-account-token'
  assert connect.account_token('other-device') is None
  assert 'private-account-token' not in connect.STATUS_PATH.read_text()
  monkeypatch.setattr(connect, 'API_HOST', 'https://api.comma.ai' if connect.API_HOST != 'https://api.comma.ai' else 'https://api.commadotai.com')
  assert connect.account_token('test-device') is None


@pytest.mark.parametrize('kwargs', [
  {'owner': False}, {'dongle': 'other-device'}, {'status': 401}, {'status': 403},
  {'upload_status': 401}, {'upload_status': 403}, {'upload_status': 412},
  {'upload_status': 429}, {'upload_status': 500}, {'payload': []},
  {'payload': [{'url': 'http://storage.example/upload'}]}, {'payload': [{'url': 'https://storage.example', 'headers': None}]},
])
def test_failed_verification_does_not_persist_or_replace_credentials(monkeypatch, kwargs):
  mock_owner(monkeypatch)
  connect.authorize_account('previous-token', 'test-device', KEY)
  before = connect.credential_path().read_bytes()
  mock_owner(monkeypatch, **kwargs)
  with pytest.raises((connect.SetupError, ValueError)):
    connect.authorize_account('new-token', 'test-device', KEY)
  assert connect.credential_path().read_bytes() == before
  connect.credential_path().unlink()
  with pytest.raises((connect.SetupError, ValueError)):
    connect.authorize_account('new-token', 'test-device', KEY)
  assert not connect.credential_path().exists()


def test_account_token_never_sent_to_custom_api_or_redirect(monkeypatch):
  monkeypatch.setattr(connect, 'API_HOST', 'https://other.example')
  monkeypatch.setattr(connect.requests, 'get', lambda *args, **kwargs: pytest.fail('Do not send account token to other hosts'))
  with pytest.raises(connect.SetupError):
    connect.authorize_account('private-token', 'test-device', KEY)
  assert not connect.credential_path().exists()


def test_bad_permissions_and_invalid_credentials_are_not_loaded(monkeypatch):
  assert connect.account_token('test-device') is None
  mock_owner(monkeypatch)
  connect.authorize_account('private-token', 'test-device', KEY)
  path = connect.credential_path()
  path.chmod(0o644)
  assert connect.account_token('test-device') is None
  path.chmod(0o600)
  for invalid in ('{', 'null', '[]', '{}'):
    path.write_text(invalid)
    assert connect.account_token('test-device') is None


def test_setup_uses_existing_completed_video_and_can_remove_authorization(monkeypatch, tmp_path):
  root = tmp_path/'routes'
  for index in range(3):
    directory = root/f'00000001--0123456789--{index}'
    directory.mkdir(parents=True)
    (directory/'.sentry').touch()
    (directory/'dcamera.hevc').write_bytes(b'footage' if index != 2 else b'')
  (root/'00000001--0123456789--1'/'sentry.lock').touch()
  assert connect.newest_saved_video(root) == '00000001--0123456789--0/dcamera.hevc'
  mock_owner(monkeypatch)
  connect.authorize_account('private-token', 'test-device', KEY)
  monkeypatch.setattr(connect.sys, 'argv', ['connect', '--clear'])
  assert connect.main() == 0
  assert not connect.credential_path().exists()


def test_upload_status_expires_and_cannot_break_uploads(monkeypatch):
  monkeypatch.setattr(connect.time, 'monotonic', lambda: 100.)
  connect.set_upload_status('Connect rejected request (412)')
  assert connect.upload_status() == 'Connect rejected request (412)'
  monkeypatch.setattr(connect.time, 'monotonic', lambda: 500.)
  assert connect.upload_status() == 'Automatic'
  monkeypatch.setattr(connect, 'STATUS_PATH', connect.STATUS_PATH/'missing'/'status.json')
  connect.set_upload_status('Uploading event files')
  assert connect.upload_status() == 'Automatic'
