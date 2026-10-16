import io
import multiprocessing
import os
import struct
from types import SimpleNamespace
from unittest.mock import patch

import pytest
import requests
import zstandard

from openpilot.common import upload_priority as priority
from openpilot.common.utils import get_upload_stream


@pytest.fixture(autouse=True)
def priority_path(tmp_path, monkeypatch):
  monkeypatch.setattr(priority.Paths, 'shm_path', lambda: str(tmp_path))


def _hold_priority(directory, ready):
  with patch.object(priority.Paths, 'shm_path', return_value=directory), priority.map_download_priority():
    ready.send(True)
    ready.recv()


def test_process_exit_releases_pause_without_deleting_state(tmp_path):
  ctx = multiprocessing.get_context('spawn')
  parent, child = ctx.Pipe()
  process = ctx.Process(target=_hold_priority, args=(str(tmp_path), child))
  process.start()
  try:
    assert parent.poll(10), 'map worker did not acquire priority'
    assert parent.recv()
    assert priority.map_download_active()
    process.terminate()
    process.join(5)
    assert not process.is_alive()
    assert os.path.exists(priority._priority_path())
    assert not priority.map_download_active()
  finally:
    if process.is_alive():
      process.kill()
      process.join(5)
    parent.close()
    child.close()


def test_success_and_failure_release_pause():
  assert not priority.map_download_active()
  with priority.map_download_priority():
    assert priority.map_download_active()
  assert not priority.map_download_active()
  with pytest.raises(requests.ReadTimeout), priority.map_download_priority():
    assert priority.map_download_active()
    raise requests.ReadTimeout()
  assert not priority.map_download_active()


def test_hung_request_expiry(monkeypatch):
  clock = [100.]
  monkeypatch.setattr(priority.time, 'monotonic', lambda: clock[0])
  with priority.map_download_priority():
    clock[0] += priority.MAP_PRIORITY_TIMEOUT - .1
    assert priority.map_download_active()
    clock[0] += .2
    assert not priority.map_download_active()


def test_stale_or_invalid_state_cannot_pause_uploads():
  for raw in (b'', b'bad', struct.pack('d', float('inf')), struct.pack('d', float('nan'))):
    with open(priority._priority_path(), 'wb') as file:
      file.write(raw)
    assert not priority.map_download_active()


def test_coordination_error_does_not_block_maps_or_uploads(monkeypatch):
  monkeypatch.setattr(priority.Paths, 'shm_path', lambda: '/no/such/directory')
  with priority.map_download_priority():
    assert not priority.map_download_active()
  assert priority.MapAwareReader(io.BytesIO(b'payload')).read() == b'payload'


def test_active_upload_pauses_without_consuming_body_and_resumes(monkeypatch):
  payload = os.urandom(priority.UPLOAD_CHUNK_SIZE * 3 + 7)
  stream = io.BytesIO(payload)
  checks = []
  reader = priority.MapAwareReader(stream, lambda: checks.append(stream.tell()))
  received = reader.read(priority.UPLOAD_CHUNK_SIZE)
  lease = priority.map_download_priority()
  lease.__enter__()
  sleeps = []

  def sleep(seconds):
    sleeps.append(seconds)
    assert stream.tell() == priority.UPLOAD_CHUNK_SIZE
    if len(sleeps) == 3:
      lease.__exit__(None, None, None)

  monkeypatch.setattr(priority.time, 'sleep', sleep)
  try:
    while chunk := reader.read(1024 * 1024):
      assert len(chunk) <= priority.UPLOAD_CHUNK_SIZE
      received += chunk
  finally:
    lease.__exit__(None, None, None)
  assert len(sleeps) == 3
  assert checks[1:5] == [priority.UPLOAD_CHUNK_SIZE] * 4
  assert received == payload
  assert reader.tell() == len(payload)


def test_wait_keeps_cancellation_checks_responsive():
  def cancel():
    raise InterruptedError('cancelled')

  stream = io.BytesIO(b'payload')
  with priority.map_download_priority(), pytest.raises(InterruptedError):
    priority.MapAwareReader(stream, cancel).read()
  assert stream.tell() == 0


@pytest.mark.parametrize('compress', [False, True])
def test_http_content_length_and_body_are_preserved(tmp_path, compress):
  payload = os.urandom(priority.UPLOAD_CHUNK_SIZE * 3 + 7)
  file = tmp_path / 'upload'
  file.write_bytes(payload)
  stream, size = get_upload_stream(str(file), compress)
  with stream:
    request = requests.Request('PUT', 'https://example.invalid/upload', data=priority.MapAwareReader(stream)).prepare()
    assert int(request.headers['Content-Length']) == size
    assert 'Transfer-Encoding' not in request.headers
    received = b''.join(iter(lambda: request.body.read(65536), b''))
  assert len(received) == size
  assert (zstandard.ZstdDecompressor().decompress(received, max_output_size=len(payload)) if compress else received) == payload


@pytest.mark.parametrize('connect', [False, True])
def test_upload_entry_points_wait_before_requesting_urls_or_sending_body(tmp_path, monkeypatch, connect):
  from openpilot.system.athena import athenad
  from openpilot.system.loggerd import uploader

  file = tmp_path / 'qcamera.ts'
  payload = os.urandom(priority.UPLOAD_CHUNK_SIZE * 2 + 5)
  file.write_bytes(payload)
  lease = priority.map_download_priority()
  lease.__enter__()
  slept = []
  progress = []

  def sleep(seconds):
    slept.append(seconds)
    lease.__exit__(None, None, None)

  def put(url, *, data, headers, timeout):
    assert slept and not priority.map_download_active()
    assert isinstance(data, priority.MapAwareReader)
    request = requests.Request('PUT', url, data=data, headers=headers).prepare()
    assert int(request.headers['Content-Length']) == len(payload)
    assert b''.join(iter(lambda: data.read(65536), b'')) == payload
    return SimpleNamespace(status_code=200)

  def get(*args, **kwargs):
    assert slept and not priority.map_download_active()
    return SimpleNamespace(status_code=200, text='{"url":"https://example.invalid/upload","headers":{}}')

  monkeypatch.setattr(priority.time, 'sleep', sleep)
  monkeypatch.setattr(athenad.UPLOAD_SESS, 'put', put)
  monkeypatch.setattr(uploader.requests, 'put', put)
  monkeypatch.setattr(uploader, 'fake_upload', False)
  try:
    if connect:
      item = athenad.UploadItem(path=str(file), url='https://example.invalid/upload', headers={}, created_at=0, id='test')
      assert athenad._do_upload(item, lambda size, cur: progress.append((size, cur))).status_code == 200
      assert progress[-1] == (len(payload), len(payload))
    else:
      up = object.__new__(uploader.Uploader)
      up.dongle_id = 'test'
      up.api = SimpleNamespace(get=get, get_token=lambda: 'test')
      assert up.do_upload('qcamera.ts', str(file)).status_code == 200
  finally:
    lease.__exit__(None, None, None)
