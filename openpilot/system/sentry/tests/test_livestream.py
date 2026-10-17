"""Check the actual WebRTC connection/enable lifecycle without a network peer."""
import asyncio
import json
import logging
from types import SimpleNamespace
from unittest.mock import Mock, AsyncMock

import pytest

from openpilot.common.params import Params
from openpilot.system.sentry.config import parking_mode, select_parking_mode, LOW_RISK, HIGH_RISK
from openpilot.system.webrtc.webrtcd import StreamSession
from openpilot.system.webrtc.helpers import livestream_video_active


@pytest.fixture
def session(tmp_path):
  # Skip SDP/encoder construction; exercise the real session run and cleanup.
  s = StreamSession.__new__(StreamSession)
  s.params = Params(str(tmp_path/'params'))
  for key in ('IsOffroad', 'IsLiveStreaming', 'SentryEnabled', 'SentryLivestreamOverride'):
    s.params.put_bool(key, True, block=True)
  s.identifier, s.enabled, s.is_body = 'test', True, False
  s._connected = s._sentry_mode_applied = s._cleanup_done = False
  s._cleanup_lock = asyncio.Lock()
  s.run_task = None
  s.logger = Mock(spec=logging.Logger)
  s.video_tracks = [Mock()]
  s.bitrate_controller = s.outgoing_bridge = s.incoming_bridge = None
  s.incoming_bridge_services = []
  s.stream = SimpleNamespace(set_message_handler=Mock(), wait_for_connection=AsyncMock(),
                             has_messaging_channel=lambda: False, wait_for_disconnection=AsyncMock(), stop=AsyncMock())
  return s


@pytest.mark.parametrize('prewarm', [False, True])
def test_each_live_view_applies_override_and_keeps_choice_on_disconnect(session, prewarm):
  s = session
  s.enabled = not prewarm
  s.params.put_bool('SentryOffThisDrive', True, block=True)
  s.params.put('SentryParkingModeSource', 'wifi', block=True)

  async def view():
    assert livestream_video_active(s.params) == (not prewarm)
    if prewarm:
      assert parking_mode(s.params) == 'off'
      assert s.params.get('SentryArmResetAt') is None
      s.message_handler(json.dumps({'type': 'livestreamVideoEnable', 'data': {'enabled': True}}).encode())
    assert parking_mode(s.params) == HIGH_RISK
    assert livestream_video_active(s.params)
    assert s.params.get('SentryArmResetAt') > 0
    # Duplicate enable messages preserve a local choice within this view.
    selected_at = select_parking_mode(s.params, LOW_RISK)
    for enabled in (True, True, False):
      s.message_handler(json.dumps({'type': 'livestreamVideoEnable', 'data': {'enabled': enabled}}).encode())
    assert parking_mode(s.params) == LOW_RISK
    assert s.params.get('SentryArmResetAt') == selected_at
    # Reopening live video through the same peer applies the override again.
    s.message_handler(json.dumps({'type': 'livestreamVideoEnable', 'data': {'enabled': True}}).encode())
    assert parking_mode(s.params) == HIGH_RISK
    assert s.params.get('SentryArmResetAt') > selected_at

  s.stream.wait_for_disconnection.side_effect = view
  asyncio.run(s.run())
  s.logger.exception.assert_not_called()
  assert s._cleanup_done and not s._connected
  assert not s.params.get_bool('LivestreamVideoActive')
  assert parking_mode(s.params) == HIGH_RISK


@pytest.mark.parametrize('blocker', ['prewarm', 'failed_connection', 'no_video', 'body'])
def test_non_viewing_sessions_never_select_a_mode(session, blocker):
  if blocker == 'prewarm':
    session.enabled = False
  elif blocker == 'failed_connection':
    session.stream.wait_for_connection.side_effect = TimeoutError
  elif blocker == 'no_video':
    session.video_tracks = []
  else:
    session.is_body = True
  asyncio.run(session.run())
  assert session.params.get('SentryArmResetAt') is None
  assert parking_mode(session.params) == LOW_RISK


def test_enable_before_connection_waits_for_connection(session):
  session.enabled = False
  session.message_handler(json.dumps({'type': 'livestreamVideoEnable', 'data': {'enabled': True}}).encode())
  assert session.params.get('SentryArmResetAt') is None
  assert not livestream_video_active(session.params)
  asyncio.run(session.run())
  assert parking_mode(session.params) == HIGH_RISK


def test_sentry_error_does_not_disconnect_stream(session, monkeypatch):
  from openpilot.system.sentry import config
  monkeypatch.setattr(config, 'apply_livestream_mode', Mock(side_effect=RuntimeError('settings unavailable')))
  asyncio.run(session.run())
  session.stream.wait_for_disconnection.assert_awaited_once()
  assert session._cleanup_done


def test_connect_release_resumes_sentry_while_peer_stays_connected(session, monkeypatch):
  from openpilot.system.sentry import config
  monkeypatch.setattr(config, 'read_runtime', lambda: {'capture': True})
  s = session

  async def view():
    assert livestream_video_active(s.params)
    assert not config.sentry_capture_requested(False, s.params, None)
    selected_at = s.params.get('SentryArmResetAt')
    # Connect.release() only disables video; its peer can stay up for minutes.
    s.message_handler(json.dumps({'type': 'livestreamVideoEnable', 'data': {'enabled': False}}))
    assert s._connected and s.params.get_bool('IsLiveStreaming')
    s.stream.stop.assert_not_awaited()
    s.video_tracks[0].enable.assert_called_with(False)
    assert not livestream_video_active(s.params)
    assert config.sentry_capture_requested(False, s.params, None)
    assert parking_mode(s.params) == HIGH_RISK
    # The same ready connection can open live video again.
    s.message_handler(json.dumps({'type': 'livestreamVideoEnable', 'data': {'enabled': True}}))
    assert livestream_video_active(s.params)
    assert not config.sentry_capture_requested(False, s.params, None)
    assert s.params.get('SentryArmResetAt') > selected_at

  s.stream.wait_for_disconnection.side_effect = view
  asyncio.run(s.run())
  s.logger.exception.assert_not_called()
  assert config.sentry_capture_requested(False, s.params, None)


def test_cancelled_view_and_late_messages_do_not_leave_camera_busy(session):
  async def cancel_view():
    viewed = asyncio.Event()

    async def wait_for_close():
      viewed.set()
      await asyncio.Event().wait()

    session.stream.wait_for_disconnection.side_effect = wait_for_close
    session.start()
    await asyncio.wait_for(viewed.wait(), 1.)
    assert livestream_video_active(session.params)
    await session.stop()
    assert not livestream_video_active(session.params)
    # Late callbacks from this peer must not change a newer viewer or local mode.
    selected_at = select_parking_mode(session.params, LOW_RISK)
    session.params.put_bool('LivestreamVideoActive', True, block=True)
    for enabled in (False, True):
      session.message_handler(json.dumps({'type': 'livestreamVideoEnable', 'data': {'enabled': enabled}}))
    await session.stop()  # Cleanup is idempotent, including the shared flag.
    assert livestream_video_active(session.params)
    assert session.params.get('SentryArmResetAt') == selected_at

  asyncio.run(cancel_view())
  session.logger.exception.assert_not_called()


@pytest.mark.parametrize('error', [TimeoutError, RuntimeError])
def test_answer_failure_releases_camera_and_schedules_teardown(session, monkeypatch, error):
  from openpilot.system.webrtc import webrtcd
  session.get_answer = AsyncMock(side_effect=error)
  monkeypatch.setattr(webrtcd, 'StreamSession', lambda body: session)
  teardown = Mock()
  monkeypatch.setattr(webrtcd, 'schedule_teardown', teardown)
  session.params.put_bool('LivestreamVideoActive', True, block=True)
  state = webrtcd.ServerState()
  body = json.dumps({'sdp': '', 'cameras': ['wideRoad'], 'enabled': True}).encode()
  with pytest.raises(error):
    asyncio.run(webrtcd.handle_get_stream(state, body, 'application/json'))
  assert not livestream_video_active(session.params)
  assert not state.streams
  teardown.assert_called_once_with(state)


def test_camera_ownership_handoff_and_idle_prewarm(session, monkeypatch):
  from openpilot.system.manager import process_config as processes
  from openpilot.system.sentry import config
  params, cp = session.params, SimpleNamespace(notCar=False)
  monkeypatch.setattr(config, 'read_runtime', lambda: {'capture': True})
  for name in ('encoderd', 'sentry_encoderd', 'stream_encoderd'):
    monkeypatch.setattr(processes.managed_processes[name], 'proc', None)
  def run(name):
    return processes.managed_processes[name].should_run(False, params, cp)

  # Idle Connect prewarm leaves Sentry running and doesn't start another encoder.
  assert run('webrtcd') and run('camerad') and run('sentry_encoderd')
  assert not run('stream_encoderd')
  monkeypatch.setattr(processes.managed_processes['sentry_encoderd'], 'proc', SimpleNamespace(is_alive=lambda: True))
  params.put_bool('LivestreamVideoActive', True, block=True)
  assert not run('sentry_encoderd') and not run('sentry_loggerd')
  assert not run('stream_encoderd')  # Wait for Sentry's encoder to finish stopping.
  monkeypatch.setattr(processes.managed_processes['sentry_encoderd'], 'proc', None)
  assert run('stream_encoderd') and run('camerad')

  # Video disabled: stop streaming, wait for its encoder to exit, then resume.
  monkeypatch.setattr(processes.managed_processes['stream_encoderd'], 'proc', SimpleNamespace(is_alive=lambda: True))
  params.put_bool('LivestreamVideoActive', False, block=True)
  assert run('webrtcd') and not run('stream_encoderd') and not run('sentry_encoderd')
  monkeypatch.setattr(processes.managed_processes['stream_encoderd'], 'proc', None)
  assert run('sentry_encoderd') and run('camerad')

  # With Sentry skipped, an idle connection alone no longer powers the cameras.
  params.put_bool('SentryOffThisDrive', True, block=True)
  assert run('webrtcd') and not run('camerad')
  assert not run('sentry_encoderd') and not run('stream_encoderd')
  params.put_bool('LivestreamVideoActive', True, block=True)
  params.put_bool('IsLiveStreaming', False, block=True)
  assert not livestream_video_active(params)  # Ignition/manager clearing wins.
  assert not run('stream_encoderd')
  assert processes.managed_processes['stream_encoderd'].should_run(True, params, SimpleNamespace(notCar=True))
