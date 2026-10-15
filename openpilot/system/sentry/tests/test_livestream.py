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
  s.logger = Mock(spec=logging.Logger)
  s.video_tracks = [Mock()]
  s.bitrate_controller = s.outgoing_bridge = s.incoming_bridge = None
  s.incoming_bridge_services = []
  s.stream = SimpleNamespace(set_message_handler=Mock(), wait_for_connection=AsyncMock(),
                             has_messaging_channel=lambda: False, wait_for_disconnection=AsyncMock(), stop=AsyncMock())
  return s


@pytest.mark.parametrize('prewarm', [False, True])
def test_only_live_view_selects_once_and_keeps_choice_on_disconnect(session, prewarm):
  s = session
  s.enabled = not prewarm
  s.params.put_bool('SentryOffThisDrive', True, block=True)
  s.params.put('SentryParkingModeSource', 'wifi', block=True)

  async def view():
    if prewarm:
      assert parking_mode(s.params) == 'off'
      assert s.params.get('SentryArmResetAt') is None
      s.message_handler(json.dumps({'type': 'livestreamVideoEnable', 'data': {'enabled': True}}).encode())
    assert parking_mode(s.params) == HIGH_RISK
    assert s.params.get('SentryArmResetAt') > 0
    # A later local selection wins over repeated enable messages and pause/resume.
    selected_at = select_parking_mode(s.params, LOW_RISK)
    for enabled in (True, False, True):
      s.message_handler(json.dumps({'type': 'livestreamVideoEnable', 'data': {'enabled': enabled}}).encode())
    assert parking_mode(s.params) == LOW_RISK
    assert s.params.get('SentryArmResetAt') == selected_at

  s.stream.wait_for_disconnection.side_effect = view
  asyncio.run(s.run())
  s.logger.exception.assert_not_called()
  assert s._cleanup_done and not s._connected
  assert parking_mode(s.params) == LOW_RISK


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
  asyncio.run(session.run())
  assert parking_mode(session.params) == HIGH_RISK


def test_sentry_error_does_not_disconnect_stream(session, monkeypatch):
  from openpilot.system.sentry import config
  monkeypatch.setattr(config, 'apply_livestream_mode', Mock(side_effect=RuntimeError('settings unavailable')))
  asyncio.run(session.run())
  session.stream.wait_for_disconnection.assert_awaited_once()
  assert session._cleanup_done
