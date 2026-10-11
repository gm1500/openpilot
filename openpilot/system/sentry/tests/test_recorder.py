"""Integration: real HEVC/H264 packets, native RAM recorder, native route readers.

Requires the built system/loggerd/sentry_loggerd binary and PyAV.
"""
import os
import signal
import subprocess
import time
from fractions import Fraction
from pathlib import Path

import numpy as np
import pytest
import zstandard

av = pytest.importorskip("av", reason="native video integration tests require PyAV")

from openpilot.cereal import messaging, log
from openpilot.cereal.services import SERVICE_LIST
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix

ROOT = Path(__file__).resolve().parents[3]
BINARY = ROOT/'system/loggerd/sentry_loggerd'
STREAMS = ('sentryNarrowEncodeData', 'sentryWideEncodeData', 'sentryCabinEncodeData', 'sentryQcamEncodeData')
FIELDS = ('narrowRoadEncodeData', 'wideRoadEncodeData', 'cabinEncodeData', 'qNarrowRoadEncodeData')
FILES = ('fcamera.hevc', 'ecamera.hevc', 'dcamera.hevc', 'qcamera.ts')


def sample_packet(codec_name, noisy=False):
  codec = av.CodecContext.create(codec_name, 'w')
  codec.width, codec.height = 526, 330
  codec.pix_fmt = 'yuv420p'
  codec.time_base, codec.framerate = Fraction(1, 20), Fraction(20, 1)
  codec.options = {'preset': 'ultrafast', 'tune': 'zerolatency'}
  if codec_name == 'libx265':
    codec.options['x265-params'] = 'pools=1:frame-threads=1:log-level=error'
  image = np.full((330, 526, 3), 75, dtype=np.uint8)
  image[90:190, 200:320] = (200, 20, 20)
  if noisy:
    image = np.random.default_rng(0).integers(0, 256, image.shape, dtype=np.uint8)
  packets = codec.encode(av.VideoFrame.from_ndarray(image, format='rgb24'))+codec.encode(None)
  assert len(packets) == 1 and packets[0].is_keyframe
  return bytes(packets[0])


def messages(path):
  with path.open('rb') as file, zstandard.ZstdDecompressor().stream_reader(file) as stream:
    data = stream.read()
  return list(log.Event.read_multiple_bytes(data))


@pytest.mark.skipif(not BINARY.exists(), reason='build sentry_loggerd first')
def test_recorder_waits_for_publishers_at_startup(tmp_path, monkeypatch):
  monkeypatch.setenv('LOG_ROOT', str(tmp_path/'routes'))
  with OpenpilotPrefix():
    messaging.reset_context()
    status = messaging.SubMaster(['sentryRecorderState'])
    pm = messaging.PubMaster(['sentryState', 'pandaStates', 'deviceState'])
    with (tmp_path/'startup.log').open('w') as logfile:
      process = subprocess.Popen([str(BINARY)], cwd=BINARY.parent, stdout=logfile, stderr=subprocess.STDOUT)
      try:
        # Neither the controller nor panda/hardwared has published yet. Then
        # introduce each service independently, as happens on a real cold boot.
        for service in (None, 'sentryState', 'pandaStates', 'deviceState'):
          if service is not None:
            msg = messaging.new_message(service, 1 if service == 'pandaStates' else None, valid=True)
            if service == 'sentryState':
              msg.sentryState.capture = True
            if service == 'pandaStates':
              msg.pandaStates[0].pandaType = log.PandaState.PandaType.tres
            pm.send(service, msg)
          deadline = time.monotonic()+3.
          while time.monotonic() < deadline:
            assert process.poll() is None, (tmp_path/'startup.log').read_text()
            status.update(100)
            if status.updated['sentryRecorderState']:
              assert not status['sentryRecorderState'].recording
              break
          else:
            pytest.fail('recorder did not report its waiting state')
        assert not list((tmp_path/'routes').iterdir())
      finally:
        if process.poll() is None:
          process.send_signal(signal.SIGINT)
          process.wait(timeout=8)
        assert process.returncode == 0, (tmp_path/'startup.log').read_text()


@pytest.mark.skipif(not BINARY.exists(), reason='build sentry_loggerd first')
@pytest.mark.parametrize('stop_reason', ['ignition', 'disabled', 'manager'])
def test_native_ram_event_and_stop(tmp_path, monkeypatch, stop_reason):
  monkeypatch.setenv('LOG_ROOT', str(tmp_path/'routes'))
  hevc, h264 = sample_packet('libx265'), sample_packet('libx264')
  with OpenpilotPrefix():
    messaging.reset_context()
    params = Params()
    params.put('SentryPreMinutes', 5, block=True)
    pm = messaging.PubMaster([*STREAMS, 'sentryState', 'pandaStates', 'deviceState'])
    status = messaging.SubMaster(['sentryRecorderState'])
    with (tmp_path/'recorder.log').open('w') as logfile:
      process = subprocess.Popen([str(BINARY)], cwd=BINARY.parent, stdout=logfile, stderr=subprocess.STDOUT)
      try:
        record_until = 0
        def control(trigger=False, ignition=False, capture=True):
          nonlocal record_until
          if trigger:
            record_until = time.monotonic_ns()+10_000_000_000
          msg = messaging.new_message('sentryState', valid=True)
          msg.sentryState.capture = capture
          msg.sentryState.motion = True  # Eye-only movement must never save or bookmark.
          msg.sentryState.recordUntilMonoTime = record_until
          msg.sentryState.locationValid = True
          msg.sentryState.locationSource = 'slcParked'
          msg.sentryState.locationSavedAt = time.time()-600  # noqa: TID251
          msg.sentryState.latitude, msg.sentryState.longitude = 53.5, -113.5
          msg.sentryState.locationAccuracy = 6.
          pm.send('sentryState', msg)
          msg = messaging.new_message('pandaStates', 1, valid=True)
          msg.pandaStates[0].ignitionLine = ignition
          msg.pandaStates[0].pandaType = log.PandaState.PandaType.tres
          pm.send('pandaStates', msg)
          msg = messaging.new_message('deviceState', valid=True)
          msg.deviceState.started = False
          pm.send('deviceState', msg)

        def await_state(predicate, trigger=False, ignition=False, capture=True):
          deadline = time.monotonic()+5
          while time.monotonic() < deadline:
            assert process.poll() is None, (tmp_path/'recorder.log').read_text()
            control(trigger, ignition, capture)
            status.update(50)
            if predicate(status['sentryRecorderState']):
              return
          pytest.fail(f'recorder timeout: {status["sentryRecorderState"]}\n{(tmp_path/"recorder.log").read_text()}')

        await_state(lambda _: status.updated['sentryRecorderState'])
        base = time.monotonic_ns()-200_000_000_000
        wall_base = time.time_ns()-(time.monotonic_ns()-base)
        # Four minutes of source segments from an already-running encoder. The
        # saved route must start at zero, with one clock origin for every log.
        for segment in range(4):
          control()
          for frame in range(20):
            for c in range(4):
              msg = messaging.new_message(FIELDS[c], valid=True)
              msg.logMonoTime = base+segment*60_000_000_000+frame*50_000_000
              data = getattr(msg, FIELDS[c])
              data.width, data.height = 526, 330
              data.data = h264 if c == 3 else hevc
              idx = data.idx
              idx.type = log.EncodeIndex.Type.qcameraH264 if c == 3 else log.EncodeIndex.Type.fullHEVC
              idx.frameId = segment*1200+frame
              idx.segmentId, idx.segmentNum, idx.flags = frame, segment+7, 8
              idx.timestampSof = idx.timestampEof = msg.logMonoTime
              pm.send(STREAMS[c], msg)
            time.sleep(.003)
        await_state(lambda s: s.bufferBytes > 10000)
        assert not list((tmp_path/'routes').iterdir()), 'idle capture wrote a route to disk'
        assert params.get('RouteCount') is None
        trigger_time = time.monotonic_ns()
        await_state(lambda s: s.recording, trigger=True)
        if stop_reason == 'disabled':
          # Keep visual motion active through the tail without confirming new
          # motion. It must not create extra markers, even after the quiet gap.
          deadline = time.monotonic()+5.5
          while time.monotonic() < deadline:
            control()
            status.update(50)
          await_state(lambda s: s.recording and status.updated['sentryRecorderState'], trigger=True)
        if stop_reason == 'manager':
          # The manager can observe the master switch before the controller does.
          # Its normal stop signal must finalize an event that is still recording.
          process.send_signal(signal.SIGINT)
          process.wait(timeout=8)
        else:
          # Both ignition and capture=false override an unexpired event request.
          await_state(lambda s: not s.recording and s.bufferBytes == 0, trigger=True,
                      ignition=stop_reason == 'ignition', capture=stop_reason != 'disabled')
        dirs = sorted((tmp_path/'routes').iterdir())
        assert len(dirs) == 4
        route_init = messages(dirs[0]/'qlog.zst')[0]
        assert route_init.logMonoTime == base
        assert abs(route_init.initData.wallTimeNanos-wall_base) < 100_000_000
        for number, directory in enumerate(dirs):
          assert directory.name.endswith(f'--{number}')
          assert (directory/'.sentry').exists()
          assert not list(directory.glob('*.lock'))
          assert 'user.preserve' not in os.listxattr(directory)
          for filename in FILES:
            with av.open(str(directory/filename)) as video:
              frames = list(video.decode(video=0))
            assert len(frames) == 20
            assert (frames[0].width, frames[0].height) == (526, 330)
          events = messages(directory/'qlog.zst')
          for filename in ('qlog.zst', 'rlog.zst'):
            log_events = messages(directory/filename)
            assert log_events[0].which() == 'initData'
            assert log_events[0].as_builder().to_bytes() == route_init.as_builder().to_bytes()
            start = next(e for e in log_events if e.which() == 'sentinel')
            assert start.logMonoTime == base+number*60_000_000_000
            assert start.sentinel.type == ('startOfRoute' if number == 0 else 'startOfSegment')
          indexes = [getattr(e, e.which()) for e in events if e.which().endswith('EncodeIdx')]
          assert len(indexes) == 80
          assert all(i.segmentNum == number for i in indexes)
          assert events[-1].sentinel.type == ('endOfRoute' if number == 3 else 'endOfSegment')
        all_events = [e for directory in dirs for e in messages(directory/'qlog.zst')]
        assert any(e.which() == 'sentryState' and e.sentryState.locationSource == 'slcParked' for e in all_events)
        assert any(e.which() == 'gpsLocation' and e.gpsLocation.source == 'fusion' for e in all_events)
        for filename in ('qlog.zst', 'rlog.zst'):
          bookmarks = [e for directory in dirs for e in messages(directory/filename) if e.which() == 'userBookmark']
          assert len(bookmarks) == (2 if stop_reason == 'disabled' else 1)
          assert trigger_time <= bookmarks[0].logMonoTime <= time.monotonic_ns()
          if len(bookmarks) == 2:
            assert bookmarks[1].logMonoTime-bookmarks[0].logMonoTime >= 5_000_000_000
      finally:
        if process.poll() is None:
          process.send_signal(signal.SIGINT)
          process.wait(timeout=8)
        assert process.returncode == 0, (tmp_path/'recorder.log').read_text()


@pytest.mark.skipif(not BINARY.exists(), reason='build sentry_loggerd first')
@pytest.mark.parametrize('publisher_first', [True, False])
def test_video_queue_wrap_and_repeated_events(tmp_path, monkeypatch, publisher_first):
  monkeypatch.setenv('LOG_ROOT', str(tmp_path/'routes'))
  # Larger real keyframes cross both the old 1 MiB mapping and the registered
  # 10 MiB queue multiple times without a many-minute real-time camera test.
  hevc, h264 = sample_packet('libx265', noisy=True), sample_packet('libx264', noisy=True)
  frame_count = 2*max(SERVICE_LIST[s].queue_size for s in STREAMS)//min(len(hevc), len(h264))+1
  with OpenpilotPrefix() as prefix:
    messaging.reset_context()
    pm = messaging.PubMaster(['sentryState', 'pandaStates', 'deviceState'])
    video_pm = messaging.PubMaster(STREAMS) if publisher_first else None
    status = messaging.SubMaster(['sentryRecorderState'])
    with (tmp_path/'recorder.log').open('w') as logfile:
      process = subprocess.Popen([str(BINARY)], cwd=BINARY.parent, stdout=logfile, stderr=subprocess.STDOUT)
      try:
        def await_state(predicate, trigger=False, capture=True):
          deadline = time.monotonic()+8
          while time.monotonic() < deadline:
            assert process.poll() is None, (tmp_path/'recorder.log').read_text()
            msg = messaging.new_message('sentryState', valid=True)
            msg.sentryState.capture = capture
            msg.sentryState.recordUntilMonoTime = time.monotonic_ns()+10_000_000_000 if trigger else 0
            pm.send('sentryState', msg)
            msg = messaging.new_message('pandaStates', 1, valid=True)
            msg.pandaStates[0].pandaType = log.PandaState.PandaType.tres
            pm.send('pandaStates', msg)
            pm.send('deviceState', messaging.new_message('deviceState', valid=True))
            status.update(50)
            if status.updated['sentryRecorderState'] and predicate(status['sentryRecorderState']):
              return
          pytest.fail(f'recorder timeout: {status["sentryRecorderState"]}\n{(tmp_path/"recorder.log").read_text()}')

        await_state(lambda _: True)
        for service in STREAMS:
          # Fail safely before publishing: a smaller file would SIGBUS the
          # Python publisher as it writes beyond the recorder's truncated map.
          assert (Path(prefix.msgq_path)/service).stat().st_size >= SERVICE_LIST[service].queue_size
        if video_pm is None:
          video_pm = messaging.PubMaster(STREAMS)
        for service in STREAMS:
          assert video_pm.wait_for_readers_to_update(service, 2)

        base = time.monotonic_ns()-180_000_000_000
        sent_bytes = 0
        for first, last in ((0, frame_count//2), (frame_count//2, frame_count)):
          for frame in range(first, last):
            for camera, service in enumerate(STREAMS):
              msg = messaging.new_message(FIELDS[camera], valid=True)
              msg.logMonoTime = base+frame*50_000_000
              data = getattr(msg, FIELDS[camera])
              data.width, data.height = 526, 330
              data.data = h264 if camera == 3 else hevc
              idx = data.idx
              idx.type = log.EncodeIndex.Type.qcameraH264 if camera == 3 else log.EncodeIndex.Type.fullHEVC
              idx.frameId, idx.segmentId, idx.segmentNum, idx.flags = frame, frame, 0, 8
              idx.timestampSof = idx.timestampEof = msg.logMonoTime
              video_pm.send(service, msg)
              sent_bytes += len(data.data)
            if frame % 20 == 0:
              await_state(lambda _: True)
            time.sleep(.003)
          await_state(lambda s, expected=sent_bytes: s.bufferBytes >= expected)
          # Both requests exercise the same recordUntil path as Save event and
          # motion. The second must work after continued capture/queue wraps.
          await_state(lambda s: s.recording, trigger=True)
          await_state(lambda s: not s.recording)

        await_state(lambda s: s.bufferBytes == 0, capture=False)
        directories = sorted((tmp_path/'routes').iterdir())
        assert len(directories) == 2
        for directory, expected in zip(directories, (frame_count//2, frame_count), strict=True):
          assert not list(directory.glob('*.lock'))
          for filename in FILES:
            with av.open(str(directory/filename)) as video:
              assert sum(1 for _ in video.decode(video=0)) == expected
      finally:
        if process.poll() is None:
          process.send_signal(signal.SIGINT)
          process.wait(timeout=8)
        assert process.returncode == 0, (tmp_path/'recorder.log').read_text()


@pytest.mark.skipif(not (ROOT/'system/loggerd/encoderd').exists(), reason='build encoderd first')
def test_encoder_private_watermark_buffers(monkeypatch, tmp_path):
  import io
  import socket as unix_socket
  try:
    probe = unix_socket.socket(unix_socket.AF_UNIX, unix_socket.SOCK_SEQPACKET)
    probe.close()
  except PermissionError:
    pytest.skip('environment does not permit VisionIPC Unix sockets')
  from msgq.visionipc import VisionIpcServer, VisionIpcClient
  from openpilot.system.sentry.config import DEFAULTS
  monkeypatch.setenv('LOG_ROOT', str(tmp_path/'routes'))
  with OpenpilotPrefix():
    messaging.reset_context()
    params = Params()
    for key, value in DEFAULTS.items():
      params.put(key, value, block=True)
    server = VisionIpcServer('camerad')
    width, height = 640, 360
    for camera in (0, 1, 2):
      server.create_buffers(camera, 8, width, height)
    server.start_listener()
    observer = VisionIpcClient('camerad', 0, True)
    deadline = time.monotonic()+2
    while not observer.connect(False) and time.monotonic() < deadline:
      time.sleep(.01)
    assert observer.is_connected()
    socket = messaging.sub_sock('sentryQcamEncodeData', timeout=100)
    image = np.full(width*height*3//2, 128, dtype=np.uint8)
    packets = []
    with (tmp_path/'encoder.log').open('w') as logfile:
      process = subprocess.Popen([str(ROOT/'system/loggerd/encoderd'), '--sentry'],
                                 cwd=ROOT/'system/loggerd', stdout=logfile, stderr=subprocess.STDOUT)
      try:
        for frame in range(65):
          assert process.poll() is None, (tmp_path/'encoder.log').read_text()
          now = time.monotonic_ns()
          for camera in (0, 1, 2):
            server.send(camera, image, frame, now, now)
          time.sleep(.05)
          msg = messaging.recv_one_or_none(socket)
          if msg:
            packets.append(bytes(msg.qNarrowRoadEncodeData.data))
        source = observer.recv(100)
        assert source is not None and np.all(source.data == 128), 'watermark modified a shared camera buffer'
      finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=8)
      assert process.returncode == 0, (tmp_path/'encoder.log').read_text()
      del observer, server
    assert len(packets) > 20
    with av.open(io.BytesIO(b''.join(packets)), format='h264') as video:
      frames = list(video.decode(video=0))
    assert len(frames) > 20
    image = frames[-1].to_ndarray(format='gray')
    assert image.shape == (330, 526)
    assert image[290:308].max() > 200 and image[290:308].min() < 30
    assert image[:250].std() < 2
