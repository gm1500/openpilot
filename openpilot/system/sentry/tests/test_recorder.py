"""Integration: real HEVC/H264 packets, native RAM recorder, native route readers.

Requires the built system/loggerd/sentry_loggerd binary and PyAV.
"""
import os
import re
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
STREAMS = ('sentryNarrowEncodeData', 'sentryWideEncodeData', 'sentryCabinEncodeData',
           'sentryQcamEncodeData', 'sentryWideQcamEncodeData', 'sentryCabinQcamEncodeData')
FIELDS = ('narrowRoadEncodeData', 'wideRoadEncodeData', 'cabinEncodeData', *('qNarrowRoadEncodeData',)*3)
FILES = ('fcamera.hevc', 'ecamera.hevc', 'dcamera.hevc', 'qcamera.ts')
CAMERA_INDEX = {0: 0, 2: 1, 1: 2}  # VisionStreamType -> narrow, wide, cabin output order.
COLORS = (45, 110, 180)


def sample_packet(codec_name, noisy=False, color=75):
  codec = av.CodecContext.create(codec_name, 'w')
  codec.width, codec.height = 526, 330
  codec.pix_fmt = 'yuv420p'
  codec.time_base, codec.framerate = Fraction(1, 20), Fraction(20, 1)
  codec.options = {'preset': 'ultrafast', 'tune': 'zerolatency'}
  if codec_name == 'libx265':
    codec.options['x265-params'] = 'pools=1:frame-threads=1:log-level=error'
  image = np.full((330, 526, 3), color, dtype=np.uint8)
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
@pytest.mark.parametrize('stop_reason,trigger_camera,pre_seconds', [('ignition', 0, 15), ('disabled', 1, 30), ('manager', 2, 60)])
def test_native_ram_event_and_stop(tmp_path, monkeypatch, stop_reason, trigger_camera, pre_seconds):
  monkeypatch.setenv('LOG_ROOT', str(tmp_path/'routes'))
  hevc = [sample_packet('libx265', color=color) for color in COLORS]
  h264 = [sample_packet('libx264', color=color) for color in COLORS]
  with OpenpilotPrefix():
    messaging.reset_context()
    params = Params()
    params.put('SentryPreSeconds', pre_seconds, block=True)
    pm = messaging.PubMaster([*STREAMS, 'sentryState', 'pandaStates', 'deviceState'])
    status = messaging.SubMaster(['sentryRecorderState'])
    live_selfdrive = messaging.sub_sock('selfdriveState')
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
          msg.sentryState.motionCamera = (trigger_camera+1)%3
          msg.sentryState.triggerCamera = trigger_camera
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
        base = time.monotonic_ns()-75_000_000_000
        wall_base = time.time_ns()-(time.monotonic_ns()-base)
        # 75 seconds across two native minutes. At each duration, trim the first
        # minute to actual seconds without shifting the next minute's clock.
        for segment, count in ((0, 60), (1, 15)):
          control()
          for frame in range(count):
            for c in range(len(STREAMS)):
              msg = messaging.new_message(FIELDS[c], valid=True)
              msg.logMonoTime = base+(segment*60+frame)*1_000_000_000
              data = getattr(msg, FIELDS[c])
              data.width, data.height = 526, 330
              data.data = h264[c-3] if c >= 3 else hevc[c]
              idx = data.idx
              idx.type = log.EncodeIndex.Type.qcameraH264 if c >= 3 else log.EncodeIndex.Type.fullHEVC
              idx.frameId = segment*1200+frame*20+10000*(c%3)
              idx.segmentId, idx.segmentNum, idx.flags = frame*20, segment+7, 8
              idx.timestampSof = idx.timestampEof = msg.logMonoTime
              pm.send(STREAMS[c], msg)
            time.sleep(.003)
        await_state(lambda s: s.bufferBytes > 10000)
        assert 0 < status['sentryRecorderState'].bufferSeconds <= pre_seconds
        assert not list((tmp_path/'routes').iterdir()), 'idle capture wrote a route to disk'
        assert params.get('RouteCount') is None
        trigger_time = time.monotonic_ns()
        await_state(lambda s: s.recording, trigger=True)
        event_route = status['sentryRecorderState'].route
        if stop_reason == 'disabled':
          # Keep visual motion active through the tail without confirming new
          # motion. It must not create extra markers, even after the quiet gap.
          deadline = time.monotonic()+5.5
          while time.monotonic() < deadline:
            control()
            status.update(50)
          first_deadline = record_until
          await_state(lambda s: s.recording and status.updated['sentryRecorderState'], trigger=True)
          assert record_until > first_deadline
          # The second confirmation must retain the route beyond the first
          # event's original tail, not close it and replay pre-roll as a new one.
          while time.monotonic_ns() <= first_deadline+200_000_000:
            control()
            status.update(50)
            assert status['sentryRecorderState'].recording
            assert status['sentryRecorderState'].route == event_route
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
        assert messaging.recv_one_or_none(live_selfdrive) is None, 'timeline annotation leaked to live alerts'
        assert len(dirs) == (1 if pre_seconds == 15 else 2)
        assert {directory.name.rsplit('--', 1)[0] for directory in dirs} == {event_route}
        route_init = messages(dirs[0]/'qlog.zst')[0]
        first_source = 1 if pre_seconds == 15 else 0
        assert route_init.logMonoTime == base+first_source*60_000_000_000
        assert abs(route_init.initData.wallTimeNanos-(wall_base+first_source*60_000_000_000)) < 100_000_000
        for number, directory in enumerate(dirs):
          assert directory.name.endswith(f'--{number}')
          assert (directory/'.sentry').exists()
          assert not list(directory.glob('*.lock'))
          assert 'user.preserve' not in os.listxattr(directory)
          events = messages(directory/'qlog.zst')
          preview_indexes = [e.qNarrowRoadEncodeIdx for e in events if e.which() == 'qNarrowRoadEncodeIdx']
          timestamps = [i.timestampEof for i in preview_indexes]
          assert timestamps and all(t >= trigger_time-pre_seconds*1_000_000_000 for t in timestamps)
          if number == 0:
            assert timestamps[0] <= trigger_time-(pre_seconds-2)*1_000_000_000
          source_segment = first_source+number
          first_frame = (timestamps[0]-base)//1_000_000_000-source_segment*60
          expected_frames = list(range(first_frame, 60 if source_segment == 0 else 15))
          assert timestamps == [base+(source_segment*60+f)*1_000_000_000 for f in expected_frames]
          for c, filename in enumerate(FILES):
            with av.open(str(directory/filename)) as video:
              frames = list(video.decode(video=0))
            assert len(frames) == len(expected_frames)
            assert (frames[0].width, frames[0].height) == (526, 330)
            source = c if c < 3 else CAMERA_INDEX[trigger_camera]
            assert abs(float(frames[0].to_ndarray(format='rgb24')[10, 10].mean())-COLORS[source]) < 10
            if c < 3:
              assert (directory/filename).read_bytes() == hevc[c]*len(expected_frames)
          for filename in ('qlog.zst', 'rlog.zst'):
            log_events = messages(directory/filename)
            assert log_events[0].which() == 'initData'
            assert log_events[0].as_builder().to_bytes() == route_init.as_builder().to_bytes()
            start = next(e for e in log_events if e.which() == 'sentinel')
            assert start.logMonoTime == timestamps[0]
            assert start.sentinel.type == ('startOfRoute' if number == 0 else 'startOfSegment')
            # Orange Sentry annotations span the saved pre-roll and tail in both
            # logs, without ever claiming driving engagement or requesting sound.
            timeline = [e for e in log_events if e.which() == 'selfdriveState']
            assert len(timeline) == 2
            # Connect's offset zero is initData, which can precede the first
            # retained frame in a trimmed minute. Cover that leading interval.
            assert timeline[0].logMonoTime == (route_init.logMonoTime if number == 0 else start.logMonoTime)
            if number == 0:
              assert timeline[0].logMonoTime < timestamps[0], 'fixture must exercise a partial first minute'
            assert timeline[-1].logMonoTime == log_events[-1].logMonoTime
            for event in timeline:
              state = event.selfdriveState
              assert event.valid
              assert state.state == 'disabled' and not state.enabled and not state.active and not state.engageable
              assert state.alertStatus == 'userPrompt' and state.alertSize == 'small'
              assert state.alertType == 'sentryRecording/permanent'
              assert state.alertText1 == 'Sentry recording' and state.alertText2 == 'Parked event'
              assert state.alertSound == 'none' and state.alertHudVisual == 'none'
          indexes = [getattr(e, e.which()) for e in events if e.which().endswith('EncodeIdx')]
          assert len(indexes) == 4*len(expected_frames)
          assert all(i.segmentNum == number for i in indexes)
          assert [i.frameId for i in preview_indexes] == [source_segment*1200+frame*20+10000*CAMERA_INDEX[trigger_camera]
                                                       for frame in expected_frames]
          assert events[-1].sentinel.type == ('endOfRoute' if number == len(dirs)-1 else 'endOfSegment')
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
  # 10 MiB queues without a many-minute real-time camera test. With six streams,
  # one wrap stays below the recorder's 128 MiB cap while retaining every frame.
  hevc, h264 = sample_packet('libx265', noisy=True), sample_packet('libx264', noisy=True)
  frame_count = max(SERVICE_LIST[s].queue_size for s in STREAMS)//min(len(hevc), len(h264))+1
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

        base = time.monotonic_ns()-30_000_000_000
        sent_bytes = 0
        for first, last in ((0, frame_count//2), (frame_count//2, frame_count)):
          for frame in range(first, last):
            for camera, service in enumerate(STREAMS):
              msg = messaging.new_message(FIELDS[camera], valid=True)
              msg.logMonoTime = base+frame*50_000_000
              data = getattr(msg, FIELDS[camera])
              data.width, data.height = 526, 330
              data.data = h264 if camera >= 3 else hevc
              idx = data.idx
              idx.type = log.EncodeIndex.Type.qcameraH264 if camera >= 3 else log.EncodeIndex.Type.fullHEVC
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


@pytest.mark.skipif(not BINARY.exists(), reason='build sentry_loggerd first')
@pytest.mark.parametrize('requested_keyframes', [False, True])
def test_preview_switches_within_a_segment_on_keyframes_and_preserves_overlapping_history(tmp_path, monkeypatch, requested_keyframes):
  monkeypatch.setenv('LOG_ROOT', str(tmp_path/'routes'))
  count = 120
  hevc = [sample_packet('libx265', color=color) for color in COLORS]
  previews = []
  for color in COLORS:
    codec = av.CodecContext.create('libx264', 'w')
    codec.width, codec.height, codec.pix_fmt = 526, 330, 'yuv420p'
    codec.time_base, codec.framerate = Fraction(1, 20), Fraction(20, 1)
    codec.options = {'preset': 'ultrafast', 'tune': 'zerolatency', 'forced-idr': '1',
                     'x264-params': 'keyint=20:min-keyint=20:scenecut=0'}
    packets = []
    for frame in range(count):
      image = av.VideoFrame.from_ndarray(np.full((330, 526, 3), color, dtype=np.uint8), format='rgb24')
      image.pts = frame
      if requested_keyframes and frame in (24, 64, 91):
        image.pict_type = av.video.frame.PictureType.I
      packets.extend(codec.encode(image))
    packets.extend(codec.encode(None))
    assert len(packets) == count
    source = []
    for packet in packets:
      # Match hardware output: SPS/PPS are separate from the keyframe payload.
      data = bytes(packet)
      starts = [m.start() for m in re.finditer(rb'\x00\x00(?:\x00)?\x01', data)]+[len(data)]
      header, payload = bytearray(), bytearray()
      for start, end in zip(starts, starts[1:], strict=False):
        nal = data[start:end]
        kind = nal.lstrip(b'\x00')[1] & 31
        (header if kind in (7, 8) else payload).extend(nal)
      source.append((bytes(header), bytes(payload), packet.is_keyframe))
    keys = [i for i, (_, _, key) in enumerate(source) if key]
    if requested_keyframes:
      assert all(frame in keys for frame in (24, 64, 91))
    else:
      assert keys == list(range(0, count, 20))
    previews.append(source)

  with OpenpilotPrefix():
    messaging.reset_context()
    pm = messaging.PubMaster([*STREAMS, 'sentryState', 'pandaStates', 'deviceState'])
    status = messaging.SubMaster(['sentryRecorderState'])
    base = time.monotonic_ns()-10_000_000_000
    recording, camera, control_frame = False, 2, 0
    with (tmp_path/'recorder.log').open('w') as logfile:
      process = subprocess.Popen([str(BINARY)], cwd=BINARY.parent, stdout=logfile, stderr=subprocess.STDOUT)
      try:
        def control():
          msg = messaging.new_message('sentryState', valid=True)
          msg.logMonoTime = base+control_frame*50_000_000
          msg.sentryState.capture = True
          msg.sentryState.triggerCamera = camera
          msg.sentryState.recordUntilMonoTime = time.monotonic_ns()+60_000_000_000 if recording else 0
          pm.send('sentryState', msg)
          msg = messaging.new_message('pandaStates', 1, valid=True)
          msg.pandaStates[0].pandaType = log.PandaState.PandaType.tres
          pm.send('pandaStates', msg)
          pm.send('deviceState', messaging.new_message('deviceState', valid=True))

        def await_state(predicate):
          deadline = time.monotonic()+5
          while time.monotonic() < deadline:
            assert process.poll() is None, (tmp_path/'recorder.log').read_text()
            control()
            status.update(50)
            if status.updated['sentryRecorderState'] and predicate(status['sentryRecorderState']):
              return
          pytest.fail('Recorder did not reach the expected preview state')

        def send_frames(start, end):
          for frame in range(start, end):
            control()
            for c, service in enumerate(STREAMS):
              msg = messaging.new_message(FIELDS[c], valid=True)
              msg.logMonoTime = base+frame*50_000_000
              data = getattr(msg, FIELDS[c])
              data.width, data.height = 526, 330
              if c >= 3:
                data.header, data.data, key = previews[c-3][frame]
              else:
                data.data, key = hevc[c], True
              idx = data.idx
              idx.type = log.EncodeIndex.Type.qcameraH264 if c >= 3 else log.EncodeIndex.Type.fullHEVC
              idx.frameId, idx.segmentId, idx.segmentNum = frame+10000*(c%3), frame, 7
              idx.flags = 8 if key else 0
              idx.timestampSof = idx.timestampEof = msg.logMonoTime
              pm.send(service, msg)
            time.sleep(.01)
          await_state(lambda _: True)  # Drain all six queues before changing the view.

        await_state(lambda _: True)
        send_frames(0, 20)
        assert not list((tmp_path/'routes').iterdir())
        recording, control_frame = True, 19
        await_state(lambda s: s.recording and s.previewCamera == 2)
        send_frames(20, 23)
        camera, control_frame = 1, 23
        await_state(lambda s: s.previewCamera == 1)
        send_frames(23, 63)  # Wait for a requested keyframe, or the regular GOP as fallback.
        camera, control_frame = 0, 63
        await_state(lambda s: s.previewCamera == 0)
        send_frames(63, 90)  # The recorder can splice any source cleanly within this SAME segment.
        recording = False
        await_state(lambda s: not s.recording)
        recording, camera, control_frame = True, 1, 90
        await_state(lambda s: s.recording and s.previewCamera == 1)
        send_frames(90, 120)
        recording = False
        await_state(lambda s: not s.recording)
        directories = sorted((tmp_path/'routes').iterdir())
        assert len(directories) == 2
        def next_key(start):
          return next(f for f in range(start, count) if previews[0][f][2])
        expected = [(1, f) for f in range(23)]+[(2, f) for f in range(next_key(23), 63)]+[(0, f) for f in range(next_key(63), 90)]
        for directory, full_count in zip(directories, (90, 120), strict=True):
          if full_count == 120:
            expected += [(2, f) for f in range(next_key(90), 120)]
          assert directory.name.endswith('--0') and not list(directory.glob('*.lock'))
          with av.open(str(directory/'qcamera.ts')) as video:
            assert video.streams.video[0].codec_context.name == 'h264'
            frames = list(video.decode(video=0))
          assert len(frames) == len(expected)
          assert all(a.pts < b.pts for a, b in zip(frames, frames[1:], strict=False))
          for image, (source, _) in zip(frames, expected, strict=True):
            assert abs(float(image.to_ndarray(format='rgb24')[10, 10].mean())-COLORS[source]) < 10
          indexes = [e.qNarrowRoadEncodeIdx for e in messages(directory/'qlog.zst') if e.which() == 'qNarrowRoadEncodeIdx']
          assert [i.frameId for i in indexes] == [10000*source+frame for source, frame in expected]
          assert [i.segmentId for i in indexes] == list(range(len(expected)))
          for c, filename in enumerate(FILES[:3]):
            assert (directory/filename).read_bytes() == hevc[c]*full_count
      finally:
        if process.poll() is None:
          process.send_signal(signal.SIGINT)
          process.wait(timeout=8)
        assert process.returncode == 0, (tmp_path/'recorder.log').read_text()


@pytest.mark.skipif(not (ROOT/'system/loggerd/encoderd').exists(), reason='build encoderd first')
@pytest.mark.parametrize('legacy_cameras_disabled', [True, False])
def test_encoder_private_watermark_buffers(monkeypatch, tmp_path, legacy_cameras_disabled):
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
    if legacy_cameras_disabled:
      # Old camera choices must not silently remove footage now that the row is gone.
      for key in ('SentryNarrowCamera', 'SentryWideCamera', 'SentryCabinCamera'):
        (Path(params.get_param_path())/key).write_bytes(b'0')
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
    sockets = [messaging.sub_sock(service, timeout=100) for service in STREAMS]
    control = messaging.PubMaster(['sentryState'])
    image = np.full(width*height*3//2, 128, dtype=np.uint8)
    packets = [[] for _ in sockets]
    indexes = [[] for _ in sockets]
    with (tmp_path/'encoder.log').open('w') as logfile:
      process = subprocess.Popen([str(ROOT/'system/loggerd/encoderd'), '--sentry'],
                                 cwd=ROOT/'system/loggerd', stdout=logfile, stderr=subprocess.STDOUT)
      try:
        for frame in range(65):
          assert process.poll() is None, (tmp_path/'encoder.log').read_text()
          now = time.monotonic_ns()
          if frame >= 20:
            msg = messaging.new_message('sentryState', valid=True)
            msg.sentryState.capture = True
            msg.sentryState.recordUntilMonoTime = now+60_000_000_000
            msg.sentryState.triggerCamera = 1 if 35 <= frame < 50 else 2
            control.send('sentryState', msg)
          for camera in (0, 1, 2):
            server.send(camera, image, frame, now, now)
          time.sleep(.05)
          for i, (socket, frames) in enumerate(zip(sockets, packets, strict=True)):
            msg = messaging.recv_one_or_none(socket)
            if msg:
              data = getattr(msg, FIELDS[i])
              frames.append(bytes(data.data))
              indexes[i].append((data.idx.frameId, bool(data.idx.flags & 8)))
        source = observer.recv(100)
        assert source is not None and np.all(source.data == 128), 'watermark modified a shared camera buffer'
      finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=8)
      assert process.returncode == 0, (tmp_path/'encoder.log').read_text()
      del observer, server
    for i, source in enumerate(packets):
      assert len(source) > 20
      assert all(source)
      if i < 3:
        continue  # PC encoderd uses FFVHUFF instead of the device's HEVC codec.
      with av.open(io.BytesIO(b''.join(source)), format='h264') as video:
        frames = list(video.decode(video=0))
      assert len(frames) > 20
      image = frames[-1].to_ndarray(format='gray')
      assert image.shape == (330, 526)
      assert image[290:308].max() > 200 and image[290:308].min() < 30
      assert image[:250].std() < 2
    for source, switch_frame in ((4, 20), (5, 35), (4, 50)):
      # Repeated 20 Hz control messages must request one frame at the handoff,
      # not continuously force I-frames or wait for the next one-second GOP.
      keys = [frame for frame, key in indexes[source] if key]
      handoff = next(frame for frame in keys if frame >= switch_frame)
      assert handoff <= switch_frame+2, (switch_frame, keys)
      assert sum(switch_frame <= frame < switch_frame+10 for frame in keys) == 1
      offset = next(i for i, (frame, _) in enumerate(indexes[source]) if frame == handoff)
      with av.open(io.BytesIO(b''.join(packets[source][offset:])), format='h264') as video:
        assert len(list(video.decode(video=0))) == len(packets[source])-offset
