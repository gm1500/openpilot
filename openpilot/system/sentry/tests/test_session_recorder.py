"""Decode real camera packets across overlapping events and sparse session minutes."""
import json
import shutil
import subprocess
from fractions import Fraction

import numpy as np
import pytest

av = pytest.importorskip('av')

from openpilot.cereal import messaging
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.system.sentry.tests.test_recorder import COLORS, FIELDS, FILES, FILLER_FRAME_ID, ROOT, assert_footage_timeline, messages

BINARY = ROOT/'system/loggerd/test_sentry_recorder'


@pytest.mark.skipif(not BINARY.exists(), reason='build test_sentry_recorder first')
def test_armed_session_overlaps_gaps_and_rearm(tmp_path, monkeypatch):
  root = tmp_path/'routes'
  monkeypatch.setenv('LOG_ROOT', str(root))
  for camera in range(6):
    codec = av.CodecContext.create('libx264' if camera >= 3 else 'libx265', 'w')
    codec.width, codec.height, codec.pix_fmt = 526, 330, 'yuv420p'
    codec.time_base, codec.framerate = Fraction(1, 20), Fraction(20, 1)
    options = {'preset': 'ultrafast', 'tune': 'zerolatency'}
    if camera >= 3:
      options['x264-params'] = 'keyint=20:min-keyint=20:scenecut=0:repeat-headers=1'
    else:
      options['x265-params'] = 'pools=1:frame-threads=1:log-level=error:keyint=20:min-keyint=20:scenecut=0:repeat-headers=1:bframes=0'
    codec.options = options
    packets = []
    for frame in range(40):
      image = np.full((330, 526, 3), COLORS[camera%3], dtype=np.uint8)
      image[100:140, 100+frame:140+frame] = 220
      picture = av.VideoFrame.from_ndarray(image, format='rgb24')
      picture.pts = frame
      packets.extend(codec.encode(picture))
    packets.extend(codec.encode(None))
    assert len(packets) == 40 and [i for i, p in enumerate(packets) if p.is_keyframe] == [0, 20]
    with (tmp_path/f'{camera}.capnp').open('wb') as file:
      for packet in packets:
        msg = messaging.new_message(FIELDS[camera], valid=True)
        data = getattr(msg, FIELDS[camera])
        data.width, data.height, data.data = 526, 330, bytes(packet)
        data.idx.type = 'qcameraH264' if camera >= 3 else 'fullHEVC'
        data.idx.flags = 8 if packet.is_keyframe else 0
        file.write(msg.to_bytes())
  with OpenpilotPrefix():
    result = subprocess.run([str(BINARY), str(tmp_path)], capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')
    validate_session(root, tmp_path)


def validate_session(root, tmp_path):
  first, second, filled, long, base = (tmp_path/'routes.txt').read_text().splitlines()
  base = int(base)
  expected = {
    f'{first}--0': list(range(440))+list(range(600, 940)),
    f'{first}--3': list(range(3640, 3980)),
    f'{second}--0': list(range(6200, 7400)),
    f'{second}--1': list(range(7400, 7640)),
    f'{filled}--0': list(range(8000, 8240)),
    f'{filled}--1': list(range(9700, 10400)),
    f'{filled}--2': list(range(10440, 10940)),
    f'{long}--0': list(range(20000, 20240)),
    f'{long}--600': list(range(740040, 740380)),
  }
  orange = {f'{first}--0': [(0, 22), (30, 47)], f'{first}--3': [(182, 199)],
            f'{second}--0': [(310, 370)], f'{second}--1': [(370, 382)],
            f'{filled}--0': [(400, 412)], f'{filled}--1': [(485, 520)], f'{filled}--2': [(522, 547)],
            f'{long}--0': [(1000, 1012)], f'{long}--600': [(37002, 37019)]}
  bookmarks_by_segment = {f'{first}--0': [10, 20, 45], f'{first}--3': [197],
                          f'{second}--0': [320, 330, 340, 350, 360], f'{second}--1': [370, 380],
                          f'{filled}--0': [410], f'{filled}--1': [], f'{filled}--2': [537, 545],
                          f'{long}--0': [1010], f'{long}--600': [37017]}
  origins = {first: base, second: base+310_000_000_000, filled: base+400_000_000_000, long: base+1_000_000_000_000}
  gaps = {f'{first}--{n}' for n in (1, 2)} | {f'{long}--{n}' for n in range(1, 600)}
  colour_route = (tmp_path/'colour_route.txt').read_text().strip()
  assert {d.name for d in root.iterdir()} == set(expected) | gaps | {f'{colour_route}--{n}' for n in range(2)}, \
    'only bridge whole minutes between saved events'
  validate_colours(root, colour_route, base, tmp_path)
  inits = {}
  full_inits = {}
  for name, frame_numbers in expected.items():
    directory = root/name
    assert not list(directory.glob('*.lock'))
    events = messages(directory/'qlog.zst')
    for filename in ('qlog.zst', 'rlog.zst'):
      logged = messages(directory/filename)
      assert_footage_timeline(logged)
      states = [e for e in logged if e.which() == 'selfdriveState']
      if states[0].selfdriveState.alertStatus == 'normal':
        states = states[1:]
      spans = [((a.logMonoTime-base)/1e9, (b.logMonoTime-base)/1e9) for a, b in zip(states[::2], states[1::2], strict=True)]
      assert spans == orange[name], 'orange must cover footage, with no filler state messages during quiet gaps'
    route, number = name.rsplit('--', 1)
    current = events[0].as_builder().to_bytes()
    assert current == inits.setdefault(route, current), 'the armed-session time origin must not move'
    full_inits[route] = events[0].initData
    origin = origins[route]
    assert events[0].logMonoTime == origin
    number = int(name.rsplit('--', 1)[1])
    for c, filename in enumerate(FILES):
      field = ('narrowRoadEncodeIdx', 'wideRoadEncodeIdx', 'cabinEncodeIdx', 'qNarrowRoadEncodeIdx')[c]
      indexes = [getattr(e, field) for e in events if e.which() == field]
      real = [i for i in indexes if i.frameId != FILLER_FRAME_ID]
      assert [i.timestampEof for i in real] == [base+f*50_000_000 for f in frame_numbers]
      assert [i.segmentId for i in indexes] == list(range(len(indexes)))
      assert [i.segmentIdEncode for i in indexes] == list(range(len(indexes)))
      assert all(i.segmentNum == number for i in indexes)
      with av.open(str(directory/filename)) as video:
        decoded = list(video.decode(video=0))
      assert len(decoded) == len(indexes), f'{filename} lost frames at an event boundary'
      if c == 3:
        times = [float(f.pts*f.time_base) for f in decoded]
        assert np.diff(times) == pytest.approx(np.diff([i.timestampEof for i in indexes])/1e9, abs=.001)
        assert max(np.diff(times)) <= 1.001, 'no holes between real footage and filler'
        assert indexes[0].timestampEof == origin+number*60_000_000_000
        for frame, idx in zip(decoded, indexes, strict=True):
          assert (frame.width, frame.height) == (526, 330)
          if idx.frameId == FILLER_FRAME_ID:
            assert np.max(frame.to_ndarray(format='rgb24')) <= 1
            continue
          n = (idx.timestampEof-base)//50_000_000
          # The first event keeps its lens through overlap; a later cabin event
          # takes the previously unsaved pre-roll, while each HEVC keeps its lens.
          source = 2 if (600 <= n < 940 or route == second) else 1
          assert abs(float(frame.to_ndarray(format='rgb24')[10, 10].mean())-COLORS[source]) < 10
      else:
        assert abs(float(decoded[0].to_ndarray(format='rgb24')[10, 10].mean())-COLORS[c]) < 10
    bookmarks = [(e.logMonoTime-base)/1e9 for e in events if e.which() == 'userBookmark']
    assert bookmarks == bookmarks_by_segment[name]
    assert [e.which() for e in messages(directory/'rlog.zst')] == [e.which() for e in events]

  sizes, video_sizes = [], []
  for name in sorted(gaps):
    directory = root/name
    assert {f.name for f in directory.iterdir()} == {'.sentry', 'qlog.zst', 'qcamera.ts'}, 'no HEVC/rlog or sensor data in a gap'
    events = messages(directory/'qlog.zst')
    assert [e.which() for e in events] == ['initData', 'sentinel', 'selfdriveState']+['qNarrowRoadEncodeIdx']*62+['sentinel']
    route, number = name.rsplit('--', 1)
    origin = origins[route]
    start = origin+int(number)*60_000_000_000
    assert [e.logMonoTime for e in events[:3]] == [origin, start, start]
    assert events[-1].logMonoTime == start+60_000_000_000-1
    indexes = [e.qNarrowRoadEncodeIdx for e in events[3:-1]]
    times = [start+n*1_000_000_000 for n in range(60)]+[start+59_900_000_000, start+59_950_000_000]
    assert [i.timestampEof for i in indexes] == times
    assert all(i.frameId == FILLER_FRAME_ID and i.flags == 8 for i in indexes)
    assert [i.segmentId for i in indexes] == list(range(62))
    assert [i.segmentIdEncode for i in indexes] == list(range(62))
    assert all(i.segmentNum == int(number) for i in indexes)
    assert events[1].sentinel.type == 'startOfSegment'
    assert events[-1].sentinel.type == 'endOfSegment'
    assert events[2].valid and not events[2].selfdriveState.enabled and not events[2].selfdriveState.active
    assert events[2].selfdriveState.state == 'disabled' and events[2].selfdriveState.alertStatus == 'normal'
    full = full_inits[route]
    compact = events[0].initData
    for field in ('wallTimeNanos', 'dongleId', 'deviceType', 'version', 'osVersion', 'dirty', 'gitCommit',
                  'gitCommitDate', 'gitBranch', 'gitRemote', 'gitSrcCommit', 'gitSrcCommitDate'):
      assert getattr(compact, field) == getattr(full, field)
    assert not compact.params.entries and not compact.commands.entries and not compact.kernelArgs
    size = (directory/'qlog.zst').stat().st_size
    assert 0 < size < 4096, 'one quiet minute must remain a small compressed metadata file'
    sizes.append(size)
    video_size = (directory/'qcamera.ts').stat().st_size
    assert 0 < video_size < 48_000
    video_sizes.append(video_size)
    if int(number) in (1, 2, 599):
      with av.open(str(directory/'qcamera.ts')) as video:
        frames = list(video.decode(video=0))
      assert len(frames) == 62 and all(f.key_frame for f in frames)
      assert np.diff([float(f.pts*f.time_base) for f in frames]) == pytest.approx(np.diff(times)/1e9, abs=.001)
      assert all(np.max(f.to_ndarray(format='rgb24')) <= 1 for f in frames)
  print(f'Gap logs: {len(sizes)} files, {sum(sizes)} bytes, max {max(sizes)} bytes/minute;',
        f'video: {sum(video_sizes)} bytes, max {max(video_sizes)} bytes/minute;',
        f'finish backlog: {(tmp_path/"gap_finish_ms.txt").read_text().strip()} ms')
  # Keep a small, anonymous fixture for the optional Connect hls.js contract
  # check after OpenpilotPrefix cleans up the native route directories.
  playback = tmp_path/'connect-preview'
  playback.mkdir()
  for number in range(4):
    shutil.copyfile(root/f'{first}--{number}'/'qcamera.ts', playback/f'{number}.ts')


def validate_colours(root, route, base, tmp_path):
  expected = [
    [(600, 'orange'), (610, 'green'), (611, 'red'), (613, 'orange'), (620, 'green'),
     (622, 'blue'), (644, 'orange'), (659, 'red'), (660, 'blue')],
    [(660, 'green'), (662, 'orange'), (670, 'green'), (672, 'blue')],
  ]
  export = []
  for number, transitions in enumerate(expected):
    for filename in ('qlog.zst', 'rlog.zst'):
      logged = messages(root/f'{route}--{number}'/filename)
      states = [e for e in logged if e.which() == 'selfdriveState']
      actual = []
      for e in states:
        state = e.selfdriveState
        assert e.valid and state.state == 'disabled' and not state.active and not state.engageable
        mode = 'green' if state.enabled else 'red' if state.alertStatus == 'critical' else \
               'orange' if state.alertStatus == 'userPrompt' else 'blue'
        actual.append(((e.logMonoTime-base)/1e9, mode))
        assert str(state.alertStatus) == {'green': 'normal', 'red': 'critical', 'orange': 'userPrompt', 'blue': 'normal'}[mode]
        assert state.alertType == {'green': 'sentryAudio/permanent', 'red': 'sentryGyro/permanent',
                                   'orange': 'sentryRecording/permanent', 'blue': ''}[mode]
      assert actual == transitions
      if filename == 'qlog.zst':
        for e in logged:
          offset = round((e.logMonoTime-base-600_000_000_000)/1e6)
          if e.which() == 'selfdriveState':
            export.append({'type': 'state', 'route_offset_millis': offset, 'route_offset_nanos': 0,
                           'data': {'enabled': e.selfdriveState.enabled, 'state': 'disabled',
                                    'alertStatus': {'normal': 0, 'userPrompt': 1, 'critical': 2}[str(e.selfdriveState.alertStatus)]}})
          elif e.which() == 'userBookmark':
            export.append({'type': 'user_bookmark', 'route_offset_millis': offset, 'route_offset_nanos': 0, 'data': {}})
  (tmp_path/'connect-events.json').write_text(json.dumps({'duration': 72000, 'events': export}))
