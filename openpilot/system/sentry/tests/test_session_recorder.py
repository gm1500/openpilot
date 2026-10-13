"""Decode real camera packets across overlapping events and sparse session minutes."""
import subprocess
from fractions import Fraction

import numpy as np
import pytest

av = pytest.importorskip('av')

from openpilot.cereal import messaging
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.system.sentry.tests.test_recorder import COLORS, FIELDS, FILES, ROOT, messages

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
  first, second, base = (tmp_path/'routes.txt').read_text().splitlines()
  base = int(base)
  expected = {
    f'{first}--0': list(range(440))+list(range(600, 940)),
    f'{first}--3': list(range(3640, 3980)),
    f'{second}--0': list(range(6200, 6440)),
  }
  assert {d.name for d in root.iterdir()} == set(expected), 'quiet minutes must not create files'
  init = None
  for name, frame_numbers in expected.items():
    directory = root/name
    assert not list(directory.glob('*.lock'))
    events = messages(directory/'qlog.zst')
    assert not any(e.which() == 'selfdriveState' for e in events), 'no orange annotations'
    if name.startswith(first):
      current = events[0].as_builder().to_bytes()
      if init is None:
        init = current
      assert current == init, 'the armed-session time origin must not move'
      origin = base
    else:
      origin = base+310_000_000_000
    assert events[0].logMonoTime == origin
    number = int(name.rsplit('--', 1)[1])
    for c, filename in enumerate(FILES):
      field = ('narrowRoadEncodeIdx', 'wideRoadEncodeIdx', 'cabinEncodeIdx', 'qNarrowRoadEncodeIdx')[c]
      indexes = [getattr(e, field) for e in events if e.which() == field]
      assert [i.timestampEof for i in indexes] == [base+f*50_000_000 for f in frame_numbers]
      assert [i.segmentId for i in indexes] == list(range(len(frame_numbers)))
      assert all(i.segmentNum == number for i in indexes)
      with av.open(str(directory/filename)) as video:
        decoded = list(video.decode(video=0))
      assert len(decoded) == len(frame_numbers), f'{filename} lost frames at an event boundary'
      if c == 3:
        times = [float(f.pts*f.time_base) for f in decoded]
        assert np.diff(times) == pytest.approx(np.diff(frame_numbers)/20, abs=.001)
        for frame, n in zip(decoded, frame_numbers, strict=True):
          # The first event keeps its lens through overlap; a later cabin event
          # takes the previously unsaved pre-roll, while each HEVC keeps its lens.
          source = 2 if (600 <= n < 940 or n >= 6200) else 1
          assert abs(float(frame.to_ndarray(format='rgb24')[10, 10].mean())-COLORS[source]) < 10
      else:
        assert abs(float(decoded[0].to_ndarray(format='rgb24')[10, 10].mean())-COLORS[c]) < 10
    bookmarks = [(e.logMonoTime-base)/1e9 for e in events if e.which() == 'userBookmark']
    assert bookmarks == ({f'{first}--0': [10, 20, 45], f'{first}--3': [197], f'{second}--0': [320]})[name]
    assert [e.which() for e in messages(directory/'rlog.zst')] == [e.which() for e in events]
