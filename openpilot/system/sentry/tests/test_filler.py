"""Keep filler decodable with the comma 3X preview's original decoder setup."""
import hashlib
import os
from pathlib import Path
import re

import numpy as np
import pytest

av = pytest.importorskip('av')

# SPS/PPS observed in nine comma 3X Sentry preview files, including both lenses.
# These are codec metadata only; no recorded images are included in the test.
DEVICE_SPS = bytes.fromhex('67640015acb404215f5240368509a8')
DEVICE_PPS = bytes.fromhex('68ee06e2c0')
HEADER = Path(__file__).resolve().parents[2]/'loggerd/sentry_filler.h'


def units(data):
  return [u.rstrip(b'\0') for u in re.split(b'\x00\x00\x00?\x01', data) if u]


def annexb(nals):
  return b''.join(b'\x00\x00\x00\x01'+u for u in nals)


def embedded_filler():
  return bytes(int(value, 16) for value in re.findall(r'0x([0-9a-f]{2})', HEADER.read_text()))


def decode(packets):
  codec = av.CodecContext.create('h264', 'r')
  # Exercise the original codec configuration even when later samples omit
  # parameter sets, as a player initialized from the first segment may do.
  codec.extradata = annexb([DEVICE_SPS, DEVICE_PPS])
  for packet in packets:
    yield from codec.decode(av.Packet(packet))
  yield from codec.decode(None)


def test_filler_preserves_device_parameter_sets():
  headers = {u[0] & 31: u for u in units(embedded_filler()) if u[0] & 31 in (7, 8)}
  assert headers == {7: DEVICE_SPS, 8: DEVICE_PPS}, 'filler must not replace the hardware decoder configuration'


def test_filler_decodes_with_unchanged_device_configuration():
  picture = annexb([u for u in units(embedded_filler()) if u[0] & 31 == 5])
  frames = list(decode([picture]*62))
  assert len(frames) == 62
  assert all((f.width, f.height) == (526, 330) and f.key_frame for f in frames)
  assert all(np.max(f.to_ndarray(format='rgb24')) == 0 for f in frames)


def test_generator_matches_embedded_filler():
  # Load this checkout's development tool without selecting another worktree.
  import runpy
  generator = runpy.run_path(str(Path(__file__).resolve().parents[1]/'tools/generate_filler.py'))
  assert generator['filler_frame']() == embedded_filler()


@pytest.mark.skipif(not os.getenv('SENTRY_PREVIEW_SAMPLE'), reason='provide a local comma 3X qcamera.ts sample')
def test_real_preview_survives_filler_with_unchanged_configuration():
  with av.open(os.environ['SENTRY_PREVIEW_SAMPLE']) as video:
    packets = [bytes(p) for p in video.demux(video=0) if p.size]
  header_sets = {kind: {u for p in packets for u in units(p) if u[0] & 31 == kind} for kind in (7, 8)}
  assert header_sets == {7: {DEVICE_SPS}, 8: {DEVICE_PPS}}, 'sample uses a different hardware configuration'
  pictures = [annexb([u for u in units(p) if u[0] & 31 not in (7, 8)]) for p in packets]
  filler = annexb([u for u in units(embedded_filler()) if u[0] & 31 == 5])
  def digest(frame):
    return hashlib.sha256(frame.to_ndarray(format='yuv420p').tobytes()).digest()

  expected = [digest(f) for f in decode(pictures)]
  black = digest(next(decode([filler])))
  actual = [digest(f) for f in decode([filler]*7+pictures+[filler]*62+pictures)]
  assert actual == [black]*7+expected+[black]*62+expected
