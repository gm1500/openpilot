"""Print the Sentry black-frame initializer (development only; requires PyAV/libx264)."""
import re
from fractions import Fraction

import av
import numpy as np


def filler_frame():
  codec = av.CodecContext.create('libx264', 'w')
  codec.width, codec.height, codec.pix_fmt = 526, 330, 'yuv420p'
  codec.time_base, codec.framerate = Fraction(1, 20), Fraction(20)
  codec.options = {'preset': 'medium', 'tune': 'zerolatency', 'crf': '40', 'profile': 'high', 'level': '2.1',
                   'x264-params': 'keyint=20:min-keyint=20:scenecut=0:repeat-headers=1:bframes=0:threads=1'}
  image = av.VideoFrame.from_ndarray(np.zeros((330, 526, 3), dtype=np.uint8), format='rgb24')
  packets = codec.encode(image)+codec.encode(None)
  assert len(packets) == 1 and packets[0].is_keyframe
  units = [u for u in re.split(b'\x00\x00\x00?\x01', bytes(packets[0])) if u]
  # Keep only SPS, PPS and IDR. Encoder-identification SEI is unnecessary.
  assert {u[0] & 31 for u in units} == {7, 8, 6, 5}
  return b''.join(b'\x00\x00\x00\x01'+u for u in units if u[0] & 31 != 6)


if __name__ == '__main__':
  data = filler_frame()
  for offset in range(0, len(data), 16):
    print('  '+', '.join(f'0x{value:02x}' for value in data[offset:offset+16])+',')
