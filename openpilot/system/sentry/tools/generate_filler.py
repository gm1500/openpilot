"""Print a black IDR matching comma 3X previews (development only; PyAV/libx264)."""
import re
from fractions import Fraction

import av
import numpy as np

# Codec metadata observed in comma 3X hardware previews. Keeping the parameter
# sets identical avoids changing decoder configuration inside an HLS segment.
COMMA3X_SPS = bytes.fromhex('67640015acb404215f5240368509a8')
COMMA3X_PPS = bytes.fromhex('68ee06e2c0')


def filler_frame():
  codec = av.CodecContext.create('libx264', 'w')
  codec.width, codec.height, codec.pix_fmt = 526, 330, 'yuv420p'
  codec.time_base, codec.framerate = Fraction(1, 20), Fraction(20)
  codec.options = {'preset': 'medium', 'tune': 'zerolatency', 'qp': '20', 'profile': 'high', 'level': '2.1',
                   'x264-params': 'keyint=20:min-keyint=20:scenecut=0:repeat-headers=1:bframes=0:threads=1:'
                   + 'ref=1:weightp=0:chroma-qp-offset=0:psy=0:ipratio=1:deblock=0,0'}
  image = av.VideoFrame.from_ndarray(np.zeros((330, 526, 3), dtype=np.uint8), format='rgb24')
  packets = codec.encode(image)+codec.encode(None)
  assert len(packets) == 1 and packets[0].is_keyframe
  units = [u for u in re.split(b'\x00\x00\x00?\x01', bytes(packets[0])) if u and u[0] & 31 != 6]
  # Keep only SPS, PPS and IDR. Encoder-identification SEI is unnecessary.
  assert [u[0] & 31 for u in units] == [7, 8, 5]
  sps, pps, picture = units
  # All SPS fields through vui_parameters_present_flag must match. Replace
  # x264's VUI with the hardware VUI, which does not declare fixed 20 Hz timing.
  assert sps[:10] == COMMA3X_SPS[:10] and sps[10] & 0xc0 == COMMA3X_SPS[10] & 0xc0
  # x264 signals default deblocking explicitly; hardware uses the implicit
  # default. The PPS differs only in deblocking_filter_control_present_flag.
  assert pps == bytes.fromhex('68ee06f2c0')
  # This I-slice has qp_delta=0 and the default deblocking triplet '111'. With
  # the hardware PPS those three bits become valid CABAC alignment ones. The
  # slice body and its byte alignment stay unchanged. Fail regeneration if a
  # future encoder changes this header instead of emitting an incompatible IDR.
  assert picture[:4] == bytes.fromhex('658884ff')
  return b''.join(b'\x00\x00\x00\x01'+u for u in (COMMA3X_SPS, COMMA3X_PPS, picture))


if __name__ == '__main__':
  data = filler_frame()
  for offset in range(0, len(data), 16):
    print('  '+', '.join(f'0x{value:02x}' for value in data[offset:offset+16])+',')
