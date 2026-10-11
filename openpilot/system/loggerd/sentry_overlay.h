#pragma once

#include <array>
#include <ctime>
#include <map>
#ifdef __COMMA_HARDWARE__
#include <media/msm_media_info.h>
#endif
#include "system/loggerd/encoder/encoder.h"

// Private low-resolution buffers: never draw onto camerad's shared buffers.
// The V4L input-done callback releases each buffer only after hardware consumes it.
class SentryOverlay {
  std::array<VisionBuf, 9> buffers;
  SafeQueue<VisionBuf *> available;
  SubMaster sm{{"sentryState"}};
  bool watermark = Params().getBool("SentryWatermark");
  std::string name;

  static void text(VisionBuf *buf, const std::string &line, int top) {
    static const std::map<char, std::array<uint8_t, 7>> font = {
      {'0',{14,17,19,21,25,17,14}}, {'1',{4,12,4,4,4,4,14}}, {'2',{14,17,1,2,4,8,31}},
      {'3',{30,1,1,14,1,1,30}}, {'4',{2,6,10,18,31,2,2}}, {'5',{31,16,16,30,1,1,30}},
      {'6',{14,16,16,30,17,17,14}}, {'7',{31,1,2,4,8,8,8}}, {'8',{14,17,17,14,17,17,14}},
      {'9',{14,17,17,15,1,1,14}}, {'-',{0,0,0,31,0,0,0}}, {'.',{0,0,0,0,0,12,12}},
      {':',{0,12,12,0,12,12,0}}, {'+',{0,4,4,31,4,4,0}}, {',',{0,0,0,0,0,4,8}},
      {'A',{14,17,17,31,17,17,17}}, {'B',{30,17,17,30,17,17,30}}, {'C',{14,17,16,16,16,17,14}},
      {'D',{30,17,17,17,17,17,30}}, {'E',{31,16,16,30,16,16,31}}, {'F',{31,16,16,30,16,16,16}},
      {'I',{14,4,4,4,4,4,14}}, {'L',{16,16,16,16,16,16,31}}, {'N',{17,25,25,21,19,19,17}}, {'O',{14,17,17,17,17,17,14}},
      {'P',{30,17,17,30,16,16,16}}, {'R',{30,17,17,30,20,18,17}}, {'S',{15,16,16,14,1,1,30}},
      {'T',{31,4,4,4,4,4,4}}, {'U',{17,17,17,17,17,17,14}}, {'V',{17,17,17,17,17,10,4}}, {'W',{17,17,17,21,21,21,10}},
    };
    int width = std::min<int>(buf->width, line.size()*12+12);
    for (int y = top; y < top+18; ++y) memset(buf->y+y*buf->stride, 16, width);
    for (int y = top/2; y < (top+18)/2; ++y) memset(buf->uv+y*buf->stride, 128, width);
    int left = 6;
    for (char ch : line) {
      auto glyph = font.find(ch);
      if (glyph != font.end() && left+10 < buf->width) {
        for (int y = 0; y < 7; ++y) for (int x = 0; x < 5; ++x) {
          if (glyph->second[y] & (1 << (4-x))) {
            for (int yy = 0; yy < 2; ++yy) for (int xx = 0; xx < 2; ++xx)
              buf->y[(top+2+y*2+yy)*buf->stride+left+x*2+xx] = 235;
          }
        }
      }
      left += 12;
    }
  }
public:
  explicit SentryOverlay(VisionStreamType stream) {
    name = stream == VISION_STREAM_NARROW_ROAD ? "FRONT" : stream == VISION_STREAM_WIDE_ROAD ? "WIDE" : "CABIN";
    const int width = 526, height = 330;
#ifdef __COMMA_HARDWARE__
    int stride = VENUS_Y_STRIDE(COLOR_FMT_NV12, width);
    int offset = stride * VENUS_Y_SCANLINES(COLOR_FMT_NV12, height);
    int size = VENUS_BUFFER_SIZE(COLOR_FMT_NV12, width, height);
#else
    int stride = width, offset = width*height, size = offset*3/2;
#endif
    for (auto &buf : buffers) {
      buf.allocate(size);
      buf.init_yuv(width, height, stride, offset);
      available.push(&buf);
    }
  }
  ~SentryOverlay() { for (auto &buf : buffers) buf.free(); }
  void release(VisionBuf *buf) { available.push(buf); }
  VisionBuf *prepare(VisionBuf *source, const VisionIpcBufExtra &extra) {
    VisionBuf *buf = nullptr;
    if (!available.try_pop(buf, 100)) return nullptr;
    source->sync(VISIONBUF_SYNC_FROM_DEVICE);
    for (size_t y = 0; y < buf->height; ++y) {
      const auto *row = source->y+(y*source->height/buf->height)*source->stride;
      for (size_t x = 0; x < buf->width; ++x) buf->y[y*buf->stride+x] = row[x*source->width/buf->width];
    }
    for (size_t y = 0; y < buf->height/2; ++y) {
      const auto *row = source->uv+(y*source->height/buf->height)*source->stride;
      for (size_t x = 0; x < buf->width; x += 2) {
        size_t sx = (x*source->width/buf->width) & ~size_t(1);
        buf->uv[y*buf->stride+x] = row[sx];
        buf->uv[y*buf->stride+x+1] = row[sx+1];
      }
    }
    sm.update(0);
    if (watermark) {
      time_t epoch = (nanos_since_epoch() - (nanos_since_boot()-extra.timestamp_eof))/1'000'000'000;
      tm utc;
      gmtime_r(&epoch, &utc);
      char stamp[32];
      strftime(stamp, sizeof(stamp), "%Y-%m-%d %H:%M:%S UTC", &utc);
      text(buf, std::string(stamp)+" "+name, 290);
      auto state = sm["sentryState"].getSentryState();
      text(buf, state.getLocationValid() ? util::string_format("POS %.5f,%.5f", state.getLatitude(), state.getLongitude()) : "POS UNAVAILABLE", 310);
    }
    buf->sync(VISIONBUF_SYNC_TO_DEVICE);
    return buf;
  }
};
