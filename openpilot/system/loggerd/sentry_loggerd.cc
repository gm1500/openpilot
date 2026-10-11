// RAM-only parked capture. Native route files are created only after a motion trigger.
#include <sys/statvfs.h>

#include <array>
#include <climits>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>

#include "system/loggerd/encoder/encoder.h"
#include "system/loggerd/loggerd.h"
#include "system/loggerd/video_writer.h"
#include "system/sentry/state.h"

namespace fs = std::filesystem;
ExitHandler do_exit;
constexpr uint64_t SECOND = 1'000'000'000;
constexpr size_t RAM_LIMIT = 128 * 1024 * 1024;
constexpr uint64_t DISK_RESERVE = 5ULL * 1024 * 1024 * 1024;

int setting(const char *key, int fallback, int lo, int hi) {
  try { return std::clamp(std::stoi(Params().get(key)), lo, hi); } catch (...) { return fallback; }
}

struct Packet {
  int camera;  // -1 is an ordinary log message
  uint64_t time;
  // Keep Cap'n Proto's aligned allocation. Individual words are deliberately
  // non-copyable under Clang; moving the owning array also avoids a second copy.
  kj::Array<capnp::word> words;
  Packet(int camera_, cereal::Event::Reader event) : camera(camera_), time(event.getLogMonoTime()) {
    capnp::MallocMessageBuilder builder;
    builder.setRoot(event);
    words = capnp::messageToFlatArray(builder);
  }
  size_t bytes() const { return words.size() * sizeof(capnp::word); }
};
struct Segment {
  uint64_t start = 0, end = 0;
  size_t bytes = 0;
  std::deque<Packet> packets;
};

// All files stay locked until video and logs are finalized. The uploader recognizes
// .sentry and queues every selected camera on unmetered connections.
class SavedSegment {
public:
  std::string path;
  std::unique_ptr<ZstdFileWriter> rlog, qlog;
  std::array<std::unique_ptr<VideoWriter>, 4> video;
  std::array<uint32_t, 4> frame_count = {};
  int number;
  uint64_t last_time = 0;

  SavedSegment(const std::string &route, int number_, kj::ArrayPtr<capnp::word> init, uint64_t start)
      : path(Path::log_root() + "/" + route + "--" + std::to_string(number_)), number(number_) {
    fs::create_directories(path);
    std::ofstream{path + "/sentry.lock"} << getpid() << "\n";
    std::ofstream{path + "/.sentry"} << "1\n";
    rlog = std::make_unique<ZstdFileWriter>(path + "/rlog.zst", 3);
    qlog = std::make_unique<ZstdFileWriter>(path + "/qlog.zst", 3);
    capnp::FlatArrayMessageReader init_reader(init);
    MessageBuilder init_message;
    init_message.setRoot(init_reader.getRoot<cereal::Event>());
    auto init_event = init_message.getRoot<cereal::Event>();
    init_event.setLogMonoTime(start);
    init_event.getInitData().setWallTimeNanos(nanos_since_epoch()-(nanos_since_boot()-start));
    write(init_message.toBytes());
    std::string cp_bytes = Params().get("CarParamsPersistent");
    if (!cp_bytes.empty()) {
      AlignedBuffer aligned;
      capnp::FlatArrayMessageReader cp_reader(aligned.align(cp_bytes.data(), cp_bytes.size()));
      MessageBuilder cp_message;
      auto cp_event = cp_message.initEvent();
      cp_event.setLogMonoTime(start);
      cp_event.setCarParams(cp_reader.getRoot<cereal::CarParams>());
      write(cp_message.toBytes());
    }
    sentinel(number ? SentinelType::START_OF_SEGMENT : SentinelType::START_OF_ROUTE, start);
  }
  void write(kj::ArrayPtr<const kj::byte> bytes) {
    rlog->write(const_cast<kj::byte *>(bytes.begin()), bytes.size());
    qlog->write(const_cast<kj::byte *>(bytes.begin()), bytes.size());
  }
  void sentinel(SentinelType type, uint64_t time) {
    MessageBuilder msg;
    auto event = msg.initEvent();
    event.setLogMonoTime(time);
    event.initSentinel().setType(type);
    write(msg.toBytes());
  }
  void append(const Packet &packet) {
    capnp::FlatArrayMessageReader reader(packet.words.asPtr());
    auto event = reader.getRoot<cereal::Event>();
    last_time = std::max(last_time, packet.time);
    if (packet.camera < 0) {
      write(packet.words.asBytes());
      return;
    }
    int c = packet.camera;
    const auto &info = sentry_encoders[c];
    auto data = (event.*info.get_encode_data_func)();
    auto idx = data.getIdx();
    bool keyframe = idx.getFlags() & V4L2_BUF_FLAG_KEYFRAME;
    if (!video[c]) {
      // A truncated RAM segment must still start with a decodable frame.
      if (!keyframe) return;
      video[c] = std::make_unique<VideoWriter>(path.c_str(), info.filename,
        idx.getType() != cereal::EncodeIndex::Type::FULL_H_E_V_C, data.getWidth(), data.getHeight(), info.fps, idx.getType());
      auto header = data.getHeader();
      video[c]->write(const_cast<uint8_t *>(header.begin()), header.size(), idx.getTimestampEof()/1000, true, false);
    }
    auto bytes = data.getData();
    video[c]->write(const_cast<uint8_t *>(bytes.begin()), bytes.size(), idx.getTimestampEof()/1000, false, keyframe);
    // Keep source frame/timestamps but rebase segment and in-file frame indexes.
    capnp::MallocMessageBuilder index_builder;
    index_builder.setRoot(idx);
    auto rebased = index_builder.getRoot<cereal::EncodeIndex>();
    rebased.setSegmentNum(number);
    rebased.setSegmentId(frame_count[c]++);
    MessageBuilder msg;
    auto out = msg.initEvent(event.getValid());
    out.setLogMonoTime(event.getLogMonoTime());
    (out.*info.set_encode_idx_func)(rebased.asReader());
    write(msg.toBytes());
  }
  void flush() {
    rlog->flush(); qlog->flush();
    for (auto &v : video) if (v) v->flush();
  }
  void close(bool last) {
    sentinel(last ? SentinelType::END_OF_ROUTE : SentinelType::END_OF_SEGMENT, last_time);
    for (auto &v : video) v.reset();
    rlog.reset();
    qlog.reset();
    // Flush completed event segments to storage before letting the uploader see them.
    for (const auto &file : fs::directory_iterator(path)) {
      if (!file.is_regular_file()) continue;
      int fd = open(file.path().c_str(), O_RDONLY);
      if (fd >= 0) { fsync(fd); ::close(fd); }
    }
    fs::remove(path + "/sentry.lock");
  }
};

class Recorder {
public:
  std::map<int, Segment> ring;
  std::map<int, std::unique_ptr<SavedSegment>> saved;
  std::array<int, 4> source_segments = {-1, -1, -1, -1};
  std::array<uint64_t, 4> source_times = {};
  size_t buffered = 0;
  std::string route, error;
  kj::Array<capnp::word> init = logger_build_init_data(true);
  int first_segment = 0, closed_segment = -1;
  uint64_t event_start = 0, retry_after = 0, storage_check = 0;

  ~Recorder() { finish(); }
  bool disk_ok() {
    struct statvfs disk;
    if (statvfs(Path::log_root().c_str(), &disk) || disk.f_bavail * disk.f_frsize < DISK_RESERVE ||
        static_cast<double>(disk.f_bavail) / std::max<uint64_t>(disk.f_blocks, 1) < .10) {
      error = "Low storage";
      return false;
    }
    // Match normal route cleanup thresholds so low storage can recover without
    // a separate permanent event budget or manual deletion.
    error.clear();
    return true;
  }
  void finish() {
    for (auto it = saved.begin(); it != saved.end(); ++it) it->second->close(std::next(it) == saved.end());
    saved.clear();
    route.clear();
  }
  void reset() {
    finish();
    ring.clear();
    buffered = 0;
    source_segments.fill(-1);
    source_times.fill(0);
  }
  void save(int segment, const Packet &packet) {
    if (segment < first_segment || segment <= closed_segment) return;
    auto &out = saved[segment];
    if (!out) out = std::make_unique<SavedSegment>(route, segment-first_segment, init.asPtr(), packet.time);
    out->append(packet);
  }
  void append(int camera, cereal::Event::Reader event, uint64_t now) {
    int segment = ring.empty() ? 0 : ring.rbegin()->first;
    if (camera >= 0) {
      auto idx = (event.*sentry_encoders[camera].get_encode_data_func)().getIdx();
      segment = idx.getSegmentNum();
      if (source_segments[camera] > segment) reset();  // encoder restart: separate routes
      source_segments[camera] = segment;
      source_times[camera] = now;
    } else if (ring.empty()) {
      return;
    }
    auto &entry = ring[segment];
    Packet packet(camera, event);
    if (!entry.start) entry.start = packet.time;
    entry.end = std::max(entry.end, packet.time);
    entry.bytes += packet.bytes();
    buffered += packet.bytes();
    if (!route.empty()) save(segment, packet);
    entry.packets.emplace_back(std::move(packet));
    const uint64_t window = setting("SentryPreMinutes", 5, 1, 5) * 60 * SECOND;
    while (ring.size() > 1 && (buffered > RAM_LIMIT || ring.begin()->second.end + window < now)) {
      buffered -= ring.begin()->second.bytes;
      ring.erase(ring.begin());
    }
    // A pathological bitrate cannot exceed the memory budget even in a single segment.
    while (buffered > RAM_LIMIT && !ring.empty()) {
      auto &old = ring.begin()->second;
      size_t bytes = old.packets.front().bytes();
      old.packets.pop_front(); old.bytes -= bytes; buffered -= bytes;
      if (old.packets.empty()) ring.erase(ring.begin());
      else old.start = old.packets.front().time;
    }
  }
  void tick(uint64_t record_until, uint64_t now) {
    if (!route.empty()) {
      if (now >= record_until || now-event_start >= 15*60*SECOND) finish();
      else if (now-storage_check > 5*SECOND) {
        storage_check = now;
        if (!disk_ok()) { finish(); retry_after = now+30*SECOND; }
      }
      // Finalize historical segments once all recently seen camera streams advanced.
      int oldest = INT_MAX;
      for (int c = 0; c < 4; ++c) {
        if (source_times[c] && now-source_times[c] < 3*SECOND) oldest = std::min(oldest, source_segments[c]);
      }
      while (saved.size() > 1 && saved.begin()->first < oldest) {
        auto it = saved.begin();
        closed_segment = it->first;
        it->second->close(false);
        saved.erase(it);
      }
    }
    if (route.empty() && record_until > now && now >= retry_after && !ring.empty()) {
      if (!disk_ok()) { retry_after = now+30*SECOND; return; }
      route = logger_get_identifier("RouteCount");
      event_start = storage_check = now;
      uint64_t window = setting("SentryPreMinutes", 5, 1, 5)*60*SECOND;
      uint64_t cutoff = now > window ? now-window : 0;
      auto begin = ring.begin();
      // Whole native minute segments, at most the selected pre-event window.
      while (std::next(begin) != ring.end() && begin->second.start < cutoff) ++begin;
      first_segment = begin->first;
      closed_segment = first_segment-1;
      for (auto it = begin; it != ring.end(); ++it) for (const auto &packet : it->second.packets) save(it->first, packet);
    }
  }
};

int main() {
  fs::create_directories(Path::log_root());
  PubMaster pm({"sentryRecorderState"});
  SubMaster sm({"sentryState", "pandaStates", "deviceState", "clocks", "narrowRoadCameraState", "wideRoadCameraState", "cabinCameraState"});
  std::unique_ptr<Context> context(Context::create());
  std::array<std::unique_ptr<SubSocket>, 4> sockets;
  for (int c = 0; c < 4; ++c) {
    const auto &service = services.at(sentry_encoders[c].publish_name);
    // Raw sockets must use the same queue size as the encoder's PubMaster.
    // The default 1 MiB mapping truncates these 10 MiB queues and breaks both
    // processes as soon as video advances past the smaller mapping.
    sockets[c].reset(SubSocket::create(context.get(), service.name, "127.0.0.1", false, true, service.queue_size));
  }
  Recorder recorder;
  AlignedBuffer aligned;
  // A previous power interruption may leave locks. Only recover sentry-owned routes.
  for (const auto &dir : fs::directory_iterator(Path::log_root())) {
    if (!dir.is_directory() || !fs::exists(dir.path()/".sentry") || !fs::exists(dir.path()/"sentry.lock")) continue;
    std::ofstream{dir.path()/".sentry-incomplete"} << "Recovered after interrupted recording\n";
    for (const auto &file : fs::directory_iterator(dir.path())) {
      if (file.path().extension() == ".lock") fs::remove(file.path());
    }
  }
  uint64_t published = 0, location_logged = 0, flushed = 0;
  while (!do_exit) {
    sm.update(0);
    uint64_t now = nanos_since_boot();
    auto sentry = sentry_state(sm["sentryState"], sm.rcv_time("sentryState"), now);
    auto state = sentry.value_or(cereal::SentryState::Reader{});
    bool ignition = false;
    bool has_panda = sm["pandaStates"].isPandaStates() && sm["pandaStates"].getPandaStates().size() > 0;
    if (has_panda) {
      for (auto panda : sm["pandaStates"].getPandaStates()) ignition |= panda.getIgnitionLine() || panda.getIgnitionCan();
    }
    bool capture = sentry && state.getCapture() && has_panda &&
                   sm.valid("pandaStates") && sm.rcv_time("pandaStates") <= now && sm.rcv_time("pandaStates")+2*SECOND > now &&
                   sm.valid("deviceState") && sm.rcv_time("deviceState") <= now && sm.rcv_time("deviceState")+2*SECOND > now &&
                   sm["deviceState"].isDeviceState() &&
                   !ignition && !sm["deviceState"].getDeviceState().getStarted();
    if (!capture) recorder.reset();
    for (int c = 0; c < 4; ++c) {
      // Bound each drain so control/ignition updates cannot be starved by a bad publisher.
      for (int n = 0; n < 64; ++n) {
        std::unique_ptr<Message> message(sockets[c]->receive(true));
        if (!message) break;
        if (capture) {
          capnp::FlatArrayMessageReader reader(aligned.align(message.get()));
          recorder.append(c, reader.getRoot<cereal::Event>(), now);
        }
      }
    }
    if (capture) {
      for (const char *service : {"sentryState", "deviceState", "clocks", "narrowRoadCameraState", "wideRoadCameraState", "cabinCameraState"}) {
        if (sm.updated(service)) recorder.append(-1, sm[service], now);
      }
      if (state.getLocationValid() && now-location_logged >= SECOND) {
        // Log-only stationary position estimate, never published as a live GPS fix.
        // Its original sample time/accuracy and SLC/GPS provenance remain in SentryState.
        MessageBuilder position;
        auto event = position.initEvent();
        auto gps = event.initGpsLocation();
        gps.setSource(cereal::GpsLocationData::SensorSource::FUSION);
        gps.setLatitude(state.getLatitude());
        gps.setLongitude(state.getLongitude());
        gps.setHorizontalAccuracy(state.getLocationAccuracy());
        gps.setUnixTimestampMillis(state.getLocationSavedAt()*1000);
        gps.setHasFix(true);
        gps.setFlags(1);
        recorder.append(-1, event.asReader(), now);
        location_logged = now;
      }
      recorder.tick(state.getRecordUntilMonoTime(), now);
    }
    if (now-flushed >= SECOND) {
      for (auto &[number, segment] : recorder.saved) segment->flush();
      flushed = now;
    }
    if (now-published > SECOND/2) {
      MessageBuilder message;
      auto status = message.initEvent().initSentryRecorderState();
      status.setRecording(!recorder.route.empty());
      status.setRoute(recorder.route);
      status.setBufferBytes(recorder.buffered);
      status.setBufferSeconds(recorder.ring.empty() ? 0 : (now-recorder.ring.begin()->second.start)/double(SECOND));
      status.setError(recorder.error);
      pm.send("sentryRecorderState", message);
      published = now;
    }
    util::sleep_for(10);
  }
}
