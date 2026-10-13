// RAM-only parked capture. Native route files are created only after a motion trigger.
#include <sys/statvfs.h>

#include <array>
#include <climits>
#include <cmath>
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

int history_seconds() {
  // Match config.setting's nearest-preset rule, including malformed values.
  try {
    std::string raw = Params().get("SentryPreSeconds");
    size_t parsed = 0;
    double value = std::stod(raw, &parsed);
    if (!std::isfinite(value) || raw.find_first_not_of(" \t\r\n", parsed) != std::string::npos) return 60;
    int selected = 15;
    for (int seconds : {30, 60}) {
      if (std::abs(seconds-value) < std::abs(selected-value)) selected = seconds;
    }
    return selected;
  } catch (...) { return 60; }
}

struct Packet {
  int camera;  // -1 is an ordinary log message
  uint64_t time;
  uint64_t frame_time;
  bool keyframe = false;
  bool saved = false;  // Overlapping event windows must not duplicate frames or logs.
  // Keep Cap'n Proto's aligned allocation. Individual words are deliberately
  // non-copyable under Clang; moving the owning array also avoids a second copy.
  kj::Array<capnp::word> words;
  Packet(int camera_, cereal::Event::Reader event) : camera(camera_), time(event.getLogMonoTime()) {
    frame_time = time;
    if (camera >= 0) {
      auto idx = (event.*sentry_encoders[camera].get_encode_data_func)().getIdx();
      frame_time = idx.getTimestampEof();
      keyframe = idx.getFlags() & V4L2_BUF_FLAG_KEYFRAME;
    }
    capnp::MallocMessageBuilder builder;
    builder.setRoot(event);
    words = capnp::messageToFlatArray(builder);
  }
  size_t bytes() const { return words.size() * sizeof(capnp::word); }
};
struct PreviewSelection {
  uint64_t time;
  int camera;
};
int preview_at(const std::vector<PreviewSelection> &previews, uint64_t time, int camera) {
  for (const auto &selection : previews) {
    if (selection.time > time) break;
    camera = selection.camera;
  }
  return camera;
}
struct Segment {
  uint64_t end = 0;
  size_t bytes = 0;
  std::deque<Packet> packets;
  std::vector<PreviewSelection> previews;  // Retain the original view of overlapping history.
};

// All files stay locked until video and logs are finalized. The ordinary uploader
// sends qlog/preview; full logs and camera videos remain available for Connect requests.
class SavedSegment {
public:
  std::string path;
  std::unique_ptr<ZstdFileWriter> rlog, qlog;
  std::array<std::unique_ptr<VideoWriter>, 4> video;
  std::array<uint32_t, 4> frame_count = {};
  std::array<uint64_t, 4> frame_time = {};
  std::array<bool, 4> resume = {};
  int number;
  int preview_encoder = -1;
  uint64_t preview_time = 0;
  uint64_t last_time = 0;

  SavedSegment(const std::string &route, int number_, kj::ArrayPtr<capnp::word> init, uint64_t start)
      : path(Path::log_root() + "/" + route + "--" + std::to_string(number_)), number(number_) {
    fs::create_directories(path);
    std::ofstream{path + "/sentry.lock"} << getpid() << "\n";
    std::ofstream{path + "/.sentry"} << "1\n";
    rlog = std::make_unique<ZstdFileWriter>(path + "/rlog.zst", 3);
    qlog = std::make_unique<ZstdFileWriter>(path + "/qlog.zst", 3);
    // Like loggerd, every segment repeats the SAME route-level initData.
    // Advancing its timestamps per segment moves Connect's route origin and
    // can hide the earlier minutes from playback and upload requests.
    write(init.asBytes());
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
  void timeline(uint64_t time, bool available) {
    // Saved-log annotation only, never a live driving alert. Connect paints
    // userPrompt orange until normal; bookmarks remain yellow above it.
    MessageBuilder msg;
    auto event = msg.initEvent(true);
    event.setLogMonoTime(time);
    auto state = event.initSelfdriveState();
    state.setState(cereal::SelfdriveState::OpenpilotState::DISABLED);
    state.setEnabled(false);
    state.setActive(false);
    state.setEngageable(false);
    state.setAlertStatus(available ? cereal::SelfdriveState::AlertStatus::USER_PROMPT : cereal::SelfdriveState::AlertStatus::NORMAL);
    state.setAlertSize(available ? cereal::SelfdriveState::AlertSize::SMALL : cereal::SelfdriveState::AlertSize::NONE);
    if (available) {
      state.setAlertType("sentryRecording/permanent");
      state.setAlertText1("Sentry recording");
    }
    write(msg.toBytes());
  }
  void pause() { resume.fill(true); }
  void append(const Packet &packet, const std::vector<PreviewSelection> &previews) {
    capnp::FlatArrayMessageReader reader(packet.words.asPtr());
    auto event = reader.getRoot<cereal::Event>();
    last_time = std::max(last_time, packet.time);
    if (packet.camera < 0) {
      write(packet.words.asBytes());
      return;
    }
    int c = std::min(packet.camera, 3);
    const auto &info = sentry_encoders[packet.camera];
    auto data = (event.*info.get_encode_data_func)();
    auto idx = data.getIdx();
    bool keyframe = idx.getFlags() & V4L2_BUF_FLAG_KEYFRAME;
    if (frame_time[c] && packet.frame_time <= frame_time[c]) return;
    // A later event may append to an unfinished minute after an unsaved gap.
    // Rejoin every encoder on an IDR, including the three separate HEVC files.
    // Overlapping pre-roll can instead supply the very next frame with no gap.
    bool restarting = resume[c] && frame_time[c] && packet.frame_time-frame_time[c] > SECOND/info.fps*3/2;
    if (restarting && !keyframe) return;
    bool switching = false;
    if (packet.camera >= 3) {
      int desired = preview_at(previews, packet.frame_time, previews.front().camera);
      // Only qcamera.ts changes lenses. Never splice a dependent frame from a
      // different encoder or move its timestamps backwards. The player holds
      // the last frame until the new source's requested keyframe arrives;
      // the regular one-second GOP remains a fallback.
      if (packet.camera != sentry_preview_encoder(desired) || (video[c] && packet.frame_time <= preview_time)) return;
      switching = video[c] && preview_encoder != packet.camera;
      if (switching && !keyframe) return;
    }
    bool existing = video[c] != nullptr;
    if (!existing) {
      // A truncated RAM segment must still start with a decodable frame.
      if (!keyframe) return;
      video[c] = std::make_unique<VideoWriter>(path.c_str(), info.filename,
        idx.getType() != cereal::EncodeIndex::Type::FULL_H_E_V_C, data.getWidth(), data.getHeight(), info.fps, idx.getType());
      auto header = data.getHeader();
      video[c]->write(const_cast<uint8_t *>(header.begin()), header.size(), idx.getTimestampEof()/1000, true, false);
    }
    auto bytes = data.getData();
    std::vector<uint8_t> switched;
    if (existing && (switching || restarting)) {
      // Supply the new encoder's SPS/PPS in-band, without restarting the TS
      // muxer or overwriting the earlier part of this same minute.
      auto header = data.getHeader();
      switched.insert(switched.end(), header.begin(), header.end());
      switched.insert(switched.end(), bytes.begin(), bytes.end());
      bytes = kj::arrayPtr<const uint8_t>(switched.data(), switched.size());
    }
    video[c]->write(const_cast<uint8_t *>(bytes.begin()), bytes.size(), idx.getTimestampEof()/1000, false, keyframe);
    frame_time[c] = packet.frame_time;
    resume[c] = false;
    if (packet.camera >= 3) {
      // Derive availability from accepted preview frames, not the event timer
      // or minute boundary. Delay the end marker until the next actual gap or
      // finalization, since overlapping pre-roll can fill a paused interval.
      const uint64_t frame_duration = SECOND/info.fps;
      if (!preview_time || packet.frame_time-preview_time > frame_duration*3/2) {
        if (preview_time) timeline(preview_time+frame_duration, false);
        timeline(packet.frame_time, true);
      }
      preview_encoder = packet.camera;
      preview_time = packet.frame_time;
    }
    // Keep source frame/timestamps but rebase segment and in-file frame indexes.
    capnp::MallocMessageBuilder index_builder;
    index_builder.setRoot(idx);
    auto rebased = index_builder.getRoot<cereal::EncodeIndex>();
    rebased.setSegmentNum(number);
    rebased.setSegmentId(frame_count[c]++);
    rebased.setLen(bytes.size());
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
    if (preview_time) {
      uint64_t end = preview_time+SECOND/sentry_encoders[preview_encoder].fps;
      timeline(end, false);  // Stop orange here, including before wholly absent minutes.
      last_time = std::max(last_time, end);
    }
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
  std::array<int, SENTRY_ENCODER_COUNT> source_segments;
  std::array<uint64_t, SENTRY_ENCODER_COUNT> source_times = {};
  std::array<uint64_t, SENTRY_ENCODER_COUNT> source_frame_times = {};
  size_t buffered = 0;
  std::string route, error;
  kj::Array<capnp::word> init = logger_build_init_data(true);
  kj::Array<capnp::word> route_init;
  int closed_segment = -1;
  uint64_t retry_after = 0, storage_check = 0;
  uint64_t last_motion = 0;
  uint64_t last_record_until = 0;
  uint64_t route_start = 0, clip_start = 0;
  uint64_t last_preview_time = 0, preview_resume_time = 0;
  bool recording = false;
  int pre_seconds = history_seconds();
  int preview_camera = VISION_STREAM_NARROW_ROAD;

  Recorder() { source_segments.fill(-1); }
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
    recording = false;
    route_start = clip_start = 0;
    last_preview_time = preview_resume_time = 0;
    closed_segment = -1;
  }
  void reset() {
    finish();
    ring.clear();
    buffered = 0;
    source_segments.fill(-1);
    source_times.fill(0);
    source_frame_times.fill(0);
    last_motion = 0;
    last_record_until = 0;
  }
  int output_segment(uint64_t time) const {
    return time < route_start ? 0 : (time-route_start)/(SEGMENT_LENGTH*SECOND);
  }
  void save(int source_segment, Packet &packet) {
    if (packet.saved || packet.frame_time < clip_start) return;
    int segment = output_segment(packet.frame_time);
    if (segment <= closed_segment) return;
    auto &entry = ring.at(source_segment);
    if (entry.previews.empty()) entry.previews.push_back({0, preview_camera});
    auto &out = saved[segment];
    if (!out) out = std::make_unique<SavedSegment>(route, segment, route_init.asPtr(), route_start+segment*SEGMENT_LENGTH*SECOND);
    out->append(packet, entry.previews);
    last_preview_time = std::max(last_preview_time, out->preview_time);
    packet.saved = true;
  }
  void select_preview(uint64_t time, int camera) {
    preview_camera = camera;
    if (!recording || ring.empty()) return;
    auto &entry = ring.rbegin()->second;
    if (entry.previews.empty() || entry.previews.back().camera == camera) return;
    entry.previews.push_back({time, camera});
  }
  void append(int camera, cereal::Event::Reader event, uint64_t now) {
    int segment = ring.empty() ? 0 : ring.rbegin()->first;
    if (camera >= 0) {
      auto idx = (event.*sentry_encoders[camera].get_encode_data_func)().getIdx();
      segment = idx.getSegmentNum();
      if (source_segments[camera] > segment) reset();  // encoder restart: separate routes
      source_segments[camera] = segment;
      source_times[camera] = now;
      source_frame_times[camera] = idx.getTimestampEof();
    } else if (ring.empty()) {
      return;
    }
    auto &entry = ring[segment];
    Packet packet(camera, event);
    entry.end = std::max(entry.end, packet.time);
    entry.bytes += packet.bytes();
    buffered += packet.bytes();
    if (recording) save(segment, packet);
    entry.packets.emplace_back(std::move(packet));
    const uint64_t window = pre_seconds * SECOND;
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
      // Native minute boundaries organize the RAM ring only. Output routes
      // find their own first playable frame after trimming/eviction.
    }
  }
  void tick(uint64_t record_until, uint64_t now, uint64_t control_time, int trigger_camera) {
    pre_seconds = history_seconds();
    // Only confirmed motion or a diagnostic manual trigger advances the deadline.
    // The immediate eye signal must never create bookmarks or extend a route.
    uint64_t motion_time = record_until > now && record_until > last_record_until ? control_time : 0;
    last_record_until = record_until;
    // One standard route bookmark per motion burst, after five quiet seconds.
    // Write it directly to this route: the UI owns the live userBookmark topic.
    bool bookmark = motion_time && (!last_motion || motion_time >= last_motion+5*SECOND);
    if (motion_time) last_motion = motion_time;
    if (recording) {
      if (now >= record_until) {
        recording = false;
        preview_resume_time = last_preview_time+SECOND/MAIN_FPS;
        for (auto &[number, segment] : saved) segment->pause();
      } else if (now-storage_check > 5*SECOND) {
        storage_check = now;
        if (!disk_ok()) {
          // Keep the armed-session identity, but release files for cleanup.
          recording = false;
          preview_resume_time = last_preview_time+SECOND/MAIN_FPS;
          for (auto &[number, segment] : saved) {
            segment->close(false);
            closed_segment = number;
          }
          saved.clear();
          retry_after = now+30*SECOND;
        }
      }
      // Finalize historical segments once all recently seen camera streams advanced.
      int oldest = INT_MAX;
      for (int c = 0; c < SENTRY_ENCODER_COUNT; ++c) {
        if (source_times[c] && now-source_times[c] < 3*SECOND) oldest = std::min(oldest, output_segment(source_frame_times[c]));
      }
      while (recording && saved.size() > 1 && saved.begin()->first < oldest) {
        auto it = saved.begin();
        closed_segment = it->first;
        it->second->close(false);
        saved.erase(it);
      }
    }
    if (!recording && record_until > now && now >= retry_after && !ring.empty()) {
      uint64_t window = pre_seconds*SECOND;
      uint64_t cutoff = now > window ? now-window : 0;
      if (!route.empty()) cutoff = std::max(cutoff, route_start);
      if (closed_segment >= 0) cutoff = std::max(cutoff, route_start+(closed_segment+1)*SEGMENT_LENGTH*SECOND);
      const Packet *first_preview = nullptr;
      // Start exactly where the retained preview can decode, not at an older
      // source-minute boundary or an unavailable requested pre-roll time.
      for (const auto &[number, segment] : ring) {
        for (const auto &packet : segment.packets) {
          if (packet.frame_time < cutoff || !packet.keyframe || packet.camera < 3) continue;
          int camera = preview_at(segment.previews, packet.frame_time, trigger_camera);
          if (!route.empty() && packet.frame_time >= preview_resume_time) camera = trigger_camera;
          if (packet.camera == sentry_preview_encoder(camera) && (!first_preview || packet.frame_time < first_preview->frame_time)) {
            first_preview = &packet;
          }
        }
      }
      if (!first_preview) return;
      clip_start = first_preview->frame_time;
      if (!disk_ok()) { retry_after = now+30*SECOND; return; }
      storage_check = now;
      if (route.empty()) {
        route = logger_get_identifier("RouteCount");
        route_start = clip_start;
        // One identity and real-time origin for this entire armed session.
        // Quiet gaps have no files; later events retain their actual offsets.
        capnp::FlatArrayMessageReader init_reader(init.asPtr());
        MessageBuilder init_message;
        init_message.setRoot(init_reader.getRoot<cereal::Event>());
        auto init_event = init_message.getRoot<cereal::Event>();
        init_event.setLogMonoTime(route_start);
        init_event.getInitData().setWallTimeNanos(nanos_since_epoch()-(nanos_since_boot()-route_start));
        route_init = capnp::messageToFlatArray(init_message);
      }
      recording = true;
      preview_camera = trigger_camera;
      for (auto it = ring.begin(); it != ring.end(); ++it) {
        // Historical preview choices stay intact when event windows overlap.
        // Sort buffered packets by source time so a multi-lens preview is
        // replayed in order even if the encoder queues drained unevenly.
        if (it->second.previews.empty()) it->second.previews.push_back({0, trigger_camera});
        else if (preview_resume_time && it->second.previews.back().camera != trigger_camera) {
          it->second.previews.push_back({std::max(clip_start, preview_resume_time), trigger_camera});
        }
        std::vector<Packet *> packets;
        for (auto &packet : it->second.packets) packets.push_back(&packet);
        std::stable_sort(packets.begin(), packets.end(), [](auto a, auto b) { return a->frame_time < b->frame_time; });
        for (auto *packet : packets) save(it->first, *packet);
      }
      select_preview(control_time, trigger_camera);
      bookmark = true;  // Also mark diagnostic triggers and each resumed event.
    }
    if (bookmark && recording) {
      MessageBuilder message;
      auto event = message.initEvent(true);
      event.setLogMonoTime(motion_time ? motion_time : now);
      event.initUserBookmark();
      append(-1, event.asReader(), now);
    }
    if (!recording && !saved.empty()) {
      // A later trigger may reach backwards into this same output minute.
      // Wait through the maximum selectable pre-roll before unlocking it;
      // changing the setting cannot make us reopen an uploaded file.
      uint64_t immutable_before = now > 60*SECOND ? now-60*SECOND : 0;
      while (!saved.empty() && route_start+(saved.begin()->first+1)*SEGMENT_LENGTH*SECOND <= immutable_before) {
        auto it = saved.begin();
        closed_segment = it->first;
        it->second->close(false);
        saved.erase(it);
      }
    }
  }
  double buffer_seconds(uint64_t now) const {
    uint64_t oldest = now;
    for (const auto &[number, segment] : ring) {
      for (const auto &packet : segment.packets) {
        if (packet.camera >= 0) oldest = std::min(oldest, packet.frame_time);
      }
    }
    return std::min(double(pre_seconds), (now-oldest)/double(SECOND));
  }
};

int main() {
  fs::create_directories(Path::log_root());
  PubMaster pm({"sentryRecorderState"});
  SubMaster sm({"sentryState", "pandaStates", "deviceState", "clocks", "narrowRoadCameraState", "wideRoadCameraState", "cabinCameraState"});
  std::unique_ptr<Context> context(Context::create());
  std::array<std::unique_ptr<SubSocket>, SENTRY_ENCODER_COUNT> sockets;
  for (int c = 0; c < SENTRY_ENCODER_COUNT; ++c) {
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
    if (capture) recorder.tick(state.getRecordUntilMonoTime(), now, sm["sentryState"].getLogMonoTime(), state.getTriggerCamera());
    if (capture && state.getRecordUntilMonoTime() > now) {
      recorder.select_preview(sm["sentryState"].getLogMonoTime(), state.getTriggerCamera());
    }
    for (int c = 0; c < SENTRY_ENCODER_COUNT; ++c) {
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
    }
    if (now-flushed >= SECOND) {
      for (auto &[number, segment] : recorder.saved) segment->flush();
      flushed = now;
    }
    if (now-published > SECOND/2) {
      MessageBuilder message;
      auto status = message.initEvent().initSentryRecorderState();
      status.setRecording(recorder.recording);
      status.setRoute(recorder.route);
      status.setBufferBytes(recorder.buffered);
      status.setBufferSeconds(recorder.buffer_seconds(now));
      status.setError(recorder.error);
      status.setPreviewCamera(recorder.preview_camera);
      pm.send("sentryRecorderState", message);
      published = now;
    }
    util::sleep_for(10);
  }
  return 0;
}
