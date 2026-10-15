// Exercise the production recorder with real encoded packets and a deterministic
// clock, including multi-minute gaps without making the test sleep for minutes.
#define main sentry_service_main
#include "system/loggerd/sentry_loggerd.cc"
#undef main

int main(int argc, char **argv) {
  assert(argc == 2);
  fs::path fixtures(argv[1]);
  fs::create_directories(Path::log_root());
  Params().put("SentryPreSeconds", "15");
  std::array<std::vector<Packet>, SENTRY_ENCODER_COUNT> inputs;
  for (int c = 0; c < SENTRY_ENCODER_COUNT; ++c) {
    std::string data = util::read_file((fixtures/(std::to_string(c)+".capnp")).string());
    AlignedBuffer aligned;
    auto words = aligned.align(data.data(), data.size()).slice(0, data.size()/sizeof(capnp::word));
    while (words.size()) {
      capnp::FlatArrayMessageReader reader(words);
      inputs[c].emplace_back(c, reader.getRoot<cereal::Event>());
      words = words.slice(reader.getEnd()-words.begin(), words.size());
    }
    assert(inputs[c].size() == 40);
  }
  Recorder recorder;
  uint64_t base = nanos_since_boot()-600*SECOND;
  {
    MessageBuilder message;
    auto state = message.initEvent(true).initSentryState();
    assert(trigger_mode(state.asReader()) == Timeline::FOOTAGE);
    state.setAccelActive(true);
    assert(trigger_mode(state.asReader()) == Timeline::FOOTAGE);
    state.setAudioActive(true);
    assert(trigger_mode(state.asReader()) == Timeline::AUDIO);
    state.setGyroActive(true);
    assert(trigger_mode(state.asReader()) == Timeline::GYRO);  // Gyro wins simultaneous activity.
    state.setAudioActive(false);
    assert(trigger_mode(state.asReader()) == Timeline::GYRO);
  }
  uint64_t deadline = 0;
  int camera = VISION_STREAM_WIDE_ROAD;
  auto feed = [&](int first, int last, const std::map<int, int> &triggers, const std::map<int, Timeline> &modes = {}) {
    for (int frame = first*MAIN_FPS; frame < last*MAIN_FPS; ++frame) {
      uint64_t now = base+frame*(SECOND/MAIN_FPS);
      if (auto found = triggers.find(frame); found != triggers.end()) {
        deadline = now+2*SECOND;
        camera = found->second;
      }
      // Activity flags clear between hits; colour lasts for the confirmed tail.
      auto mode = modes.find(frame);
      recorder.tick(deadline, now, now, camera, mode != modes.end() ? mode->second : Timeline::FOOTAGE);
      if (deadline > now) recorder.select_preview(now, camera);
      for (int c = 0; c < SENTRY_ENCODER_COUNT; ++c) {
        capnp::FlatArrayMessageReader reader(inputs[c][frame%40].words.asPtr());
        MessageBuilder message;
        message.setRoot(reader.getRoot<cereal::Event>());
        auto event = message.getRoot<cereal::Event>();
        event.setLogMonoTime(now);
        // Builders do not share the generated Reader member-function type.
        auto data = c < 3 ? (c == 0 ? event.getNarrowRoadEncodeData() :
                            c == 1 ? event.getWideRoadEncodeData() : event.getCabinEncodeData()) : event.getQNarrowRoadEncodeData();
        auto idx = data.getIdx();
        idx.setTimestampSof(now); idx.setTimestampEof(now);
        idx.setFrameId(frame+10000*(c%3));
        idx.setSegmentNum(7+frame/(60*MAIN_FPS));
        idx.setSegmentId(frame%(60*MAIN_FPS));
        recorder.append(c, event.asReader(), now);
      }
    }
  };
  feed(0, 10, {});
  assert(recorder.route.empty() && fs::is_empty(Path::log_root()));
  feed(10, 48, {{200, 2}, {400, 2}, {900, 1}});
  const std::string route = recorder.route;
  assert(!route.empty() && !recorder.recording);
  assert(recorder.saved.size() == 1 && recorder.saved.count(0));
  assert(fs::exists(recorder.saved.at(0)->path+"/sentry.lock"));
  recorder.tick(deadline, base+120*SECOND-1, base+120*SECOND-1, camera);
  assert(recorder.saved.size() == 1);
  recorder.tick(deadline, base+120*SECOND, base+120*SECOND, camera);
  assert(recorder.saved.empty() && recorder.route == route);
  fs::path first = fs::path(Path::log_root())/(route+"--0");
  assert(!fs::exists(first/"sentry.lock"));
  std::map<std::string, std::string> immutable;
  for (const auto &entry : fs::directory_iterator(first)) immutable[entry.path().filename()] = util::read_file(entry.path());

  // There are no gap files until a later event supplies the far side. Minute 2
  // must still wait until no future 60s pre-roll can add actual footage to it.
  assert(!fs::exists(fs::path(Path::log_root())/(route+"--1")));
  assert(!fs::exists(fs::path(Path::log_root())/(route+"--2")));
  feed(180, 200, {{3940, 2}});
  assert(!recorder.recording && recorder.route == route);
  assert(recorder.saved.size() == 1 && recorder.saved.count(3));
  assert(fs::exists(fs::path(Path::log_root())/(route+"--1")/"qlog.zst"));
  assert(!fs::exists(fs::path(Path::log_root())/(route+"--2")));
  recorder.tick(deadline, base+300*SECOND, base+300*SECOND, camera);
  assert(recorder.saved.empty() && recorder.route == route);
  assert(fs::exists(fs::path(Path::log_root())/(route+"--2")/"qlog.zst"));
  for (const auto &[name, contents] : immutable) assert(util::read_file(first/name) == contents);

  // Disarming ends the grouping, even when no event file remains open.
  recorder.reset();
  assert(recorder.route.empty() && recorder.buffered == 0);
  // Overlapping events also cover a complete minute without a blue seam.
  feed(310, 383, {{6400, 1}, {6600, 1}, {6800, 1}, {7000, 1}, {7200, 1}, {7400, 1}, {7600, 1}});
  std::string next_route = recorder.route;
  assert(next_route != route && !next_route.empty());
  recorder.reset();

  // A setting change can fill a pending gap with real retained frames. It must
  // never overwrite a published minimal log or leave a lock on the gap queue.
  feed(400, 413, {{8200, 2}});
  const std::string filled_route = recorder.route;
  feed(480, 540, {{10740, 2}});
  fs::path filled = fs::path(Path::log_root())/(filled_route+"--1");
  assert(!fs::exists(filled));
  assert(!recorder.gaps.empty());
  Params().put("SentryPreSeconds", "60");
  feed(540, 548, {{10900, 2}});
  assert(fs::exists(filled/"qcamera.ts"));
  assert(recorder.gaps.empty());
  recorder.reset();
  Params().put("SentryPreSeconds", "15");

  // Mixed triggers within one event, resumed pre-roll, and a minute boundary.
  // Later audio must not recolour earlier gyro footage when pre-roll overlaps.
  assert(recorder.triggers.empty());
  feed(600, 673, {{12200, 2}, {12220, 2}, {12400, 2}, {13180, 2}, {13200, 2}, {13400, 2}},
                 {{12200, Timeline::AUDIO}, {12220, Timeline::GYRO}, {12400, Timeline::AUDIO},
                  {13180, Timeline::GYRO}, {13200, Timeline::AUDIO}, {13400, Timeline::AUDIO}});
  const std::string colour_route = recorder.route;
  recorder.reset();
  assert(recorder.triggers.empty());
  std::ofstream{fixtures/"colour_route.txt"} << colour_route << '\n';

  // Ten quiet hours must not generate files on their own. Once motion resumes,
  // drain incrementally; disarming also completes any remaining bounded ranges.
  feed(1000, 1013, {{20200, 2}});
  const std::string long_route = recorder.route;
  recorder.tick(deadline, base+36900*SECOND, base+36900*SECOND, camera);
  assert(recorder.gaps.empty() && recorder.saved.empty());
  assert(!fs::exists(fs::path(Path::log_root())/(long_route+"--1")));
  feed(37000, 37020, {{740340, 2}});
  assert(recorder.route == long_route && !recorder.gaps.empty());
  auto before_finish = std::chrono::steady_clock::now();
  recorder.reset();
  double finish_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-before_finish).count();
  std::ofstream{fixtures/"gap_finish_ms.txt"} << finish_ms << '\n';
  std::ofstream{fixtures/"routes.txt"} << route << '\n' << next_route << '\n' << filled_route << '\n' << long_route << '\n' << base << '\n';
}
