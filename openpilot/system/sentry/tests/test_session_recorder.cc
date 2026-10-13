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
  uint64_t deadline = 0;
  int camera = VISION_STREAM_WIDE_ROAD;
  auto feed = [&](int first, int last, const std::map<int, int> &triggers) {
    for (int frame = first*MAIN_FPS; frame < last*MAIN_FPS; ++frame) {
      uint64_t now = base+frame*(SECOND/MAIN_FPS);
      if (auto found = triggers.find(frame); found != triggers.end()) {
        deadline = now+2*SECOND;
        camera = found->second;
      }
      recorder.tick(deadline, now, now, camera);
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

  // Minutes 1 and 2 contain no events or files. The next event starts partway
  // through minute 3, keeping the real timestamps and exactly the same initData.
  feed(180, 200, {{3940, 2}});
  assert(!recorder.recording && recorder.route == route);
  assert(recorder.saved.size() == 1 && recorder.saved.count(3));
  recorder.tick(deadline, base+300*SECOND, base+300*SECOND, camera);
  assert(recorder.saved.empty() && recorder.route == route);
  assert(!fs::exists(fs::path(Path::log_root())/(route+"--1")));
  assert(!fs::exists(fs::path(Path::log_root())/(route+"--2")));
  for (const auto &[name, contents] : immutable) assert(util::read_file(first/name) == contents);

  // Disarming ends the grouping, even when no event file remains open.
  recorder.reset();
  assert(recorder.route.empty() && recorder.buffered == 0);
  feed(310, 323, {{6400, 1}});
  std::string next_route = recorder.route;
  assert(next_route != route && !next_route.empty());
  recorder.reset();
  std::ofstream{fixtures/"routes.txt"} << route << '\n' << next_route << '\n' << base << '\n';
}
