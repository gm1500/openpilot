#include <cassert>
#include <capnp/message.h>
#include "system/sentry/state.h"

int main() {
  constexpr uint64_t now = 10'000'000'000ULL;

  // Reproduce the boot condition: SubMaster has not received a sentry message.
  cereal::Event::Reader empty;
  assert(!sentry_state(empty, 0, now));

  capnp::MallocMessageBuilder message;
  auto event = message.initRoot<cereal::Event>();
  event.setValid(true);
  event.initDeviceState();
  assert(!sentry_state(event.asReader(), now, now));

  auto state = event.initSentryState();
  state.setCapture(true);
  state.setCameraMask(7);
  auto active = sentry_state(event.asReader(), now, now);
  assert(active && active->getCapture() && active->getCameraMask() == 7);
  assert(!sentry_state(event.asReader(), 0, now));
  assert(!sentry_state(event.asReader(), now+1, now));
  assert(!sentry_state(event.asReader(), now-2'000'000'000ULL, now));
  assert(sentry_state(event.asReader(), now-1'999'999'999ULL, now));

  event.setValid(false);
  assert(!sentry_state(event.asReader(), now, now));
  event.setValid(true);
  state.setCapture(false);
  assert(!sentry_state(event.asReader(), now, now)->getCapture());
}
