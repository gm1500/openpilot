#pragma once

#include <cstdint>
#include <optional>

#include "openpilot/cereal/gen/cpp/log.capnp.h"

// C++ SubMaster starts with an empty Event, not an initialized service union.
// Check the union before accessing it, including when this subscriber starts
// after the publisher. Stale/invalid sentry messages must not keep IR/capture on.
inline std::optional<cereal::SentryState::Reader> sentry_state(cereal::Event::Reader event, uint64_t received_at, uint64_t now) {
  if (!event.isSentryState() || !event.getValid() || received_at == 0 || received_at > now || now-received_at >= 2'000'000'000ULL) {
    return std::nullopt;
  }
  return event.getSentryState();
}
