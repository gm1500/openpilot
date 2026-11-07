#include <string>
#include <vector>
#include <sys/resource.h>

#include "openpilot/cereal/messaging/messaging.h"

int main(int argc, char **argv) {
  // Unknown-service cases deliberately abort; do not leave core dumps behind.
  const rlimit limit = {0, 0};
  setrlimit(RLIMIT_CORE, &limit);
  if (argc < 3) return 2;
  const std::vector<const char *> names(argv + 2, argv + argc);
  if (std::string(argv[1]) == "sub") {
    SubMaster sm(names);
    sm.update(0);
  } else if (std::string(argv[1]) == "pub") {
    PubMaster pm(names);
  } else {
    return 2;
  }
  return 0;
}
