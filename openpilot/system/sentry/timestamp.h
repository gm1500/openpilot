#pragma once

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>

inline std::string sentry_timestamp(time_t epoch, bool local, int utc_offset) {
  if (!local || utc_offset < -24 * 3600 || utc_offset > 24 * 3600) {
    local = false;
    utc_offset = 0;
  }
  time_t shifted = epoch + utc_offset;
  tm civil = {};
  gmtime_r(&shifted, &civil);
  char stamp[32];
  strftime(stamp, sizeof(stamp), "%Y-%m-%d %H:%M:%S", &civil);
  if (!local) return std::string(stamp) + " UTC";
  char offset[16];
  std::snprintf(offset, sizeof(offset), " %c%02d%02d", utc_offset < 0 ? '-' : '+',
                std::abs(utc_offset) / 3600, std::abs(utc_offset) % 3600 / 60);
  return std::string(stamp) + offset;
}
