/*
==============================================================================
SensorForge - Process resource monitoring (implementation)
Platform-specific code is confined to this file.
==============================================================================
*/

#include "sensorforge/core/resource_monitor.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#if defined(__linux__)
#include <unistd.h>
#elif defined(__APPLE__)
#include <mach/mach.h>
#endif

namespace sensorforge::core {

namespace {
uint64_t wall_ns()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}
}  // namespace

const char * to_string(BudgetState s)
{
  switch (s) {
    case BudgetState::kOk: return "ok";
    case BudgetState::kSoftBreach: return "soft_breach";
    case BudgetState::kHardBreach: return "hard_breach";
  }
  return "unknown";
}

bool ResourceSampler::platform_supported()
{
#if defined(__linux__) || defined(__APPLE__)
  return true;
#else
  return false;
#endif
}

const char * ResourceSampler::platform_name()
{
#if defined(__linux__)
  return "linux/proc";
#elif defined(__APPLE__)
  return "macos/mach";
#else
  return "unsupported";
#endif
}

ResourceSampler::ResourceSampler() = default;

ResourceSample ResourceSampler::sample()
{
  ResourceSample s;
  s.uptime_ns = wall_ns();

#if defined(__linux__)
  // RSS from /proc/self/statm (field 2 = resident pages).
  if (std::FILE * f = std::fopen("/proc/self/statm", "r")) {
    unsigned long total_pages = 0, resident_pages = 0;
    if (std::fscanf(f, "%lu %lu", &total_pages, &resident_pages) == 2) {
      const long page = ::sysconf(_SC_PAGESIZE);
      s.rss_bytes = static_cast<uint64_t>(resident_pages) * static_cast<uint64_t>(page);
      s.available = true;
    }
    std::fclose(f);
  }
  // CPU from /proc/self/stat fields 14 (utime) and 15 (stime), in clock ticks.
  if (std::FILE * f = std::fopen("/proc/self/stat", "r")) {
    char buf[4096];
    const size_t n = std::fread(buf, 1, sizeof(buf) - 1, f);
    std::fclose(f);
    buf[n] = '\0';
    // Skip past comm, which may contain spaces, by finding the last ')'.
    char * p = buf;
    char * close_paren = nullptr;
    for (char * q = buf; *q; ++q) {
      if (*q == ')') {close_paren = q;}
    }
    if (close_paren) {
      p = close_paren + 1;
      unsigned long utime = 0, stime = 0;
      int field = 3;   // next field after comm is state (field 3)
      char * tok = std::strtok(p, " ");
      while (tok) {
        if (field == 14) {utime = std::strtoul(tok, nullptr, 10);}
        if (field == 15) {stime = std::strtoul(tok, nullptr, 10); break;}
        tok = std::strtok(nullptr, " ");
        ++field;
      }
      const long hz = ::sysconf(_SC_CLK_TCK);
      if (hz > 0) {
        s.cpu_time_ns =
          (static_cast<uint64_t>(utime + stime) * 1000000000ull) / static_cast<uint64_t>(hz);
      }
    }
  }
#elif defined(__APPLE__)
  // macOS: mach task_info. Present so the core is testable off Linux; the
  // production target remains Linux.
  mach_task_basic_info_data_t info{};
  mach_msg_type_number_t count = MACH_TASK_BASIC_INFO_COUNT;
  if (task_info(
      mach_task_self(), MACH_TASK_BASIC_INFO,
      reinterpret_cast<task_info_t>(&info), &count) == KERN_SUCCESS)
  {
    s.rss_bytes = info.resident_size;
    s.available = true;
  }
  task_thread_times_info_data_t times{};
  mach_msg_type_number_t tcount = TASK_THREAD_TIMES_INFO_COUNT;
  if (task_info(
      mach_task_self(), TASK_THREAD_TIMES_INFO,
      reinterpret_cast<task_info_t>(&times), &tcount) == KERN_SUCCESS)
  {
    s.cpu_time_ns =
      (static_cast<uint64_t>(times.user_time.seconds + times.system_time.seconds) * 1000000000ull) +
      (static_cast<uint64_t>(times.user_time.microseconds +
      times.system_time.microseconds) * 1000ull);
  }
#endif

  if (primed_ && s.uptime_ns > last_wall_ns_) {
    const double dt = static_cast<double>(s.uptime_ns - last_wall_ns_);
    const double dcpu = s.cpu_time_ns >= last_cpu_ns_
      ? static_cast<double>(s.cpu_time_ns - last_cpu_ns_) : 0.0;
    s.cpu_percent = dt > 0 ? 100.0 * dcpu / dt : 0.0;
  }
  last_cpu_ns_ = s.cpu_time_ns;
  last_wall_ns_ = s.uptime_ns;
  primed_ = true;
  return s;
}

BudgetState evaluate_budget(
  const ResourceBudget & b, uint64_t rss_bytes, uint64_t queue_bytes)
{
  if ((b.hard_rss_bytes && rss_bytes >= b.hard_rss_bytes) ||
    (b.hard_queue_bytes && queue_bytes >= b.hard_queue_bytes))
  {
    return BudgetState::kHardBreach;
  }
  if ((b.soft_rss_bytes && rss_bytes >= b.soft_rss_bytes) ||
    (b.soft_queue_bytes && queue_bytes >= b.soft_queue_bytes))
  {
    return BudgetState::kSoftBreach;
  }
  return BudgetState::kOk;
}

}  // namespace sensorforge::core
