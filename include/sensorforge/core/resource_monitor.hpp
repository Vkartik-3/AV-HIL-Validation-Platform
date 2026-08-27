/*
==============================================================================
SensorForge - Process resource monitoring
Part of the SensorForge AV HIL validation platform.

The audit found no memory ceiling, no CPU accounting and no resource
monitoring of any kind, which made every buffer bound in the system
unfalsifiable in operation.

Linux-specific /proc parsing sits behind ResourceSampler so the ROS-free core
stays portable: on Linux the /proc implementation is used, elsewhere a portable
implementation reports what the platform can supply (macOS via task_info) or
zeroes with available=false. Nothing else in the core includes <linux/...>.
==============================================================================
*/

#pragma once

#include <cstdint>
#include <string>

namespace sensorforge::core {

struct ResourceSample
{
  bool available = false;
  uint64_t rss_bytes = 0;
  double cpu_percent = 0.0;      // process CPU since the previous sample
  uint64_t cpu_time_ns = 0;      // cumulative user+sys
  uint64_t uptime_ns = 0;
};

/// Soft/hard budgets. Hard breach triggers the recorder's shedding behaviour.
struct ResourceBudget
{
  uint64_t soft_rss_bytes = 0;    // 0 = unset
  uint64_t hard_rss_bytes = 0;
  uint64_t soft_queue_bytes = 0;
  uint64_t hard_queue_bytes = 0;

  bool any() const
  {
    return soft_rss_bytes || hard_rss_bytes || soft_queue_bytes || hard_queue_bytes;
  }
};

enum class BudgetState { kOk, kSoftBreach, kHardBreach };

const char * to_string(BudgetState s);

/**
 * @brief Samples process RSS and CPU. Cheap enough for a 1 Hz timer; NOT for
 *        the per-message path.
 */
class ResourceSampler
{
public:
  ResourceSampler();

  /// Take a sample. cpu_percent is computed against the previous call.
  ResourceSample sample();

  /// True when the platform can actually supply these numbers.
  static bool platform_supported();
  static const char * platform_name();

private:
  uint64_t last_cpu_ns_ = 0;
  uint64_t last_wall_ns_ = 0;
  bool primed_ = false;
};

/// Evaluate a budget against the current RSS and total queued bytes.
BudgetState evaluate_budget(
  const ResourceBudget & budget, uint64_t rss_bytes, uint64_t queue_bytes);

}  // namespace sensorforge::core
