/*
==============================================================================
SensorForge - Per-sensor backpressure policies
Part of the SensorForge AV HIL validation platform.

When a sensor stream's buffer fills faster than the consumer drains it, the
policy decides what to sacrifice. The choice is sensor-specific:

  CAMERA : overwrite_oldest   - freshness matters more than history; drop the
                                stale frame and keep the newest.
  LIDAR  : drop_newest        - keep the in-flight sweep already queued; drop
                                the arriving frame rather than corrupt ordering.
  IMU    : drop_newest        - high rate, small payloads; a dropped sample is
                                tolerable and ordering is preserved.
  GPS    : drop_newest        - low rate; a dropped fix is tolerable.
  CAN    : block              - safety-critical bus; wait (bounded) for space.

REMOVED: kBatchAccumulate. The audit found it shared a case label with
kDropNewest and did nothing that its name or documentation described -- no
coalescing, no batch drain, no difference in behaviour whatsoever. Rather than
keep a policy name that misrepresents the code, it is deleted and IMU now
declares the drop-newest behaviour it always actually had. If batching is
wanted later it must be implemented, not named.

kNeverDropBlock is now BOUNDED. The previous implementation spun forever, which
meant a stalled consumer could hang a ROS executor thread permanently. It now
waits up to a caller-supplied budget and then reports failure, so the caller
can shed rather than deadlock.
==============================================================================
*/

#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

#include "sensorforge/core/spsc_ring.hpp"
#include "sensorforge/protocol/frame.hpp"

namespace sensorforge::core {

enum class BackpressurePolicy {
  kOverwriteOldest,   // drop oldest, enqueue newest (camera)
  kDropNewest,        // drop the arriving item (lidar, imu, gps)
  kNeverDropBlock,    // wait (bounded) for space, then fail (can)
};

inline constexpr const char * to_string(BackpressurePolicy p)
{
  switch (p) {
    case BackpressurePolicy::kOverwriteOldest: return "overwrite_oldest";
    case BackpressurePolicy::kDropNewest: return "drop_newest";
    case BackpressurePolicy::kNeverDropBlock: return "block";
  }
  return "unknown";
}

inline BackpressurePolicy policy_from_string(const std::string & s)
{
  if (s == "overwrite_oldest" || s == "overwrite") {return BackpressurePolicy::kOverwriteOldest;}
  if (s == "block" || s == "never_drop") {return BackpressurePolicy::kNeverDropBlock;}
  return BackpressurePolicy::kDropNewest;
}

/// Default policy for a given sensor type.
inline constexpr BackpressurePolicy default_policy_for(protocol::SensorType t)
{
  using protocol::SensorType;
  switch (t) {
    case SensorType::kCamera: return BackpressurePolicy::kOverwriteOldest;
    case SensorType::kCan: return BackpressurePolicy::kNeverDropBlock;
    case SensorType::kLidar:
    case SensorType::kImu:
    case SensorType::kGps:
    default: return BackpressurePolicy::kDropNewest;
  }
}

/// Outcome of applying a policy to one push attempt.
enum class PushResult {
  kEnqueued,             // item made it into the buffer
  kDroppedNewest,        // full, arriving item dropped
  kOverwrote,            // full, oldest dropped, newest enqueued
  kBlockedThenEnqueued,  // producer waited for space, then enqueued
  kDroppedAfterBlock,    // producer waited out its budget and still had no room
};

inline constexpr const char * to_string(PushResult r)
{
  switch (r) {
    case PushResult::kEnqueued: return "enqueued";
    case PushResult::kDroppedNewest: return "dropped_newest";
    case PushResult::kOverwrote: return "overwrote";
    case PushResult::kBlockedThenEnqueued: return "blocked_then_enqueued";
    case PushResult::kDroppedAfterBlock: return "dropped_after_block";
  }
  return "unknown";
}

/**
 * @brief Apply @p policy while pushing @p item into @p ring.
 *
 * Producer-thread only. @p evicted is incremented once per element actually
 * dropped by an overwrite. @p block_budget bounds kNeverDropBlock.
 */
template<typename T, size_t Capacity>
PushResult apply_policy(
  SPSCRing<T, Capacity> & ring, const T & item, BackpressurePolicy policy,
  uint64_t & evicted,
  std::chrono::microseconds block_budget = std::chrono::microseconds(2000))
{
  switch (policy) {
    case BackpressurePolicy::kOverwriteOldest: {
      if (ring.try_push(item)) {
        return PushResult::kEnqueued;
      }
      const uint64_t before = evicted;
      if (ring.push_overwrite(item, evicted)) {
        return evicted > before ? PushResult::kOverwrote : PushResult::kEnqueued;
      }
      return PushResult::kDroppedNewest;
    }

    case BackpressurePolicy::kDropNewest:
      return ring.try_push(item) ? PushResult::kEnqueued : PushResult::kDroppedNewest;

    case BackpressurePolicy::kNeverDropBlock: {
      if (ring.try_push(item)) {
        return PushResult::kEnqueued;
      }
      // Bounded wait. Never spins forever: a permanently stalled consumer
      // makes this fail rather than hang the producer's thread.
      const auto deadline = std::chrono::steady_clock::now() + block_budget;
      while (std::chrono::steady_clock::now() < deadline) {
        std::this_thread::yield();
        if (ring.try_push(item)) {
          return PushResult::kBlockedThenEnqueued;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(50));
      }
      return PushResult::kDroppedAfterBlock;
    }
  }
  return PushResult::kDroppedNewest;
}

}  // namespace sensorforge::core
