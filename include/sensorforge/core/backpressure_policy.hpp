/*
==============================================================================
SensorForge - Per-sensor backpressure policies
Part of the SensorForge AV HIL validation platform.

When a sensor stream's SPSC ring fills up faster than the consumer drains it,
the policy decides what to sacrifice. The choice is sensor-specific because
the semantics differ:

  CAMERA : overwrite_oldest   - freshness matters more than history; drop the
                                stale frame and keep the newest.
  LIDAR  : drop_newest        - keep the in-flight sweep already queued; drop
                                the arriving frame rather than corrupt ordering.
  IMU    : batch_accumulate   - high rate, small payloads; coalesce/keep and
                                let the consumer batch-drain.
  GPS    : drop_newest        - low rate; a dropped fix is tolerable, ordering
                                preserved.
  CAN    : never_drop         - safety-critical bus; block the producer until
                                space is available (no silent loss ever).

This header defines the policy enum, a per-sensor-type default mapping, and a
single apply() entry point that implements each policy against an SPSCRing.
==============================================================================
*/

#pragma once

#include <chrono>
#include <thread>

#include "sensorforge/core/spsc_ring.hpp"
#include "sensorforge/protocol/frame.hpp"

namespace sensorforge::core {

enum class BackpressurePolicy {
  kOverwriteOldest,   // drop oldest, enqueue newest (camera)
  kDropNewest,        // drop the arriving item (lidar, gps)
  kBatchAccumulate,   // like drop-newest at the ring edge, but the consumer is
                      // expected to batch-drain; producer never blocks (imu)
  kNeverDropBlock,    // block the producer until space frees up (can)
};

/// Default policy for a given sensor type.
inline constexpr BackpressurePolicy default_policy_for(protocol::SensorType t)
{
  using protocol::SensorType;
  switch (t) {
    case SensorType::kCamera: return BackpressurePolicy::kOverwriteOldest;
    case SensorType::kLidar: return BackpressurePolicy::kDropNewest;
    case SensorType::kImu: return BackpressurePolicy::kBatchAccumulate;
    case SensorType::kGps: return BackpressurePolicy::kDropNewest;
    case SensorType::kCan: return BackpressurePolicy::kNeverDropBlock;
    default: return BackpressurePolicy::kDropNewest;
  }
}

/// Outcome of applying a policy to one push attempt.
enum class PushResult {
  kEnqueued,       // item made it into the ring
  kDroppedNewest,  // ring full, arriving item dropped
  kOverwrote,      // ring full, oldest dropped, newest enqueued
  kBlockedThenEnqueued,  // producer waited for space, then enqueued
};

/**
 * @brief Apply @p policy while pushing @p item into @p ring.
 *
 * Producer-thread only (SPSC contract). For kNeverDropBlock the call spins
 * with a short yield/sleep until space is available -- callers on the CAN
 * path accept the backpressure by design.
 */
template<typename T, size_t Capacity>
PushResult apply_policy(
  SPSCRing<T, Capacity> & ring, const T & item, BackpressurePolicy policy)
{
  switch (policy) {
    case BackpressurePolicy::kOverwriteOldest:
      if (ring.try_push(item)) {
        return PushResult::kEnqueued;
      }
      ring.force_push_overwrite(item);
      return PushResult::kOverwrote;

    case BackpressurePolicy::kDropNewest:
    case BackpressurePolicy::kBatchAccumulate:
      return ring.try_push(item) ? PushResult::kEnqueued : PushResult::kDroppedNewest;

    case BackpressurePolicy::kNeverDropBlock:
      if (ring.try_push(item)) {
        return PushResult::kEnqueued;
      }
      // Block until the consumer frees a slot. Bounded backoff to avoid a hot
      // spin pinning a core.
      for (;;) {
        std::this_thread::yield();
        if (ring.try_push(item)) {
          return PushResult::kBlockedThenEnqueued;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(50));
      }
  }
  return PushResult::kDroppedNewest;  // unreachable
}

}  // namespace sensorforge::core
