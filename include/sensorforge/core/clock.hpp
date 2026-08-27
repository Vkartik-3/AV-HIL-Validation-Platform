/*
==============================================================================
SensorForge - Clock sources
Part of the SensorForge AV HIL validation platform.

The audit found a live defect: frame timestamps were stamped from
std::chrono::system_clock (wall clock) while the receiver REJECTED any frame
whose timestamp went backwards. A single NTP step backwards therefore wedged
the link permanently, with no recovery path.

This header separates the two things that were conflated:

  wall_now_ns()      std::chrono::system_clock. Human/event correlation only.
                     May jump forwards or backwards at any time.
  mono_now_ns()      std::chrono::steady_clock. Never goes backwards. Used for
                     latency measurement and for monotonicity enforcement.

MonotonicWallClock bridges them: it emits a wall-clock value for the wire that
is guaranteed never to regress for a given stream. If the OS wall clock steps
backwards, the emitted value holds at last+1 until real time catches up. The
value therefore stays meaningful for human correlation (it tracks the wall
clock in steady state) while satisfying the receiver's monotonicity check by
construction, rather than by hoping the clock behaves.

No ROS2 dependency.
==============================================================================
*/

#pragma once

#include <chrono>
#include <cstdint>

namespace sensorforge::core {

/// Wall-clock nanoseconds since the Unix epoch. NOT monotonic.
inline uint64_t wall_now_ns()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
}

/// Monotonic nanoseconds from an unspecified epoch. Never regresses.
inline uint64_t mono_now_ns()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

/**
 * @brief Emits wall-clock timestamps that never regress.
 *
 * Single-threaded use per instance (one per stream, owned by that stream's
 * producer). Counts how often the underlying wall clock regressed so the
 * condition is observable instead of silent.
 */
class MonotonicWallClock
{
public:
  uint64_t next()
  {
    const uint64_t w = wall_now_ns();
    if (w > last_) {
      last_ = w;
      return w;
    }
    // Wall clock stalled or stepped backwards. Emit a strictly increasing
    // value so downstream monotonicity checks hold, and record the event.
    if (w < last_) {
      ++regressions_;
    }
    return ++last_;
  }

  uint64_t last() const {return last_;}
  uint64_t regressions() const {return regressions_;}

private:
  uint64_t last_ = 0;
  uint64_t regressions_ = 0;
};

}  // namespace sensorforge::core
