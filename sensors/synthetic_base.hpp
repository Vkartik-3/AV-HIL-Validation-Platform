/*
==============================================================================
SensorForge - Synthetic sensor publisher base (Extension I)
Part of the SensorForge AV HIL validation platform.

Common scaffolding for the synthetic AV sensor publishers: a ROS2 node that
declares the shared parameters (rate_hz, seed, frame_id, topic), owns a
deterministic RNG seeded by `seed`, drives a wall-timer at `rate_hz`, and calls
on_tick(seq, stamp) for the subclass to build and publish its message.

Determinism: the same `seed` produces the same data sequence, so scenarios are
reproducible. Timestamps are wall-clock (this->now()). The per-message sequence
number is tracked in seq_ and used to drive deterministic content; the
authoritative on-wire sequence for gap detection is stamped by the SensorForge
frame protocol (Extension B) at the transport layer.

Sensor publishers use best-effort SensorData QoS, matching real sensor drivers.
==============================================================================
*/

#pragma once

#include <cstdint>
#include <random>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace sensorforge::sensors {

class SyntheticPublisherBase : public rclcpp::Node
{
public:
  explicit SyntheticPublisherBase(const std::string & node_name)
  : rclcpp::Node(node_name)
  {
    rate_hz_ = this->declare_parameter<double>("rate_hz", 10.0);
    seed_ = static_cast<uint64_t>(this->declare_parameter<int64_t>("seed", 42));
    frame_id_ = this->declare_parameter<std::string>("frame_id", "sensor");
    topic_ = this->declare_parameter<std::string>("topic", "sensor");
    rng_.seed(seed_);
  }

protected:
  /// Called once per tick; subclass builds + publishes its message.
  virtual void on_tick(uint64_t seq, const rclcpp::Time & stamp) = 0;

  /// Start the periodic timer. Call after the concrete publisher is created.
  void start()
  {
    const double hz = rate_hz_ > 0.0 ? rate_hz_ : 1.0;
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / hz));
    timer_ = this->create_wall_timer(
      period,
      [this]() {
        const rclcpp::Time stamp = this->now();
        on_tick(seq_, stamp);
        ++seq_;
      });
    RCLCPP_INFO(
      this->get_logger(), "Publishing %s at %.1f Hz (seed=%llu)",
      topic_.c_str(), rate_hz_, static_cast<unsigned long long>(seed_));
  }

  /// Gaussian noise sample with the given sigma (deterministic given seed).
  double noise(double sigma)
  {
    std::normal_distribution<double> d(0.0, sigma);
    return d(rng_);
  }

  /// Uniform sample in [lo, hi].
  double uniform(double lo, double hi)
  {
    std::uniform_real_distribution<double> d(lo, hi);
    return d(rng_);
  }

  double rate_hz_ = 10.0;
  uint64_t seed_ = 42;
  std::string frame_id_ = "sensor";
  std::string topic_ = "sensor";
  std::mt19937_64 rng_;
  uint64_t seq_ = 0;

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sensorforge::sensors
