/*
==============================================================================
SensorForge - Synthetic camera publisher (Extension I)
Publishes sensor_msgs/Image (rgb8) at a configurable resolution with a moving
synthetic gradient pattern, deterministic given the seed.

Default resolution is 320x240 rgb8 (230,400 B/frame -> ~6.9 MB/s at 30 Hz),
which stays comfortably within the practical DDS loopback throughput on a
typical cloud instance. A 640x480 frame is ~0.92 MB and at 30 Hz (~27.6 MB/s)
fragments into ~15 UDP datagrams over best-effort QoS -- losing any one drops
the whole frame -- which produced the systematic ~50% drop. Resolution stays
configurable via the `width`/`height` parameters for callers that want more.
==============================================================================
*/

#include <cstdint>

#include <sensor_msgs/msg/image.hpp>

#include "synthetic_base.hpp"

namespace sensorforge::sensors {

class CameraPublisher : public SyntheticPublisherBase
{
public:
  CameraPublisher()
  : SyntheticPublisherBase("camera_publisher")
  {
    width_ = static_cast<uint32_t>(this->declare_parameter<int64_t>("width", 320));
    height_ = static_cast<uint32_t>(this->declare_parameter<int64_t>("height", 240));
    noise_amp_ = static_cast<int>(this->declare_parameter<int64_t>("noise_amplitude", 8));
    pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_, rclcpp::SensorDataQoS());
    start();
  }

protected:
  void on_tick(uint64_t seq, const rclcpp::Time & stamp) override
  {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.height = height_;
    msg.width = width_;
    msg.encoding = "rgb8";
    msg.is_bigendian = 0;
    msg.step = width_ * 3;
    msg.data.resize(static_cast<size_t>(msg.step) * height_);

    // Diagonal gradient that scrolls with the sequence number; small
    // deterministic per-pixel noise. Uses a cheap inline LCG rather than
    // constructing a std::normal_distribution per pixel -- the latter dominated
    // CPU (hundreds of thousands of distribution samples per frame) and made the
    // timer callback block, throttling the effective publish rate below 30 Hz.
    const int phase = static_cast<int>(seq) & 0xFF;
    const int amp = noise_amp_;
    uint32_t rng = static_cast<uint32_t>(seed_) ^ static_cast<uint32_t>(seq * 2654435761u);
    auto next = [&rng]() -> uint32_t {
        rng = rng * 1664525u + 1013904223u;
        return rng;
      };
    for (uint32_t y = 0; y < height_; ++y) {
      for (uint32_t x = 0; x < width_; ++x) {
        const size_t off = static_cast<size_t>(y) * msg.step + static_cast<size_t>(x) * 3;
        const int n = amp > 0
          ? (static_cast<int>((next() >> 23) % static_cast<uint32_t>(2 * amp + 1)) - amp)
          : 0;
        const int r = (static_cast<int>(x + phase) & 0xFF) + n;
        const int g = (static_cast<int>(y + phase) & 0xFF) + n;
        const int b = (static_cast<int>(x + y) & 0xFF) + n;
        msg.data[off + 0] = static_cast<uint8_t>(r < 0 ? 0 : (r > 255 ? 255 : r));
        msg.data[off + 1] = static_cast<uint8_t>(g < 0 ? 0 : (g > 255 ? 255 : g));
        msg.data[off + 2] = static_cast<uint8_t>(b < 0 ? 0 : (b > 255 ? 255 : b));
      }
    }
    pub_->publish(msg);
  }

private:
  uint32_t width_ = 320;
  uint32_t height_ = 240;
  int noise_amp_ = 8;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

}  // namespace sensorforge::sensors

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sensorforge::sensors::CameraPublisher>());
  rclcpp::shutdown();
  return 0;
}
