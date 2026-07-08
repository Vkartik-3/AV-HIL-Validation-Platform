/*
==============================================================================
SensorForge - Synthetic LiDAR publisher (Extension I)
Publishes sensor_msgs/PointCloud2 with a configurable-density synthetic point
cloud (a noisy ring sweep), deterministic given the seed.
==============================================================================
*/

#include <cmath>
#include <cstring>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "synthetic_base.hpp"

namespace sensorforge::sensors {

class LidarPublisher : public SyntheticPublisherBase
{
public:
  LidarPublisher()
  : SyntheticPublisherBase("lidar_publisher")
  {
    num_points_ = static_cast<int>(this->declare_parameter<int64_t>("num_points", 16384));
    noise_sigma_ = this->declare_parameter<double>("noise_sigma_m", 0.02);
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      topic_, rclcpp::SensorDataQoS());
    start();
  }

protected:
  void on_tick(uint64_t seq, const rclcpp::Time & stamp) override
  {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.height = 1;
    msg.width = static_cast<uint32_t>(num_points_);
    msg.is_bigendian = false;
    msg.is_dense = true;

    // Three float32 fields: x, y, z.
    msg.fields.resize(3);
    const char * names[3] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i) {
      msg.fields[i].name = names[i];
      msg.fields[i].offset = static_cast<uint32_t>(i * 4);
      msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
      msg.fields[i].count = 1;
    }
    msg.point_step = 12;
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(static_cast<size_t>(msg.row_step));

    // Synthetic ring sweep: points on a circle of slowly varying radius, with
    // Gaussian range noise. Deterministic given the seed.
    const double base_radius = 5.0 + 0.5 * std::sin(static_cast<double>(seq) * 0.05);
    for (int i = 0; i < num_points_; ++i) {
      const double angle = (2.0 * M_PI * i) / num_points_;
      const double r = base_radius + noise(noise_sigma_);
      const float x = static_cast<float>(r * std::cos(angle));
      const float y = static_cast<float>(r * std::sin(angle));
      const float z = static_cast<float>(0.1 * std::sin(angle * 4.0) + noise(noise_sigma_));
      const size_t off = static_cast<size_t>(i) * msg.point_step;
      std::memcpy(&msg.data[off + 0], &x, 4);
      std::memcpy(&msg.data[off + 4], &y, 4);
      std::memcpy(&msg.data[off + 8], &z, 4);
    }
    pub_->publish(msg);
  }

private:
  int num_points_ = 16384;
  double noise_sigma_ = 0.02;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

}  // namespace sensorforge::sensors

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sensorforge::sensors::LidarPublisher>());
  rclcpp::shutdown();
  return 0;
}
