/*
==============================================================================
SensorForge - Synthetic GPS publisher (Extension I)
Publishes sensor_msgs/NavSatFix along a synthetic circular trajectory around a
configurable origin, with deterministic position noise.
==============================================================================
*/

#include <cmath>

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "synthetic_base.hpp"

namespace sensorforge::sensors {

class GpsPublisher : public SyntheticPublisherBase
{
public:
  GpsPublisher()
  : SyntheticPublisherBase("gps_publisher")
  {
    origin_lat_ = this->declare_parameter<double>("origin_lat", 40.4237);   // West Lafayette
    origin_lon_ = this->declare_parameter<double>("origin_lon", -86.9212);
    radius_m_ = this->declare_parameter<double>("radius_m", 50.0);
    pos_sigma_m_ = this->declare_parameter<double>("pos_sigma_m", 1.5);
    pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(topic_, rclcpp::SensorDataQoS());
    start();
  }

protected:
  void on_tick(uint64_t seq, const rclcpp::Time & stamp) override
  {
    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    // Circular trajectory; convert local metres offset to lat/lon degrees.
    const double t = static_cast<double>(seq) / (rate_hz_ > 0 ? rate_hz_ : 1.0);
    const double dx = radius_m_ * std::cos(t * 0.1) + noise(pos_sigma_m_);
    const double dy = radius_m_ * std::sin(t * 0.1) + noise(pos_sigma_m_);
    constexpr double kMetersPerDegLat = 111320.0;
    const double meters_per_deg_lon = kMetersPerDegLat * std::cos(origin_lat_ * M_PI / 180.0);
    msg.latitude = origin_lat_ + dy / kMetersPerDegLat;
    msg.longitude = origin_lon_ + dx / meters_per_deg_lon;
    msg.altitude = 200.0 + noise(pos_sigma_m_);

    msg.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    msg.position_covariance[0] = pos_sigma_m_ * pos_sigma_m_;
    msg.position_covariance[4] = pos_sigma_m_ * pos_sigma_m_;
    msg.position_covariance[8] = pos_sigma_m_ * pos_sigma_m_ * 4.0;

    pub_->publish(msg);
  }

private:
  double origin_lat_ = 40.4237;
  double origin_lon_ = -86.9212;
  double radius_m_ = 50.0;
  double pos_sigma_m_ = 1.5;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;
};

}  // namespace sensorforge::sensors

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sensorforge::sensors::GpsPublisher>());
  rclcpp::shutdown();
  return 0;
}
