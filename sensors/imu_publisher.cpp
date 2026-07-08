/*
==============================================================================
SensorForge - Synthetic IMU publisher (Extension I)
Publishes sensor_msgs/Imu from a simple synthetic motion model (gentle yaw
oscillation + gravity) with configurable noise sigma, deterministic per seed.
==============================================================================
*/

#include <cmath>

#include <sensor_msgs/msg/imu.hpp>

#include "synthetic_base.hpp"

namespace sensorforge::sensors {

class ImuPublisher : public SyntheticPublisherBase
{
public:
  ImuPublisher()
  : SyntheticPublisherBase("imu_publisher")
  {
    accel_sigma_ = this->declare_parameter<double>("accel_sigma", 0.05);
    gyro_sigma_ = this->declare_parameter<double>("gyro_sigma", 0.01);
    pub_ = this->create_publisher<sensor_msgs::msg::Imu>(topic_, rclcpp::SensorDataQoS());
    start();
  }

protected:
  void on_tick(uint64_t seq, const rclcpp::Time & stamp) override
  {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;

    // Synthetic yaw oscillation about Z.
    const double t = static_cast<double>(seq) / (rate_hz_ > 0 ? rate_hz_ : 1.0);
    const double yaw = 0.2 * std::sin(t * 0.5);
    msg.orientation.w = std::cos(yaw * 0.5);
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = std::sin(yaw * 0.5);

    msg.angular_velocity.x = noise(gyro_sigma_);
    msg.angular_velocity.y = noise(gyro_sigma_);
    msg.angular_velocity.z = 0.1 * std::cos(t * 0.5) + noise(gyro_sigma_);

    msg.linear_acceleration.x = noise(accel_sigma_);
    msg.linear_acceleration.y = noise(accel_sigma_);
    msg.linear_acceleration.z = 9.81 + noise(accel_sigma_);

    // Diagonal covariances reflecting the configured noise.
    msg.angular_velocity_covariance[0] = gyro_sigma_ * gyro_sigma_;
    msg.angular_velocity_covariance[4] = gyro_sigma_ * gyro_sigma_;
    msg.angular_velocity_covariance[8] = gyro_sigma_ * gyro_sigma_;
    msg.linear_acceleration_covariance[0] = accel_sigma_ * accel_sigma_;
    msg.linear_acceleration_covariance[4] = accel_sigma_ * accel_sigma_;
    msg.linear_acceleration_covariance[8] = accel_sigma_ * accel_sigma_;

    pub_->publish(msg);
  }

private:
  double accel_sigma_ = 0.05;
  double gyro_sigma_ = 0.01;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
};

}  // namespace sensorforge::sensors

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sensorforge::sensors::ImuPublisher>());
  rclcpp::shutdown();
  return 0;
}
