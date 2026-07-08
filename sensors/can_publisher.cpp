/*
==============================================================================
SensorForge - Synthetic CAN publisher (Extension I)
Writes synthetic CAN frames to a virtual CAN interface (SocketCAN vcan0). No
physical hardware is required; the vcan kernel module provides the bus:

    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan
    sudo ip link set up vcan0

Publishes a rotating set of ECU-style signals (wheel speed, steering angle,
throttle) at the configured rate, deterministic given the seed. Linux only.
==============================================================================
*/

#include <cmath>
#include <cstring>

#include "synthetic_base.hpp"

#if defined(__linux__)
#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace sensorforge::sensors {

class CanPublisher : public SyntheticPublisherBase
{
public:
  CanPublisher()
  : SyntheticPublisherBase("can_publisher")
  {
    ifname_ = this->declare_parameter<std::string>("can_interface", "vcan0");
#if defined(__linux__)
    open_socket();
#else
    RCLCPP_WARN(this->get_logger(), "SocketCAN is Linux-only; CAN publisher is a no-op here");
#endif
    start();
  }

  ~CanPublisher() override
  {
#if defined(__linux__)
    if (sock_ >= 0) {
      ::close(sock_);
    }
#endif
  }

protected:
  void on_tick(uint64_t seq, const rclcpp::Time & /*stamp*/) override
  {
#if defined(__linux__)
    if (sock_ < 0) {
      return;
    }
    // Rotate across three ECU signal frames.
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    const double t = static_cast<double>(seq) / (rate_hz_ > 0 ? rate_hz_ : 1.0);
    const int which = static_cast<int>(seq % 3);
    switch (which) {
      case 0: {  // wheel speed, ID 0x100
        frame.can_id = 0x100;
        frame.can_dlc = 8;
        const uint16_t speed = static_cast<uint16_t>(
          std::abs(2000.0 * (1.0 + std::sin(t * 0.5)) + noise(20.0)));
        std::memcpy(frame.data, &speed, 2);
        break;
      }
      case 1: {  // steering angle, ID 0x200
        frame.can_id = 0x200;
        frame.can_dlc = 4;
        const int16_t angle = static_cast<int16_t>(500.0 * std::sin(t * 0.3) + noise(5.0));
        std::memcpy(frame.data, &angle, 2);
        break;
      }
      default: {  // throttle, ID 0x300
        frame.can_id = 0x300;
        frame.can_dlc = 2;
        const uint8_t throttle = static_cast<uint8_t>(
          std::min(255.0, std::max(0.0, 128.0 + 100.0 * std::sin(t * 0.2) + noise(4.0))));
        frame.data[0] = throttle;
        break;
      }
    }
    const ssize_t n = ::write(sock_, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame))) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000, "CAN write failed on %s", ifname_.c_str());
    }
#else
    (void)seq;
#endif
  }

private:
#if defined(__linux__)
  void open_socket()
  {
    sock_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CAN socket");
      return;
    }
    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, ifname_.c_str(), IFNAMSIZ - 1);
    if (::ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "CAN interface %s not found (did you bring up vcan0?)", ifname_.c_str());
      ::close(sock_);
      sock_ = -1;
      return;
    }
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(sock_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to bind CAN socket to %s", ifname_.c_str());
      ::close(sock_);
      sock_ = -1;
      return;
    }
    RCLCPP_INFO(this->get_logger(), "CAN publisher bound to %s", ifname_.c_str());
  }

  int sock_ = -1;
#endif
  std::string ifname_ = "vcan0";
};

}  // namespace sensorforge::sensors

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sensorforge::sensors::CanPublisher>());
  rclcpp::shutdown();
  return 0;
}
