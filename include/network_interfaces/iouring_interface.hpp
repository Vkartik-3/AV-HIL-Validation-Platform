/*
==============================================================================
SensorForge - io_uring UDP transport backend (Extension E)
Part of the SensorForge AV HIL validation platform.

An alternative to the Boost.Asio UdpInterface built on Linux io_uring:
  - a fixed-size buffer pool registered with the kernel (io_uring_register_
    buffers) so there is no per-message allocation on the hot path,
  - batched submission of recvmsg / sendmsg SQEs,
  - a dedicated completion thread draining the CQ and re-arming receives.

Registered as a third pluginlib network interface. Linux + liburing only; on
other platforms the class compiles to a stub that reports failure so the plugin
library still builds.
==============================================================================
*/

#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include "network_interfaces/network_interface_base.hpp"

#if defined(__linux__) && defined(SENSORFORGE_HAVE_LIBURING)
#include <liburing.h>
#include <netinet/in.h>
#define SENSORFORGE_IOURING_ENABLED 1
#endif

namespace network_bridge
{

class IoUringInterface : public NetworkInterface
{
public:
  IoUringInterface()
  : NetworkInterface() {}

  ~IoUringInterface() override
  {
    close();
  }

protected:
  void initialize_() override;

public:
  bool has_failed() const override;
  bool is_ready() const override;
  void open() override;
  void close() override;
  void write(const std::vector<uint8_t> & data) override;

private:
  void load_parameters();

  std::string local_address_;
  int receive_port_ = 0;
  std::string remote_address_;
  int send_port_ = 0;

  std::atomic<bool> ready_{false};
  std::atomic<bool> failed_{false};
  std::atomic<bool> running_{false};

#if defined(SENSORFORGE_IOURING_ENABLED)
  // Buffer pool: fixed-size slots registered with the ring.
  static constexpr uint32_t kNumRecvBuffers = 64;
  static constexpr uint32_t kNumSendBuffers = 64;
  static constexpr size_t kBufferSize = 65536;

  struct Slot
  {
    std::vector<uint8_t> buf;
    struct iovec iov;
    struct msghdr msg;
    struct sockaddr_in addr;
    uint32_t index;
    bool is_recv;
  };

  void setup_sockets();
  void arm_recv(Slot & slot);
  void completion_loop();
  Slot * acquire_send_slot();

  struct io_uring ring_;
  bool ring_ready_ = false;
  int recv_fd_ = -1;
  int send_fd_ = -1;
  struct sockaddr_in remote_sockaddr_ {};

  std::vector<Slot> recv_slots_;
  std::vector<Slot> send_slots_;
  std::vector<struct iovec> registered_iovecs_;
  std::atomic<uint32_t> next_send_slot_{0};

  std::thread cq_thread_;
#endif
};

}  // namespace network_bridge
