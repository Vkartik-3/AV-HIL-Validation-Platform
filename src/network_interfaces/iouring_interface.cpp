/*
==============================================================================
SensorForge - io_uring UDP transport backend (implementation, Extension E)
Part of the SensorForge AV HIL validation platform.
==============================================================================
*/

#include "network_interfaces/iouring_interface.hpp"

#if defined(SENSORFORGE_IOURING_ENABLED)
#include <arpa/inet.h>
#include <cstring>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace network_bridge
{

void IoUringInterface::initialize_()
{
  load_parameters();
}

void IoUringInterface::load_parameters()
{
  const std::string prefix = "IoUringInterface.";
  node_->declare_parameter(prefix + "local_address", std::string(""));
  node_->declare_parameter(prefix + "receive_port", 0);
  node_->declare_parameter(prefix + "remote_address", std::string(""));
  node_->declare_parameter(prefix + "send_port", 0);
  node_->get_parameter(prefix + "local_address", local_address_);
  node_->get_parameter(prefix + "receive_port", receive_port_);
  node_->get_parameter(prefix + "remote_address", remote_address_);
  node_->get_parameter(prefix + "send_port", send_port_);
}

bool IoUringInterface::has_failed() const {return failed_.load();}
bool IoUringInterface::is_ready() const {return ready_.load() && !failed_.load();}

#if !defined(SENSORFORGE_IOURING_ENABLED)

// --- Stub build (non-Linux or liburing unavailable) -------------------------
void IoUringInterface::open()
{
  RCLCPP_ERROR(
    node_->get_logger(),
    "IoUringInterface requires Linux + liburing; not available in this build");
  failed_.store(true);
}
void IoUringInterface::close() {}
void IoUringInterface::write(const std::vector<uint8_t> &) {}

#else

void IoUringInterface::setup_sockets()
{
  send_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  recv_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (send_fd_ < 0 || recv_fd_ < 0) {
    RCLCPP_FATAL(node_->get_logger(), "io_uring: failed to create sockets");
    failed_.store(true);
    return;
  }

  // Bind the receive socket to local_address:receive_port.
  sockaddr_in local {};
  local.sin_family = AF_INET;
  local.sin_port = htons(static_cast<uint16_t>(receive_port_));
  local.sin_addr.s_addr = local_address_.empty() ?
    htonl(INADDR_ANY) : ::inet_addr(local_address_.c_str());
  if (::bind(recv_fd_, reinterpret_cast<sockaddr *>(&local), sizeof(local)) < 0) {
    RCLCPP_FATAL(node_->get_logger(), "io_uring: failed to bind receive socket");
    failed_.store(true);
    return;
  }

  // Cache the remote endpoint for sends.
  std::memset(&remote_sockaddr_, 0, sizeof(remote_sockaddr_));
  remote_sockaddr_.sin_family = AF_INET;
  remote_sockaddr_.sin_port = htons(static_cast<uint16_t>(send_port_));
  remote_sockaddr_.sin_addr.s_addr = ::inet_addr(remote_address_.c_str());
}

void IoUringInterface::arm_recv(Slot & slot)
{
  struct io_uring_sqe * sqe = io_uring_get_sqe(&ring_);
  if (!sqe) {
    return;
  }
  std::memset(&slot.msg, 0, sizeof(slot.msg));
  slot.iov.iov_base = slot.buf.data();
  slot.iov.iov_len = slot.buf.size();
  slot.msg.msg_iov = &slot.iov;
  slot.msg.msg_iovlen = 1;
  slot.msg.msg_name = &slot.addr;
  slot.msg.msg_namelen = sizeof(slot.addr);
  io_uring_prep_recvmsg(sqe, recv_fd_, &slot.msg, 0);
  io_uring_sqe_set_data(sqe, &slot);
}

void IoUringInterface::open()
{
  failed_.store(false);
  ready_.store(false);
  setup_sockets();
  if (failed_.load()) {
    return;
  }

  if (io_uring_queue_init(kNumRecvBuffers + kNumSendBuffers + 8, &ring_, 0) < 0) {
    RCLCPP_FATAL(node_->get_logger(), "io_uring: queue_init failed");
    failed_.store(true);
    return;
  }
  ring_ready_ = true;

  // Allocate + register the fixed buffer pool (recv slots then send slots).
  recv_slots_.resize(kNumRecvBuffers);
  send_slots_.resize(kNumSendBuffers);
  registered_iovecs_.clear();
  registered_iovecs_.reserve(kNumRecvBuffers + kNumSendBuffers);
  uint32_t idx = 0;
  for (auto & s : recv_slots_) {
    s.buf.assign(kBufferSize, 0);
    s.index = idx++;
    s.is_recv = true;
    registered_iovecs_.push_back({s.buf.data(), s.buf.size()});
  }
  for (auto & s : send_slots_) {
    s.buf.assign(kBufferSize, 0);
    s.index = idx++;
    s.is_recv = false;
    registered_iovecs_.push_back({s.buf.data(), s.buf.size()});
  }
  // Pin the pool so the kernel need not re-map it per operation.
  io_uring_register_buffers(&ring_, registered_iovecs_.data(),
    static_cast<unsigned>(registered_iovecs_.size()));

  // Arm all receive slots.
  for (auto & s : recv_slots_) {
    arm_recv(s);
  }
  io_uring_submit(&ring_);

  running_.store(true);
  cq_thread_ = std::thread([this]() {completion_loop();});
  ready_.store(true);
  RCLCPP_INFO(node_->get_logger(), "io_uring transport ready (%u recv / %u send buffers)",
    kNumRecvBuffers, kNumSendBuffers);
}

void IoUringInterface::completion_loop()
{
  while (running_.load()) {
    struct io_uring_cqe * cqe = nullptr;
    // Wait with a timeout so we can observe running_ going false.
    __kernel_timespec ts{0, 50 * 1000 * 1000};   // 50 ms
    const int rc = io_uring_wait_cqe_timeout(&ring_, &cqe, &ts);
    if (rc < 0 || !cqe) {
      continue;
    }
    auto * slot = static_cast<Slot *>(io_uring_cqe_get_data(cqe));
    const int res = cqe->res;
    io_uring_cqe_seen(&ring_, cqe);

    if (slot && slot->is_recv) {
      if (res > 0) {
        recv_cb_(std::span<const uint8_t>(slot->buf.data(), static_cast<size_t>(res)));
      }
      // Re-arm this receive slot for the next datagram.
      arm_recv(*slot);
      io_uring_submit(&ring_);
    }
    // Send completions need no action; the slot returns to the pool by rotation.
  }
}

IoUringInterface::Slot * IoUringInterface::acquire_send_slot()
{
  // Round-robin the send pool. With kNumSendBuffers in flight this bounds the
  // outstanding sends; overrun simply reuses the oldest slot.
  const uint32_t i = next_send_slot_.fetch_add(1) % kNumSendBuffers;
  return &send_slots_[i];
}

void IoUringInterface::write(const std::vector<uint8_t> & data)
{
  if (!ready_.load() || failed_.load()) {
    return;
  }
  if (data.size() > kBufferSize) {
    RCLCPP_WARN(node_->get_logger(), "io_uring: datagram exceeds buffer size, dropping");
    return;
  }
  Slot * slot = acquire_send_slot();
  std::memcpy(slot->buf.data(), data.data(), data.size());

  std::memset(&slot->msg, 0, sizeof(slot->msg));
  slot->iov.iov_base = slot->buf.data();
  slot->iov.iov_len = data.size();
  slot->msg.msg_iov = &slot->iov;
  slot->msg.msg_iovlen = 1;
  slot->msg.msg_name = &remote_sockaddr_;
  slot->msg.msg_namelen = sizeof(remote_sockaddr_);

  struct io_uring_sqe * sqe = io_uring_get_sqe(&ring_);
  if (!sqe) {
    RCLCPP_WARN(node_->get_logger(), "io_uring: SQ full, dropping send");
    return;
  }
  io_uring_prep_sendmsg(sqe, send_fd_, &slot->msg, 0);
  io_uring_sqe_set_data(sqe, slot);
  io_uring_submit(&ring_);   // batched: submits any other queued SQEs too
}

void IoUringInterface::close()
{
  running_.store(false);
  if (cq_thread_.joinable()) {
    cq_thread_.join();
  }
  if (ring_ready_) {
    io_uring_unregister_buffers(&ring_);
    io_uring_queue_exit(&ring_);
    ring_ready_ = false;
  }
  if (recv_fd_ >= 0) {::close(recv_fd_); recv_fd_ = -1;}
  if (send_fd_ >= 0) {::close(send_fd_); send_fd_ = -1;}
  ready_.store(false);
}

#endif  // SENSORFORGE_IOURING_ENABLED

}  // namespace network_bridge

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(network_bridge::IoUringInterface, network_bridge::NetworkInterface)
