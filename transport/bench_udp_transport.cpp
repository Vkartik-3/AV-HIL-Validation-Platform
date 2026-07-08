/*
==============================================================================
SensorForge - io_uring vs Boost.Asio UDP benchmark (Extension E)
Part of the SensorForge AV HIL validation platform.

Measures loopback UDP round-trip latency (p50/p99) and throughput (msgs/sec) at
1 KB payloads for two backends:
  - udp_asio_1kb    : Boost.Asio synchronous send_to / receive_from
  - udp_iouring_1kb : io_uring sendmsg/recvmsg with a registered buffer

An echo thread bounces each datagram back. Emits [BENCH] lines matching the
project format. Linux + liburing only; run on EC2.

Build:
  see transport/README (built by the main CMake when liburing is present).
==============================================================================
*/

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <boost/asio.hpp>

#if defined(SENSORFORGE_HAVE_LIBURING)
#include <liburing.h>
#endif

namespace {

constexpr int kPayload = 1024;
constexpr int kIters = 50000;
constexpr uint16_t kEchoPort = 45999;

using clock_t_ = std::chrono::steady_clock;

struct Stats { double p50_ns, p99_ns, msgs_per_sec; };

Stats summarize(std::vector<double> & s, double total_s)
{
  std::sort(s.begin(), s.end());
  auto pct = [&](double p) {return s.empty() ? 0.0 : s[static_cast<size_t>(p * (s.size() - 1))];};
  return {pct(0.50), pct(0.99), total_s > 0 ? s.size() / total_s : 0.0};
}

// Blocking UDP echo server on loopback:kEchoPort. Runs until `stop` is set.
void echo_server(std::atomic<bool> & ready, std::atomic<bool> & stop)
{
  int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(kEchoPort);
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  ::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
  timeval tv{0, 100000};
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  ready.store(true);

  std::vector<uint8_t> buf(65536);
  while (!stop.load()) {
    sockaddr_in from{};
    socklen_t fl = sizeof(from);
    const ssize_t n = ::recvfrom(fd, buf.data(), buf.size(), 0,
      reinterpret_cast<sockaddr *>(&from), &fl);
    if (n > 0) {
      ::sendto(fd, buf.data(), static_cast<size_t>(n), 0,
        reinterpret_cast<sockaddr *>(&from), fl);
    }
  }
  ::close(fd);
}

void bench_asio()
{
  namespace asio = boost::asio;
  asio::io_context io;
  asio::ip::udp::socket sock(io, asio::ip::udp::endpoint(asio::ip::udp::v4(), 0));
  asio::ip::udp::endpoint echo(asio::ip::make_address("127.0.0.1"), kEchoPort);

  std::vector<uint8_t> out(kPayload, 0xAB), in(65536);
  std::vector<double> lat;
  lat.reserve(kIters);
  const auto t0 = clock_t_::now();
  for (int i = 0; i < kIters; ++i) {
    const auto a = clock_t_::now();
    sock.send_to(asio::buffer(out), echo);
    asio::ip::udp::endpoint from;
    sock.receive_from(asio::buffer(in), from);
    const auto b = clock_t_::now();
    lat.push_back(std::chrono::duration<double, std::nano>(b - a).count());
  }
  const double total = std::chrono::duration<double>(clock_t_::now() - t0).count();
  const Stats s = summarize(lat, total);
  std::printf("[BENCH] udp_asio_1kb        msgs/sec=%.0f p50=%.0fns p99=%.0fns\n",
    s.msgs_per_sec, s.p50_ns, s.p99_ns);
}

#if defined(SENSORFORGE_HAVE_LIBURING)
void bench_iouring()
{
  int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in local{};
  local.sin_family = AF_INET;
  local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  ::bind(fd, reinterpret_cast<sockaddr *>(&local), sizeof(local));
  sockaddr_in echo{};
  echo.sin_family = AF_INET;
  echo.sin_port = htons(kEchoPort);
  echo.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

  io_uring ring;
  io_uring_queue_init(8, &ring, 0);
  std::vector<uint8_t> out(kPayload, 0xCD), in(65536);
  iovec iov_out{out.data(), out.size()};
  iovec iov_in{in.data(), in.size()};
  iovec regs[2] = {iov_out, iov_in};
  io_uring_register_buffers(&ring, regs, 2);

  std::vector<double> lat;
  lat.reserve(kIters);
  const auto t0 = clock_t_::now();
  for (int i = 0; i < kIters; ++i) {
    const auto a = clock_t_::now();
    msghdr smsg{};
    smsg.msg_name = &echo; smsg.msg_namelen = sizeof(echo);
    smsg.msg_iov = &iov_out; smsg.msg_iovlen = 1;
    io_uring_sqe * sqe = io_uring_get_sqe(&ring);
    io_uring_prep_sendmsg(sqe, fd, &smsg, 0);
    io_uring_submit(&ring);
    io_uring_cqe * cqe = nullptr;
    io_uring_wait_cqe(&ring, &cqe);
    io_uring_cqe_seen(&ring, cqe);

    msghdr rmsg{};
    rmsg.msg_iov = &iov_in; rmsg.msg_iovlen = 1;
    sqe = io_uring_get_sqe(&ring);
    io_uring_prep_recvmsg(sqe, fd, &rmsg, 0);
    io_uring_submit(&ring);
    io_uring_wait_cqe(&ring, &cqe);
    io_uring_cqe_seen(&ring, cqe);
    const auto b = clock_t_::now();
    lat.push_back(std::chrono::duration<double, std::nano>(b - a).count());
  }
  const double total = std::chrono::duration<double>(clock_t_::now() - t0).count();
  const Stats s = summarize(lat, total);
  std::printf("[BENCH] udp_iouring_1kb     msgs/sec=%.0f p50=%.0fns p99=%.0fns\n",
    s.msgs_per_sec, s.p50_ns, s.p99_ns);

  io_uring_unregister_buffers(&ring);
  io_uring_queue_exit(&ring);
  ::close(fd);
}
#endif

}  // namespace

int main()
{
  std::atomic<bool> ready{false}, stop{false};
  std::thread echo(echo_server, std::ref(ready), std::ref(stop));
  while (!ready.load()) {std::this_thread::sleep_for(std::chrono::milliseconds(1));}

  std::printf("=== SensorForge UDP transport benchmark (1KB, %d iters RTT) ===\n", kIters);
  bench_asio();
#if defined(SENSORFORGE_HAVE_LIBURING)
  bench_iouring();
#else
  std::printf("[BENCH] udp_iouring_1kb     SKIPPED (build without liburing)\n");
#endif

  stop.store(true);
  echo.join();
  return 0;
}
