/*
==============================================================================
SensorForge - Prometheus HTTP exporter (implementation, Extension M)
Part of the SensorForge AV HIL validation platform.
==============================================================================
*/

#include "metrics/prometheus_exporter.hpp"

#include <cstring>
#include <string>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace sensorforge::metrics {

PrometheusExporter::PrometheusExporter(Registry & registry, uint16_t port)
: registry_(registry), port_(port) {}

PrometheusExporter::~PrometheusExporter()
{
  stop();
}

bool PrometheusExporter::start()
{
  listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd_ < 0) {
    return false;
  }
  int one = 1;
  ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port_);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (::bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }
  if (::listen(listen_fd_, 8) < 0) {
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }
  running_.store(true);
  thread_ = std::thread([this]() {serve_loop();});
  return true;
}

void PrometheusExporter::stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  if (listen_fd_ >= 0) {
    ::shutdown(listen_fd_, SHUT_RDWR);
    ::close(listen_fd_);
    listen_fd_ = -1;
  }
  if (thread_.joinable()) {
    thread_.join();
  }
}

void PrometheusExporter::serve_loop()
{
  while (running_.load()) {
    const int client = ::accept(listen_fd_, nullptr, nullptr);
    if (client < 0) {
      if (!running_.load()) {
        break;
      }
      continue;
    }
    // Read (and discard) the request line/headers.
    char buf[2048];
    const ssize_t n = ::recv(client, buf, sizeof(buf) - 1, 0);
    std::string path = "/";
    if (n > 0) {
      buf[n] = '\0';
      // Parse "GET <path> HTTP/1.1".
      const char * sp1 = std::strchr(buf, ' ');
      if (sp1) {
        const char * sp2 = std::strchr(sp1 + 1, ' ');
        if (sp2) {
          path.assign(sp1 + 1, sp2);
        }
      }
    }

    std::string body;
    std::string status = "200 OK";
    std::string ctype = "text/plain; version=0.0.4; charset=utf-8";
    if (path == "/metrics" || path == "/") {
      body = registry_.render();
    } else {
      status = "404 Not Found";
      body = "not found\n";
      ctype = "text/plain";
    }

    std::string resp = "HTTP/1.1 " + status + "\r\n";
    resp += "Content-Type: " + ctype + "\r\n";
    resp += "Content-Length: " + std::to_string(body.size()) + "\r\n";
    resp += "Connection: close\r\n\r\n";
    resp += body;
    ::send(client, resp.data(), resp.size(), 0);
    ::close(client);
  }
}

}  // namespace sensorforge::metrics
