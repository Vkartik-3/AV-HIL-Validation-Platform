/*
==============================================================================
SensorForge - Prometheus HTTP exporter (Extension M)
Part of the SensorForge AV HIL validation platform.

A minimal hand-rolled HTTP/1.1 server (no heavy dependency) that serves the
registry's text exposition at GET /metrics on a background thread. POSIX
sockets; builds on Linux and macOS.
==============================================================================
*/

#pragma once

#include <atomic>
#include <cstdint>
#include <thread>

#include "metrics/registry.hpp"

namespace sensorforge::metrics {

class PrometheusExporter
{
public:
  PrometheusExporter(Registry & registry, uint16_t port = 9090);
  ~PrometheusExporter();

  PrometheusExporter(const PrometheusExporter &) = delete;
  PrometheusExporter & operator=(const PrometheusExporter &) = delete;

  /// Bind + listen + start the accept thread. Returns false on bind failure.
  bool start();
  void stop();

  uint16_t port() const {return port_;}
  bool running() const {return running_.load();}

private:
  void serve_loop();

  Registry & registry_;
  uint16_t port_;
  int listen_fd_ = -1;
  std::atomic<bool> running_{false};
  std::thread thread_;
};

}  // namespace sensorforge::metrics
