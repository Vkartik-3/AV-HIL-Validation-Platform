/*
==============================================================================
SensorForge - wal_replay: deterministic replay tool
Part of the SensorForge AV HIL validation platform.

The audit found WalReader::replay() was written, tested and reachable from NO
shipped binary -- the recording was write-only in practice. This is the
consumer that closes that gap.

Usage:
  wal_replay --dir <wal_dir> [--mode deterministic|realtime|fast]
             [--speed N] [--start-ns T] [--digest] [--verify] [--quiet]

  --digest   print a stable CRC32C over the ordered replay output. Two replays
             of the same intact recording print the same digest; any change to
             content, ordering or record count changes it. This is the value the
             record/replay regression test compares.
  --verify   additionally run decode_header() over each record's payload, which
             re-validates the SensorForge frame the recorder wrote.

Exit codes: 0 success, 1 no records found, 2 bad usage, 3 verification failure.
==============================================================================
*/

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "sensorforge/protocol/frame_codec.hpp"
#include "sensorforge/replay/wal_reader.hpp"

using namespace sensorforge;

namespace {

void usage()
{
  std::fprintf(stderr,
    "usage: wal_replay --dir <wal_dir> [--mode deterministic|realtime|fast]\n"
    "                  [--speed N] [--start-ns T] [--digest] [--verify] [--quiet]\n");
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string dir;
  std::string mode = "deterministic";
  double speed = 1.0;
  uint64_t start_ns = 0;
  bool want_digest = false;
  bool verify = false;
  bool quiet = false;

  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--dir") && i + 1 < argc) {
      dir = argv[++i];
    } else if (!std::strcmp(argv[i], "--mode") && i + 1 < argc) {
      mode = argv[++i];
    } else if (!std::strcmp(argv[i], "--speed") && i + 1 < argc) {
      speed = std::atof(argv[++i]);
    } else if (!std::strcmp(argv[i], "--start-ns") && i + 1 < argc) {
      start_ns = std::strtoull(argv[++i], nullptr, 10);
    } else if (!std::strcmp(argv[i], "--digest")) {
      want_digest = true;
    } else if (!std::strcmp(argv[i], "--verify")) {
      verify = true;
    } else if (!std::strcmp(argv[i], "--quiet")) {
      quiet = true;
    } else {
      usage();
      return 2;
    }
  }
  if (dir.empty()) {
    usage();
    return 2;
  }

  replay::ReplayMode m = replay::ReplayMode::kDeterministic;
  if (mode == "realtime") {
    m = replay::ReplayMode::kRealTime;
  } else if (mode == "fast") {
    m = replay::ReplayMode::kFast;
  } else if (mode != "deterministic") {
    usage();
    return 2;
  }

  const auto t0 = std::chrono::steady_clock::now();

  uint64_t delivered = 0;
  uint64_t payload_bytes = 0;
  uint64_t frame_ok = 0;
  uint64_t frame_bad = 0;

  // Streaming: one segment resident at a time, never the whole recording.
  const replay::ReplayStats stats = replay::stream_replay(
    dir,
    [&](const replay::ReplayRecord & r) {
      ++delivered;
      payload_bytes += r.payload.size();
      if (verify) {
        protocol::FrameHeader h;
        if (protocol::decode_header(r.payload.data(), r.payload.size(), h) ==
          protocol::FrameError::kOk)
        {
          ++frame_ok;
        } else {
          ++frame_bad;
        }
      }
    },
    m, speed, start_ns);

  const double elapsed_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

  if (want_digest) {
    const auto d = replay::compute_replay_digest(dir, start_ns);
    std::printf("[REPLAY] digest=%08x records=%llu payload_bytes=%llu\n",
      d.digest,
      static_cast<unsigned long long>(d.records),
      static_cast<unsigned long long>(d.payload_bytes));
  }

  if (!quiet) {
    std::printf(
      "[REPLAY] dir=%s mode=%s delivered=%llu segments=%u corrupt_skipped=%llu "
      "bytes_skipped=%llu payload_bytes=%llu elapsed_ms=%.3f\n",
      dir.c_str(), mode.c_str(),
      static_cast<unsigned long long>(delivered),
      stats.segments_read,
      static_cast<unsigned long long>(stats.records_corrupt_skipped),
      static_cast<unsigned long long>(stats.bytes_skipped),
      static_cast<unsigned long long>(payload_bytes),
      elapsed_ms);
    if (verify) {
      std::printf("[REPLAY] frame_verify ok=%llu bad=%llu\n",
        static_cast<unsigned long long>(frame_ok),
        static_cast<unsigned long long>(frame_bad));
    }
  }

  if (delivered == 0) {
    return 1;
  }
  if (verify && frame_bad > 0) {
    return 3;
  }
  return 0;
}
