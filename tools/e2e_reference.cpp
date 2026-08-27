/*
==============================================================================
SensorForge - End-to-end reference workload
Part of the SensorForge AV HIL validation platform.

THE POINT OF THIS BINARY
------------------------
The audit found that every previously reported metric measured something other
than the SensorForge pipeline: 9.8M msg/s was a single-threaded, cache-hot
push+pop of a POD with clock overhead inside the number, and the scenario
runner's latencies measured ROS2/DDS loopback because sensor data never
traversed the frame codec, the ring or the WAL at all.

This workload drives the REAL path, end to end, with a realistic multi-sensor
mix at realistic payload sizes:

  producer thread per stream (paced at the sensor's rate)
    -> Recorder::capture()   per-stream sequence + monotonised timestamp
    -> StreamBuffer          bounded by frames AND bytes, per-stream policy
    -> drain thread
    -> encode_frame()        magic/version/seq/timestamp + CRC32C
    -> WalWriter::append()   segmented, CRC'd, restart-safe
    -> frame sink            receiver-side FrameDecoder: CRC + per-stream
                             sequence/gap validation (the bridge's receive path)

then, after the run:
    -> wal_replay digest     deterministic replay over what was recorded

Every number it prints is produced at run time on the host it runs on. Nothing
here is hard-coded. Results are written as JSON so they can be committed as a
raw artifact rather than transcribed by hand.

Payload sizes mirror the repository's own synthetic publishers:
  lidar   16384 points x 12 B  = 196,608 B  @  10 Hz   (PointCloud2)
  camera  320 x 240 x 3        = 230,400 B  @  30 Hz   (Image rgb8)
  imu                              330 B    @ 200 Hz   (Imu)
  gps                              100 B    @  20 Hz   (NavSatFix)
  can                               16 B    @ 100 Hz   (can_frame + telemetry)
==============================================================================
*/

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "sensorforge/offload/uploader.hpp"
#include "sensorforge/pipeline/recorder.hpp"
#include "sensorforge/protocol/frame_codec.hpp"
#include "sensorforge/replay/wal_reader.hpp"

using namespace sensorforge;
namespace fs = std::filesystem;

namespace {

struct SensorSpec
{
  const char * name;
  protocol::SensorType type;
  size_t payload_bytes;
  double rate_hz;
};

// The reference mix. Sizes and rates mirror sensors/*_publisher.cpp defaults.
const SensorSpec kMix[] = {
  {"lidar", protocol::SensorType::kLidar, 196608, 10.0},
  {"camera", protocol::SensorType::kCamera, 230400, 30.0},
  {"imu", protocol::SensorType::kImu, 330, 200.0},
  {"gps", protocol::SensorType::kGps, 100, 20.0},
  {"can", protocol::SensorType::kCan, 16, 100.0},
};
constexpr size_t kMixCount = sizeof(kMix) / sizeof(kMix[0]);

double now_s()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

int main(int argc, char ** argv)
{
  double duration_s = 20.0;
  double rate_scale = 1.0;
  std::string wal_dir;
  std::string json_out;
  std::string fsync_policy = "on_segment_seal";
  size_t segment_mb = 16;
  bool saturate = false;
  bool keep = false;
  std::string offload_dir;
  bool offload_down = false;

  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--duration") && i + 1 < argc) {
      duration_s = std::atof(argv[++i]);
    } else if (!std::strcmp(argv[i], "--rate-scale") && i + 1 < argc) {
      rate_scale = std::atof(argv[++i]);
    } else if (!std::strcmp(argv[i], "--wal-dir") && i + 1 < argc) {
      wal_dir = argv[++i];
    } else if (!std::strcmp(argv[i], "--json") && i + 1 < argc) {
      json_out = argv[++i];
    } else if (!std::strcmp(argv[i], "--fsync") && i + 1 < argc) {
      fsync_policy = argv[++i];
    } else if (!std::strcmp(argv[i], "--segment-mb") && i + 1 < argc) {
      segment_mb = static_cast<size_t>(std::atoi(argv[++i]));
    } else if (!std::strcmp(argv[i], "--offload-dir") && i + 1 < argc) {
      offload_dir = argv[++i];
    } else if (!std::strcmp(argv[i], "--offload-down")) {
      offload_down = true;   // measure recording throughput with a dead sink
    } else if (!std::strcmp(argv[i], "--keep")) {
      keep = true;   // leave the WAL on disk (for wal_replay demos / CI)
    } else if (!std::strcmp(argv[i], "--saturate")) {
      // Producers run flat out instead of paced: exercises the backpressure
      // policies and the byte budget rather than steady-state throughput.
      saturate = true;
    } else {
      std::fprintf(stderr,
        "usage: e2e_reference [--duration S] [--rate-scale X] [--wal-dir D]\n"
        "                     [--json OUT] [--fsync never|interval|every_record|"
        "on_segment_seal]\n"
        "                     [--segment-mb N] [--saturate] [--keep]\n"
        "                     [--offload-dir D] [--offload-down]\n");
      return 2;
    }
  }

  if (wal_dir.empty()) {
    wal_dir = (fs::temp_directory_path() / "sensorforge_e2e").string();
  }
  std::error_code ec;
  fs::remove_all(wal_dir, ec);

  // ---- configure the recorder ---------------------------------------------
  pipeline::RecorderConfig cfg;
  cfg.wal_dir = wal_dir;
  cfg.wal.segment_bytes = segment_mb * 1024u * 1024u;
  cfg.wal.fsync_policy = replay::fsync_policy_from_string(fsync_policy);
  cfg.wal.fsync_interval_ms = 500;
  // A byte budget with teeth: hard breach sheds instead of growing.
  cfg.budget.soft_queue_bytes = 192u * 1024u * 1024u;
  cfg.budget.hard_queue_bytes = 384u * 1024u * 1024u;

  pipeline::Recorder recorder(cfg);

  // ---- optional offload ----------------------------------------------------
  // Sealed segments are handed to the uploader. Recording must never block on
  // it, which is what --offload-down measures.
  std::shared_ptr<offload::FilesystemDestination> dest;
  std::unique_ptr<offload::Uploader> uploader;
  if (!offload_dir.empty()) {
    fs::remove_all(offload_dir, ec);
    dest = std::make_shared<offload::FilesystemDestination>(offload_dir);
    dest->set_available(!offload_down);
    offload::UploaderConfig ucfg;
    ucfg.base_backoff_ms = 20;
    ucfg.max_backoff_ms = 200;
    uploader = std::make_unique<offload::Uploader>(dest, wal_dir, ucfg);
    recorder.set_segment_sealed_hook(
      [&uploader](const std::string & path, uint32_t id) {uploader->enqueue(path, id);});
    uploader->start();
  }

  std::vector<size_t> ids;
  for (const auto & m : kMix) {
    pipeline::StreamSpec spec;
    spec.name = m.name;
    spec.sensor_type = m.type;
    spec.policy = core::default_policy_for(m.type);
    // Per-stream limits sized to the payload: a camera stream is allowed fewer
    // frames but the same byte ceiling as a CAN stream. This is exactly the
    // runtime configurability the fixed 1024-slot compile-time ring lacked.
    spec.limits.max_frames = m.payload_bytes > 65536 ? 64 : 512;
    spec.limits.max_bytes = 48u * 1024u * 1024u;
    ids.push_back(recorder.add_stream(spec));
  }

  // ---- receiver-side validation sink --------------------------------------
  // This is the bridge's receive path: every encoded frame is re-validated
  // (magic/version/CRC32C header+payload) and checked for per-stream sequence
  // integrity, so CRC failures and sequence gaps are MEASURED, not assumed.
  protocol::FrameDecoder decoder;
  std::mutex decoder_mtx;
  std::atomic<uint64_t> frames_validated{0};
  std::atomic<uint64_t> frames_rejected{0};
  std::atomic<uint64_t> crc_failures{0};
  std::atomic<uint64_t> sink_bytes{0};

  recorder.set_frame_sink(
    [&](const std::string & stream, protocol::SensorType,
    const std::vector<uint8_t> & frame) {
      sink_bytes.fetch_add(frame.size(), std::memory_order_relaxed);
      // Per-topic stream key -- the audit found the bridge used a constant 0,
      // which made per-topic sequence integrity unobservable.
      const uint64_t key = std::hash<std::string>{}(stream);
      protocol::FrameHeader h;
      std::lock_guard<std::mutex> lk(decoder_mtx);
      const auto err = decoder.decode(frame.data(), frame.size(), key, h);
      if (err == protocol::FrameError::kOk) {
        frames_validated.fetch_add(1, std::memory_order_relaxed);
      } else {
        frames_rejected.fetch_add(1, std::memory_order_relaxed);
        if (err == protocol::FrameError::kHeaderCrcMismatch ||
          err == protocol::FrameError::kPayloadCrcMismatch)
        {
          crc_failures.fetch_add(1, std::memory_order_relaxed);
        }
      }
    });

  recorder.start();

  // ---- producers ----------------------------------------------------------
  std::atomic<bool> stop{false};
  std::vector<std::thread> producers;
  std::vector<uint64_t> attempts(kMixCount, 0);

  const double t_start = now_s();
  for (size_t i = 0; i < kMixCount; ++i) {
    producers.emplace_back(
      [&, i]() {
        const SensorSpec & m = kMix[i];
        std::vector<uint8_t> payload(m.payload_bytes);
        // Deterministic, cheap content; the pipeline cost is what is measured,
        // not payload synthesis.
        for (size_t k = 0; k < payload.size(); ++k) {
          payload[k] = static_cast<uint8_t>((k * 31 + i) & 0xFF);
        }
        const double period = 1.0 / (m.rate_hz * rate_scale);
        auto next = std::chrono::steady_clock::now();
        uint64_t n = 0;
        while (!stop.load(std::memory_order_relaxed)) {
          // Vary a byte so identical frames are not trivially compressible or
          // cache-identical between iterations.
          payload[n % payload.size()] = static_cast<uint8_t>(n);
          recorder.capture(ids[i], payload.data(), payload.size());
          ++n;
          if (!saturate) {
            next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(period));
            std::this_thread::sleep_until(next);
          }
        }
        attempts[i] = n;
      });
  }

  // 1 Hz resource sampling, as designed: not on the per-message path.
  std::thread sampler(
    [&]() {
      while (!stop.load(std::memory_order_relaxed)) {
        recorder.poll_resources();
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
      }
      recorder.poll_resources();
    });

  std::this_thread::sleep_for(
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(duration_s)));
  stop.store(true);
  for (auto & t : producers) {
    t.join();
  }
  sampler.join();
  const double t_producers_done = now_s();

  recorder.stop();   // drains everything still queued, then closes the WAL
  const double t_drained = now_s();

  offload::UploaderStats ost{};
  double offload_drain_ms = 0.0;
  if (uploader) {
    ost = uploader->stats();
    if (offload_down) {
      // Bring the destination back and time the backlog drain.
      const double d0 = now_s();
      dest->set_available(true);
      uploader->wait_drained(60000);
      offload_drain_ms = (now_s() - d0) * 1000.0;
    } else {
      uploader->wait_drained(60000);
    }
    ost = uploader->stats();
    uploader->stop();
  }

  const auto st = recorder.stats();
  const double wall_s = t_drained - t_start;
  const double produce_s = t_producers_done - t_start;

  // ---- replay + digest ----------------------------------------------------
  const double t_replay0 = now_s();
  const auto digest = replay::compute_replay_digest(wal_dir);
  const double replay_ms = (now_s() - t_replay0) * 1000.0;

  // Re-validate every recorded frame through the decoder to prove the WAL holds
  // exactly what the framer produced.
  uint64_t replay_frames_ok = 0, replay_frames_bad = 0;
  replay::stream_replay(
    wal_dir,
    [&](const replay::ReplayRecord & r) {
      protocol::FrameHeader h;
      if (protocol::decode_header(r.payload.data(), r.payload.size(), h) ==
        protocol::FrameError::kOk)
      {
        ++replay_frames_ok;
      } else {
        ++replay_frames_bad;
      }
    });

  // ---- restart-recovery timing --------------------------------------------
  const double t_rec0 = now_s();
  replay::RecoveryReport rec;
  {
    replay::WalWriter w2(wal_dir, cfg.wal);
    rec = w2.recovery();
  }
  const double recovery_ms = (now_s() - t_rec0) * 1000.0;

  const auto dstats = decoder.stats();

  // ---- report -------------------------------------------------------------
  uint64_t total_attempts = 0;
  for (auto a : attempts) {
    total_attempts += a;
  }
  const double msgs_per_s = wall_s > 0 ? st.total_captured / wall_s : 0.0;
  const double mb_per_s = wall_s > 0
    ? (static_cast<double>(st.total_frame_bytes) / (1024.0 * 1024.0)) / wall_s : 0.0;
  const double wal_mb_per_s = wall_s > 0
    ? (static_cast<double>(st.wal_bytes) / (1024.0 * 1024.0)) / wall_s : 0.0;
  const double drop_rate = total_attempts > 0
    ? 100.0 * static_cast<double>(st.total_dropped) / static_cast<double>(total_attempts) : 0.0;

  std::printf("\n===== SensorForge end-to-end reference workload =====\n");
  std::printf("mode                 %s\n", saturate ? "saturate" : "paced");
  std::printf("duration_s           %.3f (producers %.3f, drain tail %.3f)\n",
    wall_s, produce_s, wall_s - produce_s);
  std::printf("fsync_policy         %s\n", replay::to_string(cfg.wal.fsync_policy));
  std::printf("streams              %zu\n", kMixCount);
  std::printf("captured             %llu\n", (unsigned long long)st.total_captured);
  std::printf("recorded             %llu\n", (unsigned long long)st.total_recorded);
  std::printf("dropped              %llu (%.4f%%)\n",
    (unsigned long long)st.total_dropped, drop_rate);
  std::printf("overwritten          %llu\n", (unsigned long long)st.total_overwritten);
  std::printf("shed_events          %llu\n", (unsigned long long)st.shed_events);
  std::printf("msgs_per_sec         %.1f\n", msgs_per_s);
  std::printf("frame_MB_per_sec     %.3f\n", mb_per_s);
  std::printf("wal_MB_per_sec       %.3f\n", wal_mb_per_s);
  std::printf("wal_records          %llu\n", (unsigned long long)st.wal_records);
  std::printf("wal_bytes            %llu\n", (unsigned long long)st.wal_bytes);
  std::printf("wal_fsyncs           %llu\n", (unsigned long long)st.wal_fsyncs);
  std::printf("capture_to_record_us p50=%.1f p99=%.1f p999=%.1f (n=%llu)\n",
    st.p50_capture_to_record_us, st.p99_capture_to_record_us,
    st.p999_capture_to_record_us, (unsigned long long)st.latency_samples);
  std::printf("frames_validated     %llu\n", (unsigned long long)frames_validated.load());
  std::printf("frames_rejected      %llu\n", (unsigned long long)frames_rejected.load());
  std::printf("crc_failures         %llu\n", (unsigned long long)crc_failures.load());
  std::printf("sequence_gaps        %llu (missing=%llu)\n",
    (unsigned long long)dstats.sequence_gaps,
    (unsigned long long)dstats.missing_sequences);
  std::printf("decoder_streams      %llu\n", (unsigned long long)dstats.streams_tracked);
  std::printf("peak_rss_bytes       %llu (%.1f MiB)\n",
    (unsigned long long)st.peak_rss_bytes,
    static_cast<double>(st.peak_rss_bytes) / (1024.0 * 1024.0));
  std::printf("peak_cpu_percent     %.1f\n", st.peak_cpu_percent);
  std::printf("peak_queued_bytes    %llu\n", (unsigned long long)st.peak_queued_bytes);
  std::printf("budget_state         %s\n", core::to_string(st.budget_state));
  std::printf("replay_digest        %08x (records=%llu bytes=%llu)\n",
    digest.digest, (unsigned long long)digest.records,
    (unsigned long long)digest.payload_bytes);
  std::printf("replay_ms            %.3f\n", replay_ms);
  std::printf("replay_frames        ok=%llu bad=%llu\n",
    (unsigned long long)replay_frames_ok, (unsigned long long)replay_frames_bad);
  std::printf("recovery_ms          %.3f (segments=%u tail_records=%llu truncated=%llu)\n",
    recovery_ms, rec.segments_found,
    (unsigned long long)rec.valid_records_in_tail,
    (unsigned long long)rec.truncated_bytes);
  if (uploader) {
    std::printf("offload              dest=%s enqueued=%llu uploaded=%llu "
      "failed_attempts=%llu backlog=%zu drain_ms=%.1f\n",
      offload_down ? "started_down" : "healthy",
      (unsigned long long)ost.enqueued, (unsigned long long)ost.uploaded,
      (unsigned long long)ost.failed_attempts, ost.backlog, offload_drain_ms);
  }
  std::printf("\nper-stream:\n");
  std::printf("  %-8s %-10s %-18s %9s %9s %9s %9s %11s\n",
    "stream", "type", "policy", "captured", "recorded", "dropped", "overwr", "peak_bytes");
  for (const auto & s : st.streams) {
    std::printf("  %-8s %-10s %-18s %9llu %9llu %9llu %9llu %11llu\n",
      s.name.c_str(), std::string(protocol::to_string(s.sensor_type)).c_str(),
      s.policy.c_str(),
      (unsigned long long)s.captured, (unsigned long long)s.recorded,
      (unsigned long long)s.buffer.dropped, (unsigned long long)s.buffer.overwritten,
      (unsigned long long)s.buffer.peak_queued_bytes);
  }
  std::printf("\n");

  if (!json_out.empty()) {
    std::ofstream j(json_out);
    j << "{\n";
    j << "  \"mode\": \"" << (saturate ? "saturate" : "paced") << "\",\n";
    j << "  \"duration_s\": " << wall_s << ",\n";
    j << "  \"fsync_policy\": \"" << replay::to_string(cfg.wal.fsync_policy) << "\",\n";
    j << "  \"streams\": " << kMixCount << ",\n";
    j << "  \"captured\": " << st.total_captured << ",\n";
    j << "  \"recorded\": " << st.total_recorded << ",\n";
    j << "  \"dropped\": " << st.total_dropped << ",\n";
    j << "  \"drop_rate_pct\": " << drop_rate << ",\n";
    j << "  \"overwritten\": " << st.total_overwritten << ",\n";
    j << "  \"shed_events\": " << st.shed_events << ",\n";
    j << "  \"msgs_per_sec\": " << msgs_per_s << ",\n";
    j << "  \"frame_mb_per_sec\": " << mb_per_s << ",\n";
    j << "  \"wal_mb_per_sec\": " << wal_mb_per_s << ",\n";
    j << "  \"wal_records\": " << st.wal_records << ",\n";
    j << "  \"wal_bytes\": " << st.wal_bytes << ",\n";
    j << "  \"wal_fsyncs\": " << st.wal_fsyncs << ",\n";
    j << "  \"capture_to_record_us\": {\"p50\": " << st.p50_capture_to_record_us
      << ", \"p99\": " << st.p99_capture_to_record_us
      << ", \"p999\": " << st.p999_capture_to_record_us
      << ", \"samples\": " << st.latency_samples << "},\n";
    j << "  \"frames_validated\": " << frames_validated.load() << ",\n";
    j << "  \"frames_rejected\": " << frames_rejected.load() << ",\n";
    j << "  \"crc_failures\": " << crc_failures.load() << ",\n";
    j << "  \"sequence_gaps\": " << dstats.sequence_gaps << ",\n";
    j << "  \"missing_sequences\": " << dstats.missing_sequences << ",\n";
    j << "  \"peak_rss_bytes\": " << st.peak_rss_bytes << ",\n";
    j << "  \"peak_cpu_percent\": " << st.peak_cpu_percent << ",\n";
    j << "  \"peak_queued_bytes\": " << st.peak_queued_bytes << ",\n";
    j << "  \"budget_state\": \"" << core::to_string(st.budget_state) << "\",\n";
    j << "  \"replay_digest\": \"" << std::hex << digest.digest << std::dec << "\",\n";
    j << "  \"replay_records\": " << digest.records << ",\n";
    j << "  \"replay_ms\": " << replay_ms << ",\n";
    j << "  \"replay_frames_ok\": " << replay_frames_ok << ",\n";
    j << "  \"replay_frames_bad\": " << replay_frames_bad << ",\n";
    j << "  \"recovery_ms\": " << recovery_ms << ",\n";
    j << "  \"per_stream\": [\n";
    for (size_t i = 0; i < st.streams.size(); ++i) {
      const auto & s = st.streams[i];
      j << "    {\"name\": \"" << s.name << "\", \"type\": \""
        << std::string(protocol::to_string(s.sensor_type)) << "\", \"policy\": \""
        << s.policy << "\", \"captured\": " << s.captured
        << ", \"recorded\": " << s.recorded
        << ", \"dropped\": " << s.buffer.dropped
        << ", \"overwritten\": " << s.buffer.overwritten
        << ", \"peak_queued_bytes\": " << s.buffer.peak_queued_bytes
        << ", \"frame_bytes\": " << s.frame_bytes << "}"
        << (i + 1 < st.streams.size() ? "," : "") << "\n";
    }
    j << "  ]\n}\n";
    std::printf("wrote %s\n", json_out.c_str());
  }

  if (!keep) {
    fs::remove_all(wal_dir, ec);
  } else {
    std::printf("kept WAL at %s\n", wal_dir.c_str());
  }
  return (frames_rejected.load() == 0 && replay_frames_bad == 0) ? 0 : 1;
}
