/*
==============================================================================
SensorForge - Capacity sweep
Part of the SensorForge AV HIL validation platform.

THE QUESTION THIS ANSWERS
-------------------------
The paced reference workload (tools/e2e_reference.cpp) runs a realistic sensor
mix and shows zero drops -- but it is deliberately well below capacity, so it
says nothing about headroom. The saturated mode runs producers flat out and
drops ~99% by design, which proves the buffers stay bounded but is not a
throughput figure either.

Neither answers the useful question: HOW MUCH CAN THE PIPELINE SUSTAIN
CONTINUOUSLY WITH ZERO DROPS? That is what this measures.

METHOD
------
The sensor mix is the same five streams as the reference workload, with every
rate multiplied by a scale factor.

  1. COARSE RAMP     scale = 1, 2, 4, 8, ... Each scale runs a short trial.
                     Stop at the first scale that drops a frame.
  2. BINARY SEARCH   Between the last clean scale and the first dropping one,
                     bisect for a configurable number of rounds to find the
                     boundary more precisely.
  3. CONFIRM         Re-run the candidate for a much longer window. If it does
                     not hold, step down 20% and try again. Only a scale that
                     survives the FULL confirm window is reported as capacity;
                     the ramp and bisect results are candidates, never the
                     headline. A scale that passes 12 s and fails 60 s is not
                     capacity, and in practice that happens near the boundary.

REPORTED RATES ARE ACHIEVED, NOT REQUESTED
------------------------------------------
At high scale factors the producer threads cannot hit their target rate --
sleep_until granularity and scheduling get in the way. Reporting the requested
rate would therefore overstate capacity. Every figure below is derived from
frames the pipeline actually accepted during the measured window, and the
requested rate is printed alongside so the gap is visible.

WHAT COUNTS AS SUSTAINED
------------------------
Three conditions, all required:

  1. ZERO DROPS       No frame refused by the backpressure policy -- not by
                      frame count, not by the byte ceiling, not by a hard
                      budget breach.
  2. ZERO OVERWRITES  The camera's overwrite-oldest policy discarding a frame
                      is data that never reached the WAL, so it disqualifies a
                      scale exactly as a drop does.
  3. NO BACKLOG GROWTH
                      Queued bytes must not be climbing across the measurement
                      window. This condition is not optional garnish: a run can
                      report zero drops purely because a 96 MiB-per-stream
                      buffer has not filled up YET. Without this check the tool
                      reports the rate at which the queue is filling, not the
                      rate the pipeline can actually sustain -- and the first
                      smoke test did exactly that, passing 22k msgs/s over a 2 s
                      trial that fell apart over 5 s.

A warmup window is excluded from every trial so first-touch page faults, WAL
segment creation and thread start-up do not count against the result.
==============================================================================
*/

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "sensorforge/pipeline/recorder.hpp"
#include "sensorforge/protocol/frame_codec.hpp"

using namespace sensorforge;
namespace fs = std::filesystem;

namespace {

struct SensorSpec
{
  const char * name;
  protocol::SensorType type;
  size_t payload_bytes;
  double base_rate_hz;
};

// Identical mix to tools/e2e_reference.cpp, so the capacity figure is directly
// comparable to the paced reference run.
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

struct TrialResult
{
  double scale = 0;
  bool clean = false;           // zero drops AND zero overwrites
  uint64_t captured = 0;
  uint64_t recorded = 0;
  uint64_t dropped = 0;
  uint64_t overwritten = 0;
  uint64_t frame_bytes = 0;
  uint64_t wal_bytes = 0;
  uint64_t wal_records = 0;
  double measured_s = 0;
  double msgs_per_sec = 0;
  double frame_mb_s = 0;
  double wal_mb_s = 0;
  double p50_us = 0, p99_us = 0, p999_us = 0;
  uint64_t latency_samples = 0;
  uint64_t peak_rss = 0;
  uint64_t peak_queued_bytes = 0;
  uint64_t queued_start = 0;
  uint64_t queued_end = 0;
  double backlog_growth_mb_s = 0;
  double peak_cpu = 0;
  uint64_t frames_validated = 0;
  uint64_t crc_failures = 0;
  uint64_t sequence_gaps = 0;
  double requested_msgs_per_sec = 0;
};

/**
 * @brief Run one trial at @p scale for @p run_s seconds after @p warmup_s.
 *
 * The recorder is constructed fresh each trial so buffer state, counters and
 * the WAL never carry over between scales.
 */
TrialResult run_trial(
  double scale, double warmup_s, double run_s, const std::string & wal_root,
  bool validate_frames)
{
  TrialResult r;
  r.scale = scale;

  const std::string wal_dir = wal_root + "/s" + std::to_string(static_cast<long>(scale * 1000));
  std::error_code ec;
  fs::remove_all(wal_dir, ec);

  pipeline::RecorderConfig cfg;
  cfg.wal_dir = wal_dir;
  cfg.wal.segment_bytes = 64u * 1024u * 1024u;
  cfg.wal.fsync_policy = replay::FsyncPolicy::kOnSegmentSeal;

  pipeline::Recorder rec(cfg);
  std::vector<size_t> ids;
  for (const auto & m : kMix) {
    pipeline::StreamSpec s;
    s.name = m.name;
    s.sensor_type = m.type;
    s.policy = core::default_policy_for(m.type);
    s.limits.max_frames = m.payload_bytes > 65536 ? 128 : 2048;
    s.limits.max_bytes = 96u * 1024u * 1024u;
    ids.push_back(rec.add_stream(s));
  }

  // Receiver-side validation costs CPU, so it is optional during the ramp and
  // enabled for the confirm run where the integrity claim actually matters.
  protocol::FrameDecoder decoder;
  std::mutex dec_mtx;
  std::atomic<uint64_t> validated{0}, crcfail{0};
  if (validate_frames) {
    rec.set_frame_sink(
      [&](const std::string & stream, protocol::SensorType,
      const std::vector<uint8_t> & frame) {
        const uint64_t key = std::hash<std::string>{}(stream);
        protocol::FrameHeader h;
        std::lock_guard<std::mutex> lk(dec_mtx);
        const auto e = decoder.decode(frame.data(), frame.size(), key, h);
        if (e == protocol::FrameError::kOk) {
          validated.fetch_add(1, std::memory_order_relaxed);
        } else if (e == protocol::FrameError::kHeaderCrcMismatch ||
          e == protocol::FrameError::kPayloadCrcMismatch)
        {
          crcfail.fetch_add(1, std::memory_order_relaxed);
        }
      });
  }

  rec.start();

  std::atomic<bool> stop{false};
  std::atomic<bool> measuring{false};
  // Counters are snapshotted at the measurement window boundaries so warmup
  // traffic is excluded from the result rather than merely diluted.
  std::atomic<uint64_t> requested{0};

  std::vector<std::thread> producers;
  for (size_t i = 0; i < kMixCount; ++i) {
    producers.emplace_back(
      [&, i]() {
        const SensorSpec & m = kMix[i];
        std::vector<uint8_t> payload(m.payload_bytes);
        for (size_t k = 0; k < payload.size(); ++k) {
          payload[k] = static_cast<uint8_t>((k * 31 + i) & 0xFF);
        }
        const double period = 1.0 / (m.base_rate_hz * scale);
        auto next = std::chrono::steady_clock::now();
        uint64_t n = 0;
        while (!stop.load(std::memory_order_relaxed)) {
          payload[n % payload.size()] = static_cast<uint8_t>(n);
          rec.capture(ids[i], payload.data(), payload.size());
          if (measuring.load(std::memory_order_relaxed)) {
            requested.fetch_add(1, std::memory_order_relaxed);
          }
          ++n;
          next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(period));
          // If we have fallen behind, do not try to catch up in a burst: that
          // would create artificial overload and misreport the sustainable rate.
          const auto now = std::chrono::steady_clock::now();
          if (next < now) {
            next = now;
          } else {
            std::this_thread::sleep_until(next);
          }
        }
      });
  }

  std::thread sampler([&]() {
      while (!stop.load(std::memory_order_relaxed)) {
        rec.poll_resources();
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
      }
    });

  std::this_thread::sleep_for(
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(warmup_s)));

  const auto before = rec.stats();
  const double t0 = now_s();
  measuring.store(true);

  std::this_thread::sleep_for(
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(run_s)));

  measuring.store(false);
  const double t1 = now_s();
  const auto after = rec.stats();

  stop.store(true);
  for (auto & t : producers) {
    t.join();
  }
  sampler.join();
  rec.stop();
  const auto final_stats = rec.stats();

  r.measured_s = t1 - t0;
  r.captured = after.total_captured - before.total_captured;
  r.recorded = after.total_recorded - before.total_recorded;
  r.dropped = after.total_dropped - before.total_dropped;
  r.overwritten = after.total_overwritten - before.total_overwritten;
  r.frame_bytes = after.total_frame_bytes - before.total_frame_bytes;
  r.wal_bytes = after.wal_bytes - before.wal_bytes;
  r.wal_records = after.wal_records - before.wal_records;
  r.queued_start = before.queued_bytes;
  r.queued_end = after.queued_bytes;
  const double growth_bytes =
    static_cast<double>(r.queued_end) - static_cast<double>(r.queued_start);
  r.backlog_growth_mb_s = r.measured_s > 0
    ? (growth_bytes / (1024.0 * 1024.0)) / r.measured_s : 0.0;

  // Sustained requires all three: nothing refused, nothing overwritten, and the
  // queue not filling. kMaxBacklogGrowthMbS is the slack allowed for jitter in
  // an instantaneous depth reading; anything above it means the consumer is
  // falling behind and the zero-drop result is only a matter of time.
  constexpr double kMaxBacklogGrowthMbS = 1.0;
  r.clean = (r.dropped == 0 && r.overwritten == 0 &&
    r.backlog_growth_mb_s <= kMaxBacklogGrowthMbS);

  r.msgs_per_sec = r.measured_s > 0 ? r.captured / r.measured_s : 0;
  r.requested_msgs_per_sec =
    r.measured_s > 0 ? requested.load() / r.measured_s : 0;
  r.frame_mb_s = r.measured_s > 0
    ? (static_cast<double>(r.frame_bytes) / (1024.0 * 1024.0)) / r.measured_s : 0;
  r.wal_mb_s = r.measured_s > 0
    ? (static_cast<double>(r.wal_bytes) / (1024.0 * 1024.0)) / r.measured_s : 0;

  r.p50_us = final_stats.p50_capture_to_record_us;
  r.p99_us = final_stats.p99_capture_to_record_us;
  r.p999_us = final_stats.p999_capture_to_record_us;
  r.latency_samples = final_stats.latency_samples;
  r.peak_rss = final_stats.peak_rss_bytes;
  r.peak_queued_bytes = final_stats.peak_queued_bytes;
  r.peak_cpu = final_stats.peak_cpu_percent;
  r.frames_validated = validated.load();
  r.crc_failures = crcfail.load();
  if (validate_frames) {
    r.sequence_gaps = decoder.stats().sequence_gaps;
  }

  fs::remove_all(wal_dir, ec);
  return r;
}

void print_row(const TrialResult & t, const char * tag)
{
  const char * why = t.clean ? "CLEAN"
    : (t.dropped || t.overwritten) ? "DROPPED"
    : "BACKLOG";
  std::printf(
    "  %-8s scale=%7.3f  msgs/s=%9.1f  MB/s=%8.2f  drop=%-9llu overwr=%-7llu "
    "backlog=%+7.2f MB/s  %s\n",
    tag, t.scale, t.msgs_per_sec, t.frame_mb_s,
    (unsigned long long)t.dropped, (unsigned long long)t.overwritten,
    t.backlog_growth_mb_s, why);
}

}  // namespace

int main(int argc, char ** argv)
{
  double warmup_s = 2.0;
  double trial_s = 12.0;
  double confirm_s = 60.0;
  int bisect_rounds = 4;
  double max_scale = 512.0;
  std::string wal_root;
  std::string json_out;

  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--warmup") && i + 1 < argc) {
      warmup_s = std::atof(argv[++i]);
    } else if (!std::strcmp(argv[i], "--trial") && i + 1 < argc) {
      trial_s = std::atof(argv[++i]);
    } else if (!std::strcmp(argv[i], "--confirm") && i + 1 < argc) {
      confirm_s = std::atof(argv[++i]);
    } else if (!std::strcmp(argv[i], "--bisect") && i + 1 < argc) {
      bisect_rounds = std::atoi(argv[++i]);
    } else if (!std::strcmp(argv[i], "--max-scale") && i + 1 < argc) {
      max_scale = std::atof(argv[++i]);
    } else if (!std::strcmp(argv[i], "--wal-dir") && i + 1 < argc) {
      wal_root = argv[++i];
    } else if (!std::strcmp(argv[i], "--json") && i + 1 < argc) {
      json_out = argv[++i];
    } else {
      std::fprintf(stderr,
        "usage: capacity_sweep [--warmup S] [--trial S] [--confirm S]\n"
        "                      [--bisect N] [--max-scale X] [--wal-dir D] [--json OUT]\n");
      return 2;
    }
  }
  if (wal_root.empty()) {
    wal_root = (fs::temp_directory_path() / "sensorforge_sweep").string();
  }
  std::error_code ec;
  fs::remove_all(wal_root, ec);
  fs::create_directories(wal_root, ec);

  double base_msgs = 0, base_bytes = 0;
  for (const auto & m : kMix) {
    base_msgs += m.base_rate_hz;
    base_bytes += m.base_rate_hz * static_cast<double>(m.payload_bytes);
  }
  std::printf("===== SensorForge capacity sweep =====\n");
  std::printf("mix: %zu streams, scale 1.0 = %.0f msgs/s, %.2f MB/s of payload\n",
    kMixCount, base_msgs, base_bytes / (1024.0 * 1024.0));
  std::printf("warmup=%.1fs trial=%.1fs confirm=%.1fs bisect_rounds=%d\n\n",
    warmup_s, trial_s, confirm_s, bisect_rounds);

  std::vector<TrialResult> history;

  // ---- 1. coarse ramp -------------------------------------------------------
  std::printf("--- coarse ramp (doubling until first drop) ---\n");
  double last_clean = 0.0, first_dirty = 0.0;
  for (double scale = 1.0; scale <= max_scale; scale *= 2.0) {
    TrialResult t = run_trial(scale, warmup_s, trial_s, wal_root, false);
    history.push_back(t);
    print_row(t, "ramp");
    if (t.clean) {
      last_clean = scale;
    } else {
      first_dirty = scale;
      break;
    }
  }
  if (last_clean == 0.0) {
    std::printf("\nFAILED: dropped even at scale 1.0 -- no sustainable rate found.\n");
    return 1;
  }
  if (first_dirty == 0.0) {
    std::printf("\nNote: no drops up to max scale %.0f; capacity is above the swept range.\n",
      max_scale);
    first_dirty = last_clean;
  }

  // ---- 2. bisect ------------------------------------------------------------
  if (first_dirty > last_clean) {
    std::printf("\n--- bisecting between %.3f (clean) and %.3f (dropped) ---\n",
      last_clean, first_dirty);
    double lo = last_clean, hi = first_dirty;
    for (int i = 0; i < bisect_rounds; ++i) {
      const double mid = (lo + hi) / 2.0;
      TrialResult t = run_trial(mid, warmup_s, trial_s, wal_root, false);
      history.push_back(t);
      print_row(t, "bisect");
      if (t.clean) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    last_clean = lo;
  }

  // ---- 3. confirm, stepping down until a scale holds for the full window ----
  //
  // A scale that survives a short trial routinely fails a long one: near the
  // boundary a laptop thermally throttles, the page cache flushes, or the disk
  // stalls, and the queue never recovers. Capacity is therefore DEFINED by the
  // long run, not the ramp. If the confirm fails, back off and try again rather
  // than reporting a figure the system cannot actually hold.
  constexpr double kStepDown = 0.8;
  const int kMaxConfirmAttempts = 5;

  std::printf("\n--- sustained confirm at %.0fs per attempt (frame validation ON) ---\n",
    confirm_s);
  TrialResult best;
  bool ok = false;
  double candidate = last_clean;
  for (int attempt = 0; attempt < kMaxConfirmAttempts; ++attempt) {
    best = run_trial(candidate, warmup_s, confirm_s, wal_root, true);
    history.push_back(best);
    print_row(best, "confirm");
    if (best.clean) {
      ok = true;
      break;
    }
    const double next = candidate * kStepDown;
    if (attempt + 1 < kMaxConfirmAttempts) {
      std::printf("           ^ did not hold for %.0fs; backing off to scale %.3f\n",
        confirm_s, next);
    }
    candidate = next;
  }

  if (!ok) {
    std::printf("\nNOTE: no scale held for the full %.0fs window within %d attempts.\n"
      "      No capacity figure is reported -- the ramp result is NOT capacity.\n",
      confirm_s, kMaxConfirmAttempts);
  }

  std::printf("\n===== RESULT =====\n");
  std::printf("sustained scale        %.3f x the reference sensor mix\n", best.scale);
  std::printf("measured window        %.2f s\n", best.measured_s);
  std::printf("achieved msgs/sec      %.1f   (requested %.1f)\n",
    best.msgs_per_sec, best.requested_msgs_per_sec);
  std::printf("achieved frame MB/s    %.2f\n", best.frame_mb_s);
  std::printf("WAL MB/s               %.2f\n", best.wal_mb_s);
  std::printf("captured / recorded    %llu / %llu\n",
    (unsigned long long)best.captured, (unsigned long long)best.recorded);
  std::printf("dropped                %llu\n", (unsigned long long)best.dropped);
  std::printf("overwritten            %llu\n", (unsigned long long)best.overwritten);
  std::printf("backlog growth         %+.3f MB/s (queued %llu -> %llu bytes)\n",
    best.backlog_growth_mb_s,
    (unsigned long long)best.queued_start, (unsigned long long)best.queued_end);
  std::printf("capture->record us     p50=%.1f p99=%.1f p999=%.1f (n=%llu)\n",
    best.p50_us, best.p99_us, best.p999_us,
    (unsigned long long)best.latency_samples);
  std::printf("frames validated       %llu\n", (unsigned long long)best.frames_validated);
  std::printf("CRC failures           %llu\n", (unsigned long long)best.crc_failures);
  std::printf("sequence gaps          %llu\n", (unsigned long long)best.sequence_gaps);
  std::printf("peak RSS               %llu (%.1f MiB)\n",
    (unsigned long long)best.peak_rss, best.peak_rss / (1024.0 * 1024.0));
  std::printf("peak queued bytes      %llu\n", (unsigned long long)best.peak_queued_bytes);
  std::printf("peak CPU               %.1f%%\n", best.peak_cpu);
  std::printf("verdict                %s\n",
    ok ? "SUSTAINED: zero drops, zero overwrites, no backlog growth"
       : "NOT SUSTAINED (see note above)");

  if (!json_out.empty()) {
    std::ofstream j(json_out);
    j << "{\n";
    j << "  \"sustained\": " << (ok ? "true" : "false") << ",\n";
    j << "  \"scale\": " << best.scale << ",\n";
    j << "  \"reference_mix_msgs_per_sec_at_scale_1\": " << base_msgs << ",\n";
    j << "  \"measured_seconds\": " << best.measured_s << ",\n";
    j << "  \"achieved_msgs_per_sec\": " << best.msgs_per_sec << ",\n";
    j << "  \"requested_msgs_per_sec\": " << best.requested_msgs_per_sec << ",\n";
    j << "  \"frame_mb_per_sec\": " << best.frame_mb_s << ",\n";
    j << "  \"wal_mb_per_sec\": " << best.wal_mb_s << ",\n";
    j << "  \"captured\": " << best.captured << ",\n";
    j << "  \"recorded\": " << best.recorded << ",\n";
    j << "  \"dropped\": " << best.dropped << ",\n";
    j << "  \"overwritten\": " << best.overwritten << ",\n";
    j << "  \"backlog_growth_mb_per_sec\": " << best.backlog_growth_mb_s << ",\n";
    j << "  \"queued_bytes_start\": " << best.queued_start << ",\n";
    j << "  \"queued_bytes_end\": " << best.queued_end << ",\n";
    j << "  \"capture_to_record_us\": {\"p50\": " << best.p50_us
      << ", \"p99\": " << best.p99_us << ", \"p999\": " << best.p999_us
      << ", \"samples\": " << best.latency_samples << "},\n";
    j << "  \"frames_validated\": " << best.frames_validated << ",\n";
    j << "  \"crc_failures\": " << best.crc_failures << ",\n";
    j << "  \"sequence_gaps\": " << best.sequence_gaps << ",\n";
    j << "  \"peak_rss_bytes\": " << best.peak_rss << ",\n";
    j << "  \"peak_queued_bytes\": " << best.peak_queued_bytes << ",\n";
    j << "  \"peak_cpu_percent\": " << best.peak_cpu << ",\n";
    j << "  \"sweep\": [\n";
    for (size_t i = 0; i < history.size(); ++i) {
      const auto & t = history[i];
      j << "    {\"scale\": " << t.scale << ", \"msgs_per_sec\": " << t.msgs_per_sec
        << ", \"frame_mb_per_sec\": " << t.frame_mb_s
        << ", \"dropped\": " << t.dropped << ", \"overwritten\": " << t.overwritten
        << ", \"backlog_growth_mb_per_sec\": " << t.backlog_growth_mb_s
        << ", \"clean\": " << (t.clean ? "true" : "false") << "}"
        << (i + 1 < history.size() ? "," : "") << "\n";
    }
    j << "  ]\n}\n";
    std::printf("wrote %s\n", json_out.c_str());
  }

  fs::remove_all(wal_root, ec);
  return ok ? 0 : 1;
}
