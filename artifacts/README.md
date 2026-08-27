# Measured artifacts

Every file here was produced at run time by the commands listed below. Nothing
is transcribed by hand.

## Environment

```
Host      Apple M-series (arm64), 8 logical cores, macOS 15.6 (Darwin 25.6.0)
Compiler  Apple clang (CommandLineTools), -std=c++20
Build     standalone/CMakeLists.txt  (ROS-free core)
          Debug -O0 -g          for the unit suite
          Release -O3 -DNDEBUG  for e2e_reference
          -fsanitize=thread -O1 for spsc_tsan_stress
          -fsanitize=address,undefined -O1 for the ASan/UBSan run
CRC path  ARMv8 CRC32 hardware instructions (__crc32c*)
Storage   APFS on internal NVMe
Date      2026-08-27
```

**These are latency MEASUREMENTS, not latency guarantees.** SensorForge is a
performance-sensitive recording pipeline, not a hard real-time system: no
`SCHED_FIFO`, no CPU pinning, no `mlockall`, five heap allocations per message
on the bridge path, and a best-effort ROS2 wall timer as the consumer. The
percentiles below describe typical behaviour on an ordinary kernel under no
contention. What the system actually bounds is queue depth, queued bytes and
memory -- and those bounds are asserted by the tests.

**This is a developer laptop, not a controlled benchmark host.** Numbers are
reproducible in kind, not to the digit. No CPU pinning, no governor control,
other processes present.

## Files

| File | Produced by |
|---|---|
| `unit_tests.txt` | `./build/sensorforge_unit_tests` (Debug) |
| `asan_ubsan.txt` | ASan+UBSan build of the same suite |
| `tsan_stress.txt` | `SENSORFORGE_STRESS_SECONDS=30 ./spsc_tsan_stress` (TSan) |
| `e2e_paced.txt`, `e2e_reference_paced.json` | `./e2e_reference --duration 30` |
| `e2e_saturate.txt`, `e2e_reference_saturate.json` | `./e2e_reference --duration 15 --saturate` |
| `fsync_cost.txt` | `./e2e_reference --duration 10 --fsync <policy>` x4 |
| `offload_measurements.txt`, `e2e_offload_*.json` | `./e2e_reference --offload-dir D [--offload-down]` |
| `replay_run1.txt`, `replay_run2.txt` | `./wal_replay --dir D --digest --verify` x2 |
| `fuzz_smoke.txt` | compile-only; see note below |

## Linux / ROS2 artifacts

| Directory | Contents |
|---|---|
| `linux_ros2_e2e/` | Full ROS2 five-sensor run: both bridges' Prometheus scrapes, replay validation (x2), per-publisher logs, republished topic list, IMU rate, GPS echo, duration, WAL size, and the exact configs used. |
| `rosbag2_ingestion/` | rosbag2 record -> replay-through-bridge -> WAL validation. |

Produced inside `ros:humble-ros-base` (arm64) on Docker Desktop, and gated in CI
by the `ros-humble` job on x86_64 Ubuntu. Command:
`test/e2e/run_ros2_e2e.sh <install> <config-dir> <out-dir> 45`

## Not measured here

- **libFuzzer smoke**: Apple's CommandLineTools clang ships no
  `libclang_rt.fuzzer_osx.a`, so the four harnesses compile but do not link
  locally. They are built and run by the `fuzz-smoke` CI job on Ubuntu clang.
  No local fuzzing result is claimed.
- **Anything requiring ROS2**: no ROS2 distribution exists on this host, so the
  bridge node, the launch tests, the scenario runner and the sensor publisher
  executables were not built or run here. Those are covered by the `ros-humble`
  CI job. The end-to-end workload exercises the SensorForge core directly
  (producer threads -> buffer -> frame -> CRC -> WAL -> replay) and deliberately
  does NOT claim to measure DDS.
- **QNX**: QNX Software Center 2.0.4 was downloaded, but that is the package
  manager, not the SDP. The SDP itself requires an authenticated myQNX download
  which was not completed, so no toolchain, target or test result exists. Not
  built, not tested, not claimed.
- **AWS**: no EC2 instance was created. The configured credentials were invalid
  (`InvalidClientTokenId`), and Linux/ROS2 was already covered by CI (x86_64)
  and Docker (arm64), so no paid resource was launched. Nothing to terminate.
- **CAN through ROS2**: not achieved on any host tried. Docker Desktop's
  LinuxKit VM has no `vcan` module, and a `--privileged` CI container did not
  resolve it either -- the GitHub run's own annotations read
  `e2e vcan0=unavailable`. The end-to-end run therefore covers four DDS sensors
  and records the CAN stream as unavailable rather than claiming it.
