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
- **QNX**: partially closed. SDP 8.0 was installed and licensed, and the
  portable core CROSS-COMPILES cleanly for x86_64 QNX on BOTH macOS/Docker and a
  native x86_64 Linux host (see `qnx/`). No test was EXECUTED on a QNX target:
  five boot attempts across two host architectures and two image types (custom
  IFS and QNX's own unmodified `mkqnximage` image) all load the kernel and then
  emit nothing. Cross-ISA emulation and the custom IFS are ruled out; the actual
  cause is still open, with missing `/dev/kvm` the leading hypothesis rather
  than a demonstrated finding. All attempts and the alternatives not excluded
  are recorded in `qnx/qnx_boot_attempts.txt`. "Cross-compiles for QNX 8.0" is
  supported by these artifacts; "runs on QNX" is not.
- **AWS**: one `t3.large` (us-east-1, Ubuntu 22.04) was launched to reproduce
  the QNX cross-build on native x86_64 and to attempt a boot; it ran roughly 35
  minutes. Bare metal, needed to test the KVM hypothesis, could not be launched
  -- the account vCPU quota is 16 and x86 bare-metal types need 48-96. The
  instance, its security group, its key pair and its volume were all terminated
  and verified gone; no bare-metal instance was ever created.
- **CAN through ROS2**: not achieved on any host tried. Docker Desktop's
  LinuxKit VM has no `vcan` module, and a `--privileged` CI container did not
  resolve it either -- the GitHub run's own annotations read
  `e2e vcan0=unavailable`. The end-to-end run therefore covers four DDS sensors
  and records the CAN stream as unavailable rather than claiming it.
