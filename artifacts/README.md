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
- **QNX**: no SDP or toolchain present. Not built, not tested, not claimed.
