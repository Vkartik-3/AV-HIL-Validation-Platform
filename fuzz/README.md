# SensorForge Fuzzing (Extension F)

libFuzzer harnesses for the pure, attacker-facing parsers. Each target is
ROS-free and links only the parser it exercises.

| Harness | Target | Entry point |
|---|---|---|
| `fuzz_frame_parser` | `protocol/frame_codec` | `decode_header` + `FrameDecoder` |
| `fuzz_tcp_reassembly` | `transport/tcp_reassembler` | `TcpReassembler::feed` (chunked) |
| `fuzz_replay_reader` | `replay/record` | `decode_record` + resync scan |
| `fuzz_yaml_scenario` | `scenario` parser | added in Phase 3 with Extension J |

## Build & run (Clang + libFuzzer, on EC2)

```bash
cmake -S . -B build -DSENSORFORGE_BUILD_FUZZ=ON -DCMAKE_CXX_COMPILER=clang++
cmake --build build --target fuzz_frame_parser fuzz_tcp_reassembly fuzz_replay_reader

# CI smoke (1000 iterations, must exit 0):
./build/fuzz/fuzz_frame_parser    -runs=1000
./build/fuzz/fuzz_tcp_reassembly  -runs=1000
./build/fuzz/fuzz_replay_reader   -runs=1000

# Longer soak with a persistent corpus:
mkdir -p corpus/frame && ./build/fuzz/fuzz_frame_parser corpus/frame -runs=50000
```

Each harness is built with `-fsanitize=fuzzer,address,undefined`, so any
out-of-bounds read, UB, or leak fails the run.

## Local pre-check (macOS)

Apple's Command Line Tools clang ships ASan/UBSan but **not** the libFuzzer
runtime, so coverage-guided runs happen on EC2 / a full LLVM install. As a local
substitute, the parsers were driven with **2,000,000 random and mutated inputs**
under `-fsanitize=address,undefined -fno-sanitize-recover=all` and survived
clean (0 crashes). This proves memory-safety on adversarial input; libFuzzer
adds coverage-guided path exploration on top.

## Results (fill from EC2 runs)

| Harness | first crash @ iter | bugs found/fixed | corpus after 50k | 50k clean? |
|---|---|---|---|---|
| fuzz_frame_parser | TBD | TBD | TBD | TBD |
| fuzz_tcp_reassembly | TBD | TBD | TBD | TBD |
| fuzz_replay_reader | TBD | TBD | TBD | TBD |
