/*
==============================================================================
SensorForge - libFuzzer harness: WAL replay record parser (Extension F)
Part of the SensorForge AV HIL validation platform.

Feeds arbitrary bytes to the pure decode_record() and also drives a
multi-record resync scan (the same recovery loop the WAL reader uses), so both
single-record validation and the corruption-recovery walk are fuzzed. Must
never crash, over-allocate, or read out of bounds.

Build/run (clang):
  clang++ -std=c++20 -g -O1 -fsanitize=fuzzer,address,undefined -Iinclude \
      fuzz/fuzz_replay_reader.cpp -o fuzz_replay
  ./fuzz_replay -runs=1000000
==============================================================================
*/

#include <cstddef>
#include <cstdint>

#include "sensorforge/replay/record.hpp"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t * data, size_t size)
{
  using namespace sensorforge::replay;

  // 1. Single-record decode at offset 0.
  RecordHeader hdr;
  const RecordError err = decode_record(data, size, hdr);
  if (err == RecordError::kOk) {
    if (static_cast<size_t>(hdr.record_length) > size) {
      __builtin_trap();  // decoder claimed more bytes than exist
    }
    const uint8_t * p = record_payload(data);
    volatile uint8_t sink = 0;
    for (uint32_t i = 0; i < hdr.payload_size; ++i) {
      sink = static_cast<uint8_t>(sink ^ p[i]);
    }
    (void)sink;
  }

  // 2. Full corruption-recovery scan (mirrors WalReader::load resync).
  size_t pos = 0;
  uint64_t guard = 0;
  while (pos + kRecordHeaderSize <= size && guard++ < (1u << 20)) {
    RecordHeader h;
    if (decode_record(data + pos, size - pos, h) == RecordError::kOk) {
      pos += h.record_length;
    } else {
      ++pos;
    }
  }
  return 0;
}
