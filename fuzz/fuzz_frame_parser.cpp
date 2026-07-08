/*
==============================================================================
SensorForge - libFuzzer harness: frame header parser (Extension F)
Part of the SensorForge AV HIL validation platform.

Feeds arbitrary bytes to the stateless decode_header() and the stateful
FrameDecoder. Both must handle any input without crashing, reading out of
bounds, or leaking. On a valid frame we additionally sanity-check that the
reported payload fits inside the buffer.

Build/run (clang):
  clang++ -std=c++20 -g -O1 -fsanitize=fuzzer,address,undefined -Iinclude \
      fuzz/fuzz_frame_parser.cpp src/protocol/frame_codec.cpp -o fuzz_frame
  ./fuzz_frame -runs=1000000
==============================================================================
*/

#include <cstddef>
#include <cstdint>

#include "sensorforge/protocol/frame_codec.hpp"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t * data, size_t size)
{
  using namespace sensorforge::protocol;

  FrameHeader hdr;
  const FrameError err = decode_header(data, size, hdr);
  if (err == FrameError::kOk) {
    // Invariant: a validated frame's payload must fit within the input.
    if (kFrameOverhead + static_cast<size_t>(hdr.payload_size) > size) {
      __builtin_trap();  // would indicate a bounds bug in the decoder
    }
    // Touch the payload bytes so ASan verifies the range is readable.
    const uint8_t * p = payload_ptr(data);
    volatile uint8_t sink = 0;
    for (uint32_t i = 0; i < hdr.payload_size; ++i) {
      sink = static_cast<uint8_t>(sink ^ p[i]);
    }
    (void)sink;
  }

  // Exercise the stateful decoder (sequence/timestamp regression tracking).
  static FrameDecoder decoder;
  FrameHeader hdr2;
  (void)decoder.decode(data, size, /*stream_key=*/0, hdr2);
  return 0;
}
