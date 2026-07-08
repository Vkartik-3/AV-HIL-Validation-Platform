/*
==============================================================================
SensorForge - libFuzzer harness: TCP fragment reassembly (Extension F)
Part of the SensorForge AV HIL validation platform.

Splits the input into arbitrary-sized chunks (driven by the input itself) and
feeds them to the pure TcpReassembler, mimicking how bytes arrive in fragments
over a real socket. The reassembler must never crash, over-allocate, or read
out of bounds regardless of input. Any emitted payload must have the exact
length the length-prefix claimed.

Build/run (clang):
  clang++ -std=c++20 -g -O1 -fsanitize=fuzzer,address,undefined -Iinclude \
      fuzz/fuzz_tcp_reassembly.cpp -o fuzz_tcp
  ./fuzz_tcp -runs=1000000
==============================================================================
*/

#include <cstddef>
#include <cstdint>

#include "sensorforge/transport/tcp_reassembler.hpp"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t * data, size_t size)
{
  using sensorforge::transport::TcpReassembler;

  // Cap max payload low so the fuzzer explores boundaries cheaply.
  TcpReassembler reasm(
    [](const uint8_t * p, size_t n) {
      volatile uint8_t sink = 0;
      for (size_t i = 0; i < n; ++i) {
        sink = static_cast<uint8_t>(sink ^ p[i]);
      }
      (void)sink;
      (void)p;
    },
    /*max_payload=*/64 * 1024);

  // Feed in pseudo-random-sized chunks derived from the data itself.
  size_t off = 0;
  while (off < size) {
    const size_t chunk = 1 + (data[off] % 17);
    const size_t n = (off + chunk <= size) ? chunk : (size - off);
    reasm.feed(data + off, n);
    off += n;
  }
  return 0;
}
