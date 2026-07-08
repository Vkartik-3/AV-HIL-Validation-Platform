/*
==============================================================================
SensorForge - libFuzzer harness: YAML scenario parser (Extension F/J)
Part of the SensorForge AV HIL validation platform.

Feeds arbitrary bytes to parse_scenario_string(). The parser must reject any
malformed input by throwing (which we catch) rather than crashing, reading out
of bounds, or leaking.

Build/run (clang):
  clang++ -std=c++20 -g -O1 -fsanitize=fuzzer,address,undefined -I. -Iinclude \
      fuzz/fuzz_yaml_scenario.cpp scenario/scenario.cpp -lyaml-cpp -o fuzz_yaml
  ./fuzz_yaml -runs=1000
==============================================================================
*/

#include <cstddef>
#include <cstdint>
#include <string>

#include "scenario/scenario.hpp"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t * data, size_t size)
{
  const std::string yaml(reinterpret_cast<const char *>(data), size);
  try {
    const auto scenario = sensorforge::scenario::parse_scenario_string(yaml);
    (void)scenario;
  } catch (const std::exception &) {
    // Expected for malformed input.
  } catch (...) {
    // yaml-cpp may throw its own types; also acceptable (not a crash).
  }
  return 0;
}
