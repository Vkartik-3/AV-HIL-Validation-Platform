# ---------------------------------------------------------------------------
# SensorForge standalone (non-ROS) build.
#
# The SensorForge core (protocol, replay, ring, seqlock, reassembler) has no
# ROS2 dependency, so its tests / sanitizers / fuzzers / benchmarks build here
# on plain Ubuntu without ament/colcon. The ROS node itself is built by the
# top-level ament CMakeLists. Used by the SensorForge CI matrix.
#
#   cmake -S standalone -B build [-DSENSORFORGE_BUILD_*=ON]
# ---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)
project(sensorforge_standalone CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

get_filename_component(SF_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
include_directories(${SF_ROOT}/include)

find_package(Threads REQUIRED)

option(SENSORFORGE_BUILD_TESTS  "Build the GoogleTest unit suite"        ON)
# Build only the PORTABLE core: no yaml-cpp, no nlohmann_json, and therefore no
# scenario/report/parallel tests. Used for the QNX cross-build, where those two
# third-party libraries are not available and are not what is being validated --
# the portable core is. Everything else (protocol, replay, ring, seqlock,
# stream buffer, recorder, offload, trigger, policy, resource sampler) is built
# and tested exactly as on Linux/macOS.
option(SENSORFORGE_CORE_ONLY    "Build only the dependency-free portable core" OFF)
option(SENSORFORGE_BUILD_STRESS "Build the SPSC/seqlock stress test"     ON)
option(SENSORFORGE_BUILD_BENCH  "Build the Google Benchmark harness"     OFF)
option(SENSORFORGE_BUILD_FUZZ   "Build the libFuzzer harnesses (Clang)"  OFF)

# Core library (protocol + replay). Header-only pieces need no compilation.
add_library(sensorforge_protocol STATIC
  ${SF_ROOT}/src/protocol/frame_codec.cpp
  ${SF_ROOT}/src/replay/wal_writer.cpp
  ${SF_ROOT}/src/replay/wal_reader.cpp
  ${SF_ROOT}/src/core/resource_monitor.cpp
)
target_compile_features(sensorforge_protocol PUBLIC cxx_std_20)
target_link_libraries(sensorforge_protocol PUBLIC Threads::Threads)

# Recording pipeline (recorder + selective recording + policy) and offload.
# ROS-free, so the whole integrated path is testable in this standalone build.
add_library(sensorforge_pipeline STATIC
  ${SF_ROOT}/src/pipeline/recorder.cpp
  ${SF_ROOT}/src/offload/uploader.cpp
)
target_compile_features(sensorforge_pipeline PUBLIC cxx_std_20)
target_link_libraries(sensorforge_pipeline PUBLIC sensorforge_protocol Threads::Threads)

# wal_replay: the deterministic replay tool. The audit found WalReader was
# reachable from no shipped binary; this is that binary.
add_executable(wal_replay ${SF_ROOT}/src/replay/replay_main.cpp)
target_link_libraries(wal_replay PRIVATE sensorforge_protocol)

# End-to-end reference workload: drives the REAL path (buffer -> frame -> CRC
# -> WAL -> replay) with a realistic multi-sensor mix.
add_executable(e2e_reference ${SF_ROOT}/tools/e2e_reference.cpp)
target_link_libraries(e2e_reference PRIVATE sensorforge_pipeline)

# Capacity sweep: finds the highest multiple of the reference sensor mix the
# pipeline sustains with ZERO drops, then confirms it over a long window.
add_executable(capacity_sweep ${SF_ROOT}/tools/capacity_sweep.cpp)
target_link_libraries(capacity_sweep PRIVATE sensorforge_pipeline)

if(SENSORFORGE_BUILD_TESTS)
  find_package(GTest REQUIRED)
  enable_testing()
  # Fault engine: ROS-free and dependency-free, so it builds everywhere.
  add_library(sensorforge_faults STATIC ${SF_ROOT}/faults/fault_engine.cpp)
  target_include_directories(sensorforge_faults PUBLIC ${SF_ROOT})
  target_link_libraries(sensorforge_faults PUBLIC Threads::Threads)
  if(NOT SENSORFORGE_CORE_ONLY)
  find_package(yaml-cpp REQUIRED)
  # Non-ROS scenario library (parser + assertions) for the scenario tests.
  add_library(sensorforge_scenario STATIC
    ${SF_ROOT}/scenario/scenario.cpp
    ${SF_ROOT}/scenario/assertion.cpp)
  target_include_directories(sensorforge_scenario PUBLIC ${SF_ROOT})
  target_link_libraries(sensorforge_scenario PUBLIC yaml-cpp)
  # Report library (non-ROS).
  find_package(nlohmann_json REQUIRED)
  add_library(sensorforge_report STATIC
    ${SF_ROOT}/report/report_builder.cpp
    ${SF_ROOT}/report/json_report.cpp
    ${SF_ROOT}/report/html_report.cpp)
  target_include_directories(sensorforge_report PUBLIC ${SF_ROOT})
  target_link_libraries(sensorforge_report PUBLIC sensorforge_scenario nlohmann_json::nlohmann_json)
  endif()
  set(SF_CORE_TEST_SOURCES
    ${SF_ROOT}/test/unit/test_main.cpp
    ${SF_ROOT}/test/unit/test_crc32c.cpp
    ${SF_ROOT}/test/unit/test_frame_codec.cpp
    ${SF_ROOT}/test/unit/test_spsc_ring.cpp
    ${SF_ROOT}/test/unit/test_seqlock.cpp
    ${SF_ROOT}/test/unit/test_wal.cpp
    ${SF_ROOT}/test/unit/test_backpressure.cpp
    ${SF_ROOT}/test/unit/test_tcp_reassembler.cpp
    ${SF_ROOT}/test/unit/test_fault_injection.cpp
    ${SF_ROOT}/test/unit/test_metrics.cpp
    ${SF_ROOT}/test/unit/test_stream_buffer.cpp
    ${SF_ROOT}/test/unit/test_ring_overwrite.cpp
    ${SF_ROOT}/test/unit/test_wal_restart.cpp
    ${SF_ROOT}/test/unit/test_replay_digest.cpp
    ${SF_ROOT}/test/unit/test_sequence_integrity.cpp
    ${SF_ROOT}/test/unit/test_recorder.cpp
    ${SF_ROOT}/test/unit/test_recording_policy.cpp
    ${SF_ROOT}/test/unit/test_trigger.cpp
    ${SF_ROOT}/test/unit/test_offload.cpp
    ${SF_ROOT}/test/unit/test_resource_monitor.cpp
  )
  # These pull in yaml-cpp (scenario) or nlohmann_json (report).
  set(SF_DEPENDENT_TEST_SOURCES
    ${SF_ROOT}/test/unit/test_scenario.cpp
    ${SF_ROOT}/test/unit/test_report.cpp
    ${SF_ROOT}/test/unit/test_parallel.cpp
  )
  if(SENSORFORGE_CORE_ONLY)
    add_executable(sensorforge_unit_tests ${SF_CORE_TEST_SOURCES})
  else()
    add_executable(sensorforge_unit_tests
      ${SF_CORE_TEST_SOURCES} ${SF_DEPENDENT_TEST_SOURCES})
  endif()
  target_include_directories(sensorforge_unit_tests PRIVATE ${SF_ROOT}/test/unit ${SF_ROOT})
  if(SENSORFORGE_CORE_ONLY)
    target_link_libraries(sensorforge_unit_tests
      sensorforge_protocol sensorforge_pipeline sensorforge_faults
      GTest::gtest GTest::gtest_main Threads::Threads)
  else()
    target_link_libraries(sensorforge_unit_tests
      sensorforge_protocol sensorforge_pipeline
      sensorforge_scenario sensorforge_faults sensorforge_report
      GTest::gtest GTest::gtest_main Threads::Threads)
  endif()
  add_test(NAME unit COMMAND sensorforge_unit_tests)
endif()

if(SENSORFORGE_BUILD_STRESS)
  add_executable(spsc_tsan_stress ${SF_ROOT}/stress/spsc_tsan_stress.cpp)
  target_link_libraries(spsc_tsan_stress sensorforge_protocol Threads::Threads)
endif()

if(SENSORFORGE_BUILD_BENCH)
  find_package(benchmark REQUIRED)
  add_executable(sensorforge_bench
    ${SF_ROOT}/bench/bench_main.cpp
    ${SF_ROOT}/bench/bench_crc32c.cpp
    ${SF_ROOT}/bench/bench_frame.cpp
    ${SF_ROOT}/bench/bench_ring.cpp
    ${SF_ROOT}/bench/bench_e2e.cpp
  )
  target_include_directories(sensorforge_bench PRIVATE ${SF_ROOT})
  target_link_libraries(sensorforge_bench
    sensorforge_protocol benchmark::benchmark Threads::Threads)
endif()

if(SENSORFORGE_BUILD_FUZZ)
  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    message(FATAL_ERROR "SENSORFORGE_BUILD_FUZZ requires Clang for libFuzzer")
  endif()
  set(SF_FUZZ_FLAGS -fsanitize=fuzzer,address,undefined -g -O1)
  function(sf_add_fuzzer name)
    add_executable(${name} ${SF_ROOT}/fuzz/${name}.cpp ${ARGN})
    target_compile_options(${name} PRIVATE ${SF_FUZZ_FLAGS})
    target_link_options(${name} PRIVATE ${SF_FUZZ_FLAGS})
  endfunction()
  sf_add_fuzzer(fuzz_frame_parser ${SF_ROOT}/src/protocol/frame_codec.cpp)
  sf_add_fuzzer(fuzz_tcp_reassembly)
  sf_add_fuzzer(fuzz_replay_reader)
  # YAML scenario parser fuzzer (needs yaml-cpp).
  find_package(yaml-cpp QUIET)
  if(yaml-cpp_FOUND OR TARGET yaml-cpp)
    add_executable(fuzz_yaml_scenario
      ${SF_ROOT}/fuzz/fuzz_yaml_scenario.cpp ${SF_ROOT}/scenario/scenario.cpp)
    target_include_directories(fuzz_yaml_scenario PRIVATE ${SF_ROOT})
    target_compile_options(fuzz_yaml_scenario PRIVATE ${SF_FUZZ_FLAGS})
    target_link_options(fuzz_yaml_scenario PRIVATE ${SF_FUZZ_FLAGS})
    target_link_libraries(fuzz_yaml_scenario yaml-cpp)
  endif()
endif()
