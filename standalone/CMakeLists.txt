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
option(SENSORFORGE_BUILD_STRESS "Build the SPSC/seqlock stress test"     ON)
option(SENSORFORGE_BUILD_BENCH  "Build the Google Benchmark harness"     OFF)
option(SENSORFORGE_BUILD_FUZZ   "Build the libFuzzer harnesses (Clang)"  OFF)

# Core library (protocol + replay). Header-only pieces need no compilation.
add_library(sensorforge_protocol STATIC
  ${SF_ROOT}/src/protocol/frame_codec.cpp
  ${SF_ROOT}/src/replay/wal_writer.cpp
  ${SF_ROOT}/src/replay/wal_reader.cpp
)
target_compile_features(sensorforge_protocol PUBLIC cxx_std_20)

if(SENSORFORGE_BUILD_TESTS)
  find_package(GTest REQUIRED)
  find_package(yaml-cpp REQUIRED)
  enable_testing()
  # Non-ROS scenario library (parser + assertions) for the scenario tests.
  add_library(sensorforge_scenario STATIC
    ${SF_ROOT}/scenario/scenario.cpp
    ${SF_ROOT}/scenario/assertion.cpp)
  target_include_directories(sensorforge_scenario PUBLIC ${SF_ROOT})
  target_link_libraries(sensorforge_scenario PUBLIC yaml-cpp)
  # Fault engine (non-ROS).
  add_library(sensorforge_faults STATIC ${SF_ROOT}/faults/fault_engine.cpp)
  target_include_directories(sensorforge_faults PUBLIC ${SF_ROOT})
  target_link_libraries(sensorforge_faults PUBLIC Threads::Threads)
  # Report library (non-ROS).
  find_package(nlohmann_json REQUIRED)
  add_library(sensorforge_report STATIC
    ${SF_ROOT}/report/report_builder.cpp
    ${SF_ROOT}/report/json_report.cpp
    ${SF_ROOT}/report/html_report.cpp)
  target_include_directories(sensorforge_report PUBLIC ${SF_ROOT})
  target_link_libraries(sensorforge_report PUBLIC sensorforge_scenario nlohmann_json::nlohmann_json)
  add_executable(sensorforge_unit_tests
    ${SF_ROOT}/test/unit/test_main.cpp
    ${SF_ROOT}/test/unit/test_crc32c.cpp
    ${SF_ROOT}/test/unit/test_frame_codec.cpp
    ${SF_ROOT}/test/unit/test_spsc_ring.cpp
    ${SF_ROOT}/test/unit/test_seqlock.cpp
    ${SF_ROOT}/test/unit/test_wal.cpp
    ${SF_ROOT}/test/unit/test_backpressure.cpp
    ${SF_ROOT}/test/unit/test_tcp_reassembler.cpp
    ${SF_ROOT}/test/unit/test_scenario.cpp
    ${SF_ROOT}/test/unit/test_fault_injection.cpp
    ${SF_ROOT}/test/unit/test_report.cpp
    ${SF_ROOT}/test/unit/test_metrics.cpp
  )
  target_include_directories(sensorforge_unit_tests PRIVATE ${SF_ROOT}/test/unit ${SF_ROOT})
  target_link_libraries(sensorforge_unit_tests
    sensorforge_protocol sensorforge_scenario sensorforge_faults sensorforge_report
    GTest::gtest GTest::gtest_main Threads::Threads)
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
