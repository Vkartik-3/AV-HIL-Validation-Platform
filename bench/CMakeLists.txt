# ---------------------------------------------------------------------------
# SensorForge Google Benchmark harness (Extension C)
# Enabled from the top-level CMakeLists via -DSENSORFORGE_BUILD_BENCH=ON.
# Requires the Google Benchmark package (libbenchmark-dev / find_package).
# No ROS2 dependency: links only sensorforge_protocol.
# ---------------------------------------------------------------------------
find_package(benchmark REQUIRED)
find_package(Threads REQUIRED)

add_executable(sensorforge_bench
  bench_main.cpp
  bench_crc32c.cpp
  bench_frame.cpp
  bench_ring.cpp
  bench_e2e.cpp
)

# Allow "#include \"bench/bench_common.hpp\"" and "sensorforge/..." includes.
target_include_directories(sensorforge_bench PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/..
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_compile_features(sensorforge_bench PRIVATE cxx_std_20)
target_link_libraries(sensorforge_bench PRIVATE
  sensorforge_protocol
  benchmark::benchmark
  Threads::Threads
)
