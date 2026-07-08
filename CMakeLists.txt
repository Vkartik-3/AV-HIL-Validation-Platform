cmake_minimum_required(VERSION 3.8)
project(network_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZSTD REQUIRED libzstd)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)

  # The SensorForge subsystem (protocol/core, benchmarks, stress, fuzz) follows
  # its own clang-format style and is linted in the dedicated SensorForge CI
  # workflow, not the upstream ROS ament uncrustify profile. Exclude uncrustify
  # from the auto-run, then invoke it manually on the original ROS sources so
  # those (including our edits to them) stay style-gated.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_uncrustify REQUIRED)
  ament_uncrustify(
    EXCLUDE
      bench
      stress
      include/sensorforge
      src/protocol
  )

  # SensorForge GoogleTest unit suite (Extension G). Non-ROS; links the core
  # protocol/replay library. Reports its executed-assertion count as
  # "[ASSERTIONS] total=..." (must be >= 5000).
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(sensorforge_unit_tests
    test/unit/test_main.cpp
    test/unit/test_crc32c.cpp
    test/unit/test_frame_codec.cpp
    test/unit/test_spsc_ring.cpp
    test/unit/test_seqlock.cpp
    test/unit/test_wal.cpp
    test/unit/test_backpressure.cpp
    test/unit/test_tcp_reassembler.cpp
  )
  if(TARGET sensorforge_unit_tests)
    target_include_directories(sensorforge_unit_tests PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/test/unit
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(sensorforge_unit_tests sensorforge_protocol)
  endif()

  # NOTE: the internal launch tests wait up to 10s (spin_until_future_complete)
  # for a round trip; the old 2s CMake TIMEOUT killed the first-run test during
  # cold DDS discovery (order-dependent flake). Give discovery adequate room.
  add_launch_test(
    test/test_udp.py
    TIMEOUT 60  # Sets a timeout for the test in seconds
  )
  add_launch_test(
    test/test_tcp.py
    TIMEOUT 60  # Sets a timeout for the test in seconds
  )
endif()

include_directories(include)

# ---------------------------------------------------------------------------
# SensorForge core/protocol library (no ROS2 dependency).
# Kept deliberately free of rclcpp/Boost so it can also be built by the plain
# CMake CI job, the Google Benchmark harness, the GoogleTest suite and the
# libFuzzer targets without a ROS2 environment.
# ---------------------------------------------------------------------------
add_library(sensorforge_protocol STATIC
  src/protocol/frame_codec.cpp
  src/replay/wal_writer.cpp
  src/replay/wal_reader.cpp
)
target_include_directories(sensorforge_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(sensorforge_protocol PUBLIC cxx_std_20)
set_property(TARGET sensorforge_protocol PROPERTY POSITION_INDEPENDENT_CODE ON)

# Optional: build the Google Benchmark harness (bench/). Off by default so the
# standard ament/colcon build does not require the benchmark package.
option(SENSORFORGE_BUILD_BENCH "Build the SensorForge Google Benchmark harness" OFF)
if(SENSORFORGE_BUILD_BENCH)
  add_subdirectory(bench)
endif()

# Optional: build the concurrency stress test (Extension A). Intended to run
# under -fsanitize=thread in the TSan CI job.
option(SENSORFORGE_BUILD_FUZZ "Build the SensorForge libFuzzer harnesses (Clang only)" OFF)
if(SENSORFORGE_BUILD_FUZZ)
  add_subdirectory(fuzz)
endif()

option(SENSORFORGE_BUILD_STRESS "Build the SensorForge SPSC/seqlock stress test" OFF)
if(SENSORFORGE_BUILD_STRESS)
  add_executable(spsc_tsan_stress stress/spsc_tsan_stress.cpp)
  target_link_libraries(spsc_tsan_stress PRIVATE sensorforge_protocol)
  target_compile_features(spsc_tsan_stress PRIVATE cxx_std_20)
  find_package(Threads REQUIRED)
  target_link_libraries(spsc_tsan_stress PRIVATE Threads::Threads)
endif()

add_executable(network_bridge
    src/network_bridge.cpp
    src/subscription_manager.cpp
    src/subscription_manager_tf.cpp
)

add_library(udp_interface SHARED
  src/network_interfaces/udp_interface.cpp
)

add_library(tcp_interface SHARED
  src/network_interfaces/tcp_interface.cpp
)

target_link_libraries(network_bridge PUBLIC
  ${std_msgs_TARGETS}
  ${tf2_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  ${ZSTD_LIBRARIES}
  sensorforge_protocol
)

target_link_libraries(udp_interface PUBLIC
  pluginlib::pluginlib
  rclcpp::rclcpp
  ${Boost_LIBRARIES}
)

target_link_libraries(tcp_interface PUBLIC
  pluginlib::pluginlib
  rclcpp::rclcpp
  ${Boost_LIBRARIES}
)

# ---------------------------------------------------------------------------
# Synthetic AV sensor publishers (Extension I). Each is a standalone ROS2 node.
# ---------------------------------------------------------------------------
foreach(sensor lidar camera imu gps can)
  add_executable(${sensor}_publisher sensors/${sensor}_publisher.cpp)
  target_include_directories(${sensor}_publisher PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/sensors)
  target_link_libraries(${sensor}_publisher PUBLIC
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS})
endforeach()

install(TARGETS
  lidar_publisher camera_publisher imu_publisher gps_publisher can_publisher
  DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(network_bridge network_interface_plugins.xml)

install(TARGETS
  network_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  udp_interface
  tcp_interface
  DESTINATION lib/
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
