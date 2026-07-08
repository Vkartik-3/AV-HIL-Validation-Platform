# ---------------------------------------------------------------------------
# SensorForge libFuzzer harnesses (Extension F)
# Enabled with -DSENSORFORGE_BUILD_FUZZ=ON and a Clang compiler (libFuzzer).
# Each target links the pure, ROS-free parser it exercises.
#
#   fuzz_frame_parser     -> protocol/frame_codec  (frame header parser)
#   fuzz_tcp_reassembly   -> transport/tcp_reassembler (fragment reassembly)
#   fuzz_replay_reader    -> replay/record         (WAL record parser)
#   fuzz_yaml_scenario    -> added in Phase 3 with Extension J (scenario parser)
# ---------------------------------------------------------------------------
if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  message(WARNING "SENSORFORGE_BUILD_FUZZ requires Clang for libFuzzer; skipping fuzz targets")
  return()
endif()

set(SF_FUZZ_FLAGS -fsanitize=fuzzer,address,undefined -g -O1)

function(sf_add_fuzzer name)
  add_executable(${name} ${ARGN})
  target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)
  target_compile_features(${name} PRIVATE cxx_std_20)
  target_compile_options(${name} PRIVATE ${SF_FUZZ_FLAGS})
  target_link_options(${name} PRIVATE ${SF_FUZZ_FLAGS})
endfunction()

sf_add_fuzzer(fuzz_frame_parser
  fuzz_frame_parser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/protocol/frame_codec.cpp)

sf_add_fuzzer(fuzz_tcp_reassembly
  fuzz_tcp_reassembly.cpp)

sf_add_fuzzer(fuzz_replay_reader
  fuzz_replay_reader.cpp)
