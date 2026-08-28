# =============================================================================
# SensorForge - QNX cross-compilation toolchain
#
# Builds the ROS-free SensorForge core for a QNX Neutrino target with qcc/q++
# from QNX SDP 8.0. Deliberately scoped: the ROS2 bridge, sensor publishers,
# scenario runner, SocketCAN, io_uring and the Linux /proc resource sampler are
# NOT built here. The point is to prove the portable core is portable, not to
# port the application.
#
# Requires the SDP environment (QNX_HOST / QNX_TARGET), normally from:
#   source $SDP_ROOT/qnxsdp-env.sh
#
# Usage:
#   cmake -S standalone -B build-qnx \
#         -DCMAKE_TOOLCHAIN_FILE=../cmake/qnx.toolchain.cmake \
#         -DQNX_ARCH=x86_64
#
# QNX_ARCH: x86_64 (default) or aarch64le.
# =============================================================================

set(CMAKE_SYSTEM_NAME QNX)
set(CMAKE_SYSTEM_VERSION 8.0.0)

if(NOT DEFINED ENV{QNX_HOST} OR NOT DEFINED ENV{QNX_TARGET})
  message(FATAL_ERROR
    "QNX_HOST/QNX_TARGET are not set. Source the SDP environment first:\n"
    "  source <sdp>/qnxsdp-env.sh")
endif()

set(QNX_HOST   "$ENV{QNX_HOST}")
set(QNX_TARGET "$ENV{QNX_TARGET}")

if(NOT DEFINED QNX_ARCH)
  set(QNX_ARCH "x86_64")
endif()

# qcc selects the target with -V<compiler>_<target>.
if(QNX_ARCH STREQUAL "x86_64")
  set(QNX_PROCESSOR x86_64)
  set(CMAKE_SYSTEM_PROCESSOR x86_64)
  set(QNX_COMPILER_TARGET "gcc_ntox86_64")
elseif(QNX_ARCH STREQUAL "aarch64le")
  set(QNX_PROCESSOR aarch64le)
  set(CMAKE_SYSTEM_PROCESSOR aarch64)
  set(QNX_COMPILER_TARGET "gcc_ntoaarch64le")
else()
  message(FATAL_ERROR "Unsupported QNX_ARCH '${QNX_ARCH}' (use x86_64 or aarch64le)")
endif()

set(CMAKE_C_COMPILER   "${QNX_HOST}/usr/bin/qcc")
set(CMAKE_CXX_COMPILER "${QNX_HOST}/usr/bin/q++")
set(CMAKE_C_COMPILER_TARGET   ${QNX_COMPILER_TARGET})
set(CMAKE_CXX_COMPILER_TARGET ${QNX_COMPILER_TARGET})
# _QNX_SOURCE exposes the POSIX surface. Without it q++ compiles in a strict
# mode where open/read/write/close/fsync/nanosleep/fork and _POSIX_SEM_VALUE_MAX
# are all invisible -- and the failures land inside QNX's OWN headers
# (bits/this_thread_sleep.h, sys/process.h, bits/semaphore_base.h), not just in
# application code, so std::this_thread::sleep_for and std::counting_semaphore
# do not compile either. This is a compile-mode requirement of the platform, not
# a workaround for anything in SensorForge.
set(CMAKE_C_FLAGS_INIT   "-V${QNX_COMPILER_TARGET} -D_QNX_SOURCE")
set(CMAKE_CXX_FLAGS_INIT "-V${QNX_COMPILER_TARGET} -Y_gpp -D_QNX_SOURCE")

set(CMAKE_AR      "${QNX_HOST}/usr/bin/nto${QNX_PROCESSOR}-ar" CACHE FILEPATH "")
set(CMAKE_RANLIB  "${QNX_HOST}/usr/bin/nto${QNX_PROCESSOR}-ranlib" CACHE FILEPATH "")

set(CMAKE_SYSROOT "${QNX_TARGET}")
set(CMAKE_FIND_ROOT_PATH "${QNX_TARGET}/${QNX_PROCESSOR}" "${QNX_TARGET}")

# Cross-built dependencies (e.g. a QNX GoogleTest) live outside the SDP sysroot.
# Because the find modes below are ONLY, anything not on CMAKE_FIND_ROOT_PATH is
# invisible -- so fold in whatever the caller passed as CMAKE_PREFIX_PATH.
# Without this, -DCMAKE_PREFIX_PATH=/path/to/gtest-qnx is silently ignored and
# find_package(GTest) fails even though the library is right there.
if(CMAKE_PREFIX_PATH)
  list(APPEND CMAKE_FIND_ROOT_PATH ${CMAKE_PREFIX_PATH})
endif()
if(DEFINED QNX_EXTRA_FIND_ROOT)
  list(APPEND CMAKE_FIND_ROOT_PATH ${QNX_EXTRA_FIND_ROOT})
endif()
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BEFORE)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# QNX provides pthreads inside libc; no separate -lpthread.
set(THREADS_PREFER_PTHREAD_FLAG OFF)
set(CMAKE_THREAD_LIBS_INIT "")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_PTHREADS_INIT 1)
