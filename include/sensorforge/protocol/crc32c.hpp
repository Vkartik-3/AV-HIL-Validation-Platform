/*
==============================================================================
SensorForge - CRC32C (Castagnoli) implementation
Part of the SensorForge AV HIL validation platform.

Provides a hardware-accelerated CRC32C when the CPU exposes the SSE4.2
(x86) or CRC (ARMv8) instructions, and a portable table-driven software
fallback otherwise. The polynomial is 0x1EDC6F41 (reflected 0x82F63B78),
the same one used by iSCSI, ext4, and Google's crc32c library, so results
are cross-checkable against known test vectors.

Everything here is header-only and has no ROS2 dependency so it can be
linked into fuzz targets, benchmarks, and the plain (non-ROS) CMake build.
==============================================================================
*/

#pragma once

#include <cstddef>
#include <cstdint>
#include <array>

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__)
  #define SENSORFORGE_X86 1
  #include <nmmintrin.h>   // _mm_crc32_u8/u32/u64 (SSE4.2)
#endif

#if defined(__aarch64__) || defined(__ARM_FEATURE_CRC32)
  #define SENSORFORGE_ARM_CRC 1
  #include <arm_acle.h>    // __crc32cb/cw/cd
#endif

namespace sensorforge::crc {

namespace detail {

// Reflected CRC32C polynomial.
constexpr uint32_t kPoly = 0x82F63B78u;

// Compile-time generated lookup table for the software fallback.
constexpr std::array<uint32_t, 256> make_table()
{
  std::array<uint32_t, 256> table{};
  for (uint32_t i = 0; i < 256; ++i) {
    uint32_t crc = i;
    for (int k = 0; k < 8; ++k) {
      crc = (crc & 1u) ? (crc >> 1) ^ kPoly : (crc >> 1);
    }
    table[i] = crc;
  }
  return table;
}

inline constexpr std::array<uint32_t, 256> kTable = make_table();

inline uint32_t crc32c_software(uint32_t crc, const uint8_t * data, size_t len)
{
  for (size_t i = 0; i < len; ++i) {
    crc = kTable[(crc ^ data[i]) & 0xFFu] ^ (crc >> 8);
  }
  return crc;
}

#if defined(SENSORFORGE_X86)
inline bool has_sse42()
{
  // Cache the result; CPU features do not change at runtime.
  static const bool supported = []() {
    return __builtin_cpu_supports("sse4.2");
  }();
  return supported;
}

// The target attribute lets this function use SSE4.2 intrinsics even when the
// translation unit is compiled WITHOUT -msse4.2 (the default colcon build).
// Without it, GCC/Clang reject `_mm_crc32_u64` with "target specific option
// mismatch". Runtime dispatch via has_sse42() still gates whether we call it.
#if defined(__GNUC__) || defined(__clang__)
__attribute__((target("sse4.2")))
#endif
inline uint32_t crc32c_hw(uint32_t crc, const uint8_t * data, size_t len)
{
  size_t i = 0;
  for (; i + 8 <= len; i += 8) {
    uint64_t chunk;
    __builtin_memcpy(&chunk, data + i, sizeof(chunk));
    crc = static_cast<uint32_t>(_mm_crc32_u64(crc, chunk));
  }
  for (; i < len; ++i) {
    crc = _mm_crc32_u8(crc, data[i]);
  }
  return crc;
}
#endif

#if defined(SENSORFORGE_ARM_CRC)
#if defined(__GNUC__) || defined(__clang__)
__attribute__((target("+crc")))
#endif
inline uint32_t crc32c_hw(uint32_t crc, const uint8_t * data, size_t len)
{
  size_t i = 0;
  for (; i + 8 <= len; i += 8) {
    uint64_t chunk;
    __builtin_memcpy(&chunk, data + i, sizeof(chunk));
    crc = __crc32cd(crc, chunk);
  }
  for (; i < len; ++i) {
    crc = __crc32cb(crc, data[i]);
  }
  return crc;
}
#endif

}  // namespace detail

/**
 * @brief Compute the CRC32C of a buffer.
 *
 * Uses the hardware instruction when available, otherwise a table-driven
 * software implementation. The result is the standard finalized CRC32C
 * (init 0xFFFFFFFF, final XOR 0xFFFFFFFF) so it matches published vectors:
 *   crc32c("") == 0x00000000
 *   crc32c("123456789") == 0xE3069283
 */
inline uint32_t crc32c(const uint8_t * data, size_t len)
{
  uint32_t crc = 0xFFFFFFFFu;
#if defined(SENSORFORGE_X86)
  crc = detail::has_sse42()
    ? detail::crc32c_hw(crc, data, len)
    : detail::crc32c_software(crc, data, len);
#elif defined(SENSORFORGE_ARM_CRC)
  crc = detail::crc32c_hw(crc, data, len);
#else
  crc = detail::crc32c_software(crc, data, len);
#endif
  return crc ^ 0xFFFFFFFFu;
}

/// Convenience overload for contiguous byte containers (vector, span, ...).
template<typename Container>
inline uint32_t crc32c(const Container & c)
{
  return crc32c(reinterpret_cast<const uint8_t *>(c.data()), c.size());
}

}  // namespace sensorforge::crc
