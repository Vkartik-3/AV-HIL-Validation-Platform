# QNX 8.0 portability artifacts

## What these files show

| File | Contents |
|---|---|
| `qnx_build_evidence.txt` | SDP paths, `q++` targets, activated license, the built binaries, their dynamic dependencies, cross-built GoogleTest, and the bootable IFS |
| `qnx_cmake_configure.log` | CMake configure with `cmake/qnx.toolchain.cmake` |
| `qnx_cross_build.log` | Full cross-compile output (11 warnings, 0 errors) |
| `test_ifs.build` | The `mkifs` build file used to bake the test binary into a bootable image |
| `qnx_boot_attempts.txt` | All three QEMU boot attempts and why they produced no output |

## Reproducing

```bash
source <sdp>/qnxsdp-env.sh
cmake -S standalone -B build-qnx \
      -DCMAKE_TOOLCHAIN_FILE=../cmake/qnx.toolchain.cmake \
      -DQNX_ARCH=x86_64 -DSENSORFORGE_CORE_ONLY=ON \
      -DCMAKE_PREFIX_PATH=<gtest-built-for-qnx>
cmake --build build-qnx -j
```

## Status, stated exactly

**Cross-compilation: proven.** `x86_64-pc-nto-qnx8.0.0`, `q++` / GCC 12.2.0.
Zero SensorForge source changes; one toolchain flag (`-D_QNX_SOURCE`), required
because QNX's own libstdc++ and `sys/process.h` headers need it.

**Execution on target: NOT done.** QEMU on this Apple-silicon host runs x86_64
under TCG software emulation with no hardware virtualisation, and QNX's
`startup-x86` does not reach userspace under it. Running the suite requires an
x86_64 host with virtualisation support.
