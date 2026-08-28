# QNX 8.0 portability artifacts

## What these files show

| File | Contents |
|---|---|
| `qnx_build_evidence.txt` | SDP paths, `q++` targets, activated license, the built binaries, their dynamic dependencies, cross-built GoogleTest, and the bootable IFS |
| `qnx_cmake_configure.log` | CMake configure with `cmake/qnx.toolchain.cmake` |
| `qnx_cross_build.log` | Full cross-compile output (11 warnings, 0 errors) |
| `test_ifs.build` | The `mkifs` build file used to bake the test binary into a bootable image |
| `qnx_boot_attempts.txt` | QEMU boot attempts on macOS and why they produced no output |
| `qnx_ec2_x86_64.txt` | Independent reproduction of the cross-build on x86_64 Linux (EC2), plus the same boot failure there |

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

**Execution on target: NOT done.** Four boot attempts across two hosts
(Apple-silicon macOS, and x86_64 AWS EC2 running Ubuntu 22.04) and two image
types (a custom multiboot IFS with the tests baked in, and QNX's own unmodified
`mkqnximage` disk image). All four load the kernel and then emit nothing.

This isolates the cause: **QNX does not come up under QEMU TCG software
emulation.** Cross-ISA translation was the initial suspicion, but the identical
failure on a native x86_64 host rules it out, and the failure of QNX's own image
rules out the custom IFS. KVM is required.

On AWS that means a bare-metal instance; the attempt was blocked by an account
vCPU quota of 16 (x86 bare metal needs 48-96). A quota increase was requested and
remained pending. Every AWS resource created was terminated and verified gone.

To finish this on any machine with `/dev/kvm`, roughly 20 minutes:
build per the commands above, then `mkifs test_ifs.build test.ifs` and
`qemu-system-x86_64 -enable-kvm -kernel test.ifs -nographic`.
