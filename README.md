---
## Attribution and Project Status

SensorForge is built on top of network_bridge
(https://github.com/brow1633/network_bridge) by brow1633,
developed for the Indy Autonomous Challenge / Purdue AI
Racing. Licensed under original project license.

## SensorForge extensions by Kartik Vadhawana

SensorForge is a **performance-sensitive multi-sensor recording pipeline**:
bounded per-stream buffering with explicit backpressure policies, CRC-validated
binary framing, a restart-safe segmented write-ahead log, and deterministic
replay -- built so that queue depth, drops and memory stay inside stated limits
under load, and so that every one of those limits is observable while running.

It is **not a hard real-time system**, and nothing here should be read as one.
There is no `SCHED_FIFO` or `SCHED_RR`, no CPU pinning or `isolcpus`, no
`mlockall`, no priority inheritance, no deadline-miss detection, and no
worst-case execution time analysis. The consumer is a best-effort ROS2 wall
timer, and the hot path allocates. The guarantees offered are **boundedness and
observability**, not latency bounds. See "Known limitations" for the specifics.

Everything below is layered on top of upstream `network_bridge`. The boundary is
kept explicit: see "Upstream vs SensorForge" at the end of this section.

### The integrated recording path

Sensor data traverses one path, and the tests and measurements exercise that
same path rather than a parallel one:

```
producer  -> Recorder::capture()          per-stream sequence + monotonised timestamp
          -> recording policy             allow / deny / metadata-only
          -> StreamBuffer                 bounded by FRAMES and BYTES, per-stream policy
          -> drain thread
          -> encode_frame()               magic/version/seq/ts + CRC32C header & payload
          -> selective-recording gate     baseline decimation, pre-roll, trigger windows
          -> WalWriter::append()          segmented, CRC'd, restart-safe, fsync policy
          -> frame sink                   transport / fault engine
          -> FrameDecoder                 CRC + per-stream sequence-gap validation
```

`wal_replay` reads a recording back deterministically and prints a stable digest.

### What is implemented

| Area | Implementation |
|---|---|
| Multi-stream recorder | `include/sensorforge/pipeline/recorder.hpp`, `src/pipeline/recorder.cpp`. N streams, each with sensor type, independent sequence, backpressure policy, byte/frame accounting, recording counters. |
| Bounded buffering | `core/stream_buffer.hpp`. Runtime `max_frames` **and** `max_bytes` per stream. |
| Ring correctness | `core/spsc_ring.hpp`. Producer-side eviction is CAS-claimed and guarded by a published reader floor; exercised under TSan. |
| Backpressure | `core/backpressure_policy.hpp`. overwrite-oldest / drop-newest / bounded block. |
| WAL durability | `replay/wal_writer.hpp`. Segment discovery, tail validation, partial-record recovery, `never`/`on_segment_seal`/`interval`/`every_record` fsync. |
| Deterministic replay | `wal_replay` tool, streaming (one segment resident), stable CRC32C digest, frame re-validation. |
| Timestamp/sequence integrity | `core/clock.hpp` + `protocol/frame_codec.hpp`. Monotonised wall clock; per-stream gap counting; bounded state; resync instead of wedging. |
| Observability | Prometheus exporter on the **bridge** (`metrics_port`), queue depth/bytes, drops, overwrites, CRC failures, rejects, gaps, WAL records/bytes, RSS, CPU. Structured JSON events. |
| Resource budgets | `core/resource_monitor.hpp`. Soft/hard RSS and queue-byte budgets; hard breach sheds. Linux `/proc` behind a platform interface. |
| Selective recording | `pipeline/trigger.hpp`. Decimated baseline, bounded pre-roll counted against the budget, deterministic overlapping-trigger semantics. |
| Data minimisation | `pipeline/recording_policy.hpp`. Topic allowlist/denylist and metadata-only mode, tested to prove denied payloads never reach the WAL. |
| Offload | `offload/uploader.hpp`. Sealed-segment queue, bounded, background worker, durable manifest, bounded backoff, restart recovery, filesystem destination with copy→fsync→atomic rename. |

### Validation

Everything below was run and the raw output is committed under
[`artifacts/`](artifacts/). Nothing here is transcribed by hand or carried over
from a different machine.

| Check | Where | Result |
|---|---|---|
| GitHub Actions, all 8 jobs | x86_64 Ubuntu | green ([run 33122376223](https://github.com/Vkartik-3/AV-HIL-Validation-Platform/actions/runs/33122376223)) |
| ROS2 build + `colcon test` | `ros:humble`, Docker + CI | 8/8 ctest, both launch tests pass |
| ROS2 end-to-end, 4 sensors | `ros:humble`, Docker + CI | PASS (gate in `test/e2e/run_ros2_e2e.sh`; `vcan0` unavailable, so CAN excluded) |
| rosbag2 ingestion | `ros:humble`, Docker | 5,015 records, all validate as frames |
| Unit suite | macOS arm64 | 198 tests, 101,199 assertions, 0 failures |
| ASan + UBSan | macOS arm64 | 198 tests, 0 sanitizer errors |
| TSan, 30 s | macOS arm64 | 126,175,155 ring ops, 61,409,029 evictions, 0 races |
| Deterministic replay | both | identical digest across runs; CI gates on it |
| Sustained capacity | macOS laptop | **11,007 msgs/s @ 262 MB/s for 60 s, zero drops** |
| QNX 8.0 cross-compile | `q++` 12.2.0, x86_64 target | core + GoogleTest build clean on **macOS/Docker and x86_64 Linux**; **not executed on target** |

#### ROS2 end-to-end (the integrated path)

`test/e2e/run_ros2_e2e.sh` drives publishers &rarr; DDS &rarr; SubscriptionManager
&rarr; bounded buffering &rarr; framing &rarr; CRC &rarr; WAL &rarr; TCP &rarr;
receiver &rarr; replay, and **fails** on a non-deterministic replay digest, a
record that does not re-validate as a frame, any receiver sequence gap, missing
sequence, CRC failure or frame reject, or missing republished topics. Measured
(45 s, `ros:humble` on Docker, artifacts in `artifacts/linux_ros2_e2e/`):

```
duration            45.006 s          topics republished  /recv/sensors/{lidar,camera,imu,gps}
frames sent         12,610            IMU republish rate  200.1 Hz (publisher: 200 Hz)
frame bytes sent    426.2 MB          WAL records         12,610
enqueued            lidar 484 / camera 1,458 / imu 9,706 / gps 972
dropped             0 on every stream       overwritten   0
sequence gaps       0                 missing sequences   0
CRC failures        0                 frame rejects       0
peak queued bytes   camera 460,904 | lidar 196,721 | imu 3,564 | gps 128
peak RSS            27.3 MB           CPU                 5.0 %
replay              14,290 records, frame_verify ok=14,290 bad=0, digest stable
```

#### Sustained capacity (the headroom figure)

The paced run below is deliberately far below capacity and the saturated run is
deliberately above it, so neither answers "how much can this actually take?".
`tools/capacity_sweep.cpp` answers it: it ramps the whole sensor mix, bisects
the boundary, then requires the winner to hold for a full 60-second window.

A scale only counts as sustained if **all three** hold: zero drops, zero
overwrites (an overwritten frame never reached the WAL), and **no backlog
growth** -- because a run can post zero drops purely because a 96 MiB buffer has
not filled up yet. Rates are what the pipeline actually accepted, not what the
producers requested.

```
sustained          30.7x the reference sensor mix, held for 60 s
                   11,007 msgs/s  |  262.3 MB/s framed  |  262.7 MB/s to the WAL
integrity          660,554 captured, 0 dropped, 0 overwritten,
                   694,139 frames validated, 0 CRC failures, 0 sequence gaps
backlog            -0.000 MB/s (queue drained to empty by the end of the window)
latency            capture-to-record p50 204 us | p99 11.9 ms | p999 96.7 ms
resources          peak RSS 128.0 MiB, peak queued 31.5 MB, peak CPU 52%
```

Two things worth reading off that honestly. **Latency degrades near capacity**:
p99 is 11.9 ms here versus 1.4 ms on the paced run, because the queues are
genuinely deep at 30x load -- capacity and latency are a trade, not a free lunch.
And the boundary is **unstable**: 48x passed a 12 s trial and failed at 60 s
twice, so the tool stepped down until a scale actually held. The reported figure
is the one that survived the long window, not the best one ever seen.

Measured on the macOS development laptop (thermal throttling, shared disk, no
pinning), so this is a floor rather than a ceiling; a controlled server would
likely go higher. Raw sweep in `artifacts/capacity_sweep_macos.{txt,json}`.

#### Core pipeline (no ROS, five-sensor mix incl. CAN-sized payloads)

```
paced 30 s      10,804 captured = 10,804 recorded, 0 dropped, 0 CRC failures,
                0 sequence gaps, 8.53 MB/s to the WAL,
                capture-to-record p50 262 us / p99 1,398 us / p999 7,366 us,
                peak RSS 5.0 MiB, replay 10,804/10,804 verified, recovery 1.17 ms
saturated 12 s  406,927,993 capture attempts against a slower consumer:
                buffers held at 28 MB queued / 70 MiB peak RSS,
                camera overwrote 2,194,918 while the others dropped newest,
                0 CRC failures, 122,356 records all replay-verified
offload         355.9 msgs/s with a healthy destination vs 355.7 msgs/s with a
                dead one -- recording never blocks on offload; a backlog built
                during the outage drained in 484 ms with no loss
```

### Known limitations

Stated deliberately rather than papered over.

- **CAN is not exercised through the ROS2 path anywhere yet.** CAN rides
  SocketCAN (`vcan0`), not DDS. Docker Desktop's LinuxKit VM has no `vcan`
  module, and running the CI container `--privileged` did **not** fix it either
  -- the GitHub run reports `vcan0=unavailable` in its annotations, same as
  local. So the "five-sensor" end-to-end is in practice a **four-sensor** run
  (LiDAR, camera, IMU, GPS) plus a recorded `vcan0=unavailable`. The script
  deliberately reports that rather than quietly claiming five. Closing this
  needs `modprobe vcan` on a real host kernel -- a non-containerised CI job or a
  VM/EC2 instance. CAN-sized payloads *are* exercised by the ROS-free core
  workload, but not over SocketCAN through the bridge.
- **Large frames need TCP, not UDP.** A 196 KB LiDAR frame and a 230 KB camera
  frame exceed the ~64 KiB UDP datagram limit; over UDP the OS rejects them
  outright ("Message too long"). The frame header reserves `kFlagFragmented`
  but **no fragmentation is implemented**, so the end-to-end run uses the TCP
  interface. This is a real gap, not a configuration preference.
- **No capture-to-record latency percentiles on the ROS2 path.** The bridge
  never instrumented that; only the Recorder does. The ROS2 numbers above are
  throughput, drops, integrity, RSS and CPU. The p50/p99/p999 figures come from
  the core pipeline workload.
- **Real-world AV data was not ingested.** rosbag2 *ingestion* is proven end to
  end (5,084 messages recorded to a bag, replayed back through the bridge,
  5,015 WAL records, all validating as frames -- `artifacts/rosbag2_ingestion/`),
  but the bag was captured from this repository's own synthetic publishers, so
  it is **not** a real-world dataset. Ingesting KITTI or a public AV bag needs a
  registered download and a converter, which was out of scope here. Once such a
  bag exists the command is `ros2 bag play <bag> --remap /points:=/sensors/lidar`
  against a bridge configured per `config/Recorder.yaml`.
- **QNX: the core CROSS-COMPILES; the tests were never EXECUTED on a QNX
  target.** This distinction is the whole claim, so it is worth being exact.

  What is proven (`artifacts/qnx/`): with QNX SDP 8.0 installed and licensed,
  the ROS-free core builds cleanly for `x86_64-pc-nto-qnx8.0.0` using `q++`
  (`gcc_ntox86_64`, GCC 12.2.0). GoogleTest was cross-built for the same target
  and the suite links into a valid QNX Neutrino binary -- `ELF 64-bit LSB pie
  executable, x86-64, interpreter /usr/lib/ldqnx-64.so.2`, needing
  `libc.so.6`, `libstdc++.so.6`, `libm.so.3`, `libgcc_s.so.1`, `libregex.so.1`.
  **Zero SensorForge source changes were required.** The only change was one
  toolchain flag, `-D_QNX_SOURCE` (see `cmake/qnx.toolchain.cmake`), and it is
  needed because QNX's *own* headers -- `bits/this_thread_sleep.h`,
  `sys/process.h`, `bits/semaphore_base.h` -- do not compile without it.
  Notably `<filesystem>`, used by the WAL and offload code, compiled without
  incident.

  The cross-build was reproduced independently on a clean **x86_64 Linux host**
  (AWS EC2, Ubuntu 22.04, kernel 6.8) as well as on macOS/Docker: same result,
  **0 compile errors** (`artifacts/qnx/qnx_ec2_x86_64.txt`).

  What is NOT proven: **no test has run on QNX.** Boot was attempted four times
  across two hosts and two image types -- a purpose-built multiboot IFS with the
  tests baked in, and QNX's own `mkqnximage` disk image -- on both an
  Apple-silicon Mac and an x86_64 EC2 instance. Every attempt loads the kernel
  (`QNX v1.2d Boot Loader` / `Booting from ROM`) and then emits nothing.

  Two candidate causes ARE ruled out. It is not cross-ISA emulation, because a
  native x86_64 host fails identically to the Apple-silicon one. It is not the
  hand-written IFS, because QNX's own unmodified `mkqnximage` image fails the
  same way. Console wiring was checked rather than assumed
  (`startup-x86 -D8250..115200` with `devc-ser8250`, and SeaBIOS output on that
  same serial path was captured), so a silent serial port is not the explanation.

  The **actual cause remains open.** Neither host had `/dev/kvm`, so the absence
  of hardware virtualisation is the leading hypothesis and the cheapest next
  thing to test -- but it is a hypothesis, not a finding: no successful KVM boot
  was performed for comparison, and no QNX documentation was consulted stating
  that QNX 8.0 cannot run under QEMU TCG. Emulated-platform gaps, a `-cpu qemu64`
  feature mismatch, and machine-type/firmware differences are all still open
  possibilities; `artifacts/qnx/qnx_boot_attempts.txt` enumerates them.

  Testing the KVM hypothesis on AWS needs a bare-metal instance, which was
  blocked by an account vCPU quota of 16 (x86 bare-metal types need 48-96); an
  increase was requested and was still pending. All AWS resources created for
  the attempt were terminated and verified gone. Any x86_64 Linux machine with
  `/dev/kvm` is the next step, using `cmake/qnx.toolchain.cmake` and
  `artifacts/qnx/test_ifs.build`.

  So: **"the portable core cross-compiles for QNX 8.0" is defensible. "Runs on
  QNX" is not, and is not claimed anywhere.**
- **`fsync` bounds loss to the live segment**, not to zero, unless
  `every_record` is selected; its cost is in `artifacts/fsync_cost.txt`.
- **Replay ordering assumes a single writer** -- records are delivered in
  (segment id, offset) order, which equals capture order for one writer only.
- **The decoder's stream key is the frame's `sensor_type`, not the topic.** The
  topic name lives inside the compressed payload and is not available before
  validation. This is coarser than per-topic keying: two topics sharing a
  sensor type would share a sequence space.
- **No local fuzzing result** -- Apple's clang ships no libFuzzer runtime. The
  harnesses compile locally; CI runs them.
- **Not hard real-time, and the hot path allocates.** Counted in the source,
  the bridge's capture-to-wire path performs **five** heap allocations per
  message: `frame.data.resize()` in the subscription callback
  (`subscription_manager.cpp:139`), then on the send thread the vector returned
  by `create_header()`, the `message.reserve()` concatenation buffer, the
  `ZSTD_compressBound()` resize inside `compress()`, and the vector returned by
  `encode_frame()` (`frame_codec.cpp:91`). Moving a frame into a ring slot also
  frees that slot's previous buffer. The ring itself is allocation-free, but
  nothing around it is.
  Combined with a best-effort wall timer as the consumer, latency is *typical
  case measured*, not bounded. Making it hard real-time would mean preallocated
  buffer pools, `SCHED_FIFO` with `mlockall`, a PREEMPT_RT kernel and WCET
  analysis -- a redesign of the hot path, not a configuration change. The
  measured capture-to-record percentiles above are what the system does on an
  ordinary kernel under no contention; they are not a guarantee.
- **Benchmark hosts are not controlled.** A developer laptop and shared CI
  runners, with no pinning or governor control. Numbers are reproducible in
  kind, not to the digit.

### Upstream vs SensorForge

**Upstream `network_bridge` (Ethan Brown / Purdue AI Racing)** provides: the ROS2
node skeleton, `GenericSubscription`/`GenericPublisher` topic bridging, the
pluginlib transport abstraction with UDP and TCP interfaces, zstd compression,
the TF special-casing with include/exclude regex filtering, the launch files and
the two launch tests.

**SensorForge adds**: the binary frame protocol with CRC32C, the bounded stream
buffers and backpressure policies, the segmented WAL with restart recovery, the
deterministic replay tool, the recorder, selective recording, the recording
policy, sealed-segment offload, resource budgets, the bridge-level metrics
export, the benchmark/stress/fuzz harnesses, the scenario runner and fault
engine, and the CI matrix. Files under `include/sensorforge/`, `src/protocol/`,
`src/replay/`, `src/pipeline/`, `src/offload/`, `src/core/`, `tools/`, `bench/`,
`stress/`, `fuzz/`, `scenario/`, `faults/`, `report/`, `metrics/` and
`test/unit/` are SensorForge; the rest is upstream with SensorForge integration
edits.

Licensed under original project license. See LICENSE.

---

# Network Bridge
[![CI](https://github.com/brow1633/network_bridge/actions/workflows/CI.yml/badge.svg)](https://github.com/brow1633/network_bridge/actions/workflows/CI.yml)

**Network Bridge** is a lightweight ROS2 node designed for robust communication between robotic systems over arbitrary network protocols. Supporting UDP and TCP protocols out of the box, this packages seamlessly bridges ROS2 topics across networks, facilitating effective remote communications between a base station and robotic systems, or between multiple robotic systems.

## Installation
### Installation via apt
Install with:
```
sudo apt install ros-<distro>-network-bridge
```

### Building from Source
Simply clone the repository into your ROS2 workspace and build with `colcon build`.

## Usage

### Demo
#### TCP
```
ros2 launch network_bridge tcp.launch.py

ros2 topic pub /tcp1/MyDefaultTopic std_msgs/msg/String "data: 'Hello World'"

ros2 topic echo /tcp2/MyDefaultTopic
```

#### UDP
```
ros2 launch network_bridge udp.launch.py

ros2 topic pub /udp1/MyDefaultTopic std_msgs/msg/String "data: 'Hello World'"

ros2 topic echo /udp2/MyDefaultTopic
```
### Configuration
Simply setup the network interface parameters and list your desired topics to get started.  If you are using UDP over cellular data, it is recommended to setup a VPN to facilitate connection.  Also, please note that **no encryption** occurs within this package.  Currently, if you would like encryption, you must use a VPN.

See `config/Udp1.yaml` for a description of all parameters, as well as the TCP example configuration files.
#### Minimal Example
The following configuration examples demonstrate a robot sending a message on `/gps/fix` over UDP to a basestation that will then re-publish the message.  This works seamlessly on all message types, so long as they are built and sourced on both ends of the transmission.
#### Robot
```
/udp_sender:
  ros__parameters:
    UdpInterface:
      local_address: "192.168.1.2"
      receive_port: 5001
      remote_address: "192.168.1.3"
      send_port: 5000

    topics:
      - "/gps/fix"
```
#### Base Station
```
/udp_receiver:
  ros__parameters:
    UdpInterface:
      local_address: "192.168.1.3"
      receive_port: 5000
      remote_address: "192.168.1.2"
      send_port: 5001
```
#### Special case: TF
The TF topic `/tf` or `/tf_static` are handled as a special cases. The subscriber side will listen to all TF messages, accumulate them
(similarly to a TF buffer) and send all of them at the specified rate. The behavior can be disabled or forced using the `is_tf` configuration.

If some TFs need to be excluded or if the list of TFs to include is finite, one can use the include and exclude regex parameters. A transform is matched (hence excluded or included) if either the `frame_id` or `child_frame_id` are matching a pattern.
```
/udp_sender:
  ros__parameters:
    UdpInterface:
      local_address: "192.168.1.2"
      receive_port: 5001
      remote_address: "192.168.1.3"
      send_port: 5000

    topics:
      - "/prefix/tf"
      - "/tf_static"

    /prefix/tf:
      - is_tf: True
      - is_static_tf: False
      - rate: 10.

    /tf_static:
      - rate: 1.0
      - is_static_tf: True
      - exclude: ["standoff.*", "spacer.*", ".*wheel_link", ".*cliff.*"]
```

### Choice of protocol
- **UDP**: Use UDP for low-latency, high-throughput communications, where occasional data loss is tolerable. Ideal for real-time telemetry data like sensor streams.
- **TCP**: Opt for TCP when data integrity and reliability are critical. This ensures that control commands and state transitions are reliably delivered, though with potentially higher latency.

Network protocols are implemented as pluginlib plugins, allowing the creation of arbitrary interfaces using the abstract class `include/network_interfaces/network_interface_base.hpp`.  Any interface that can send and receive bytes could theoretically be implemented, including protocols that go beyond point-to-point communication, such as ZMQ.  Please consider opening a pull request if you implement a new network interface.

### Tuning
This node can be launched with logger level DEBUG, which provides useful information for tuning the compression, rate and stale message parameters.  For each message that is sent, the receiving side will output the number of bytes received, the decompressed size in bytes and the transmission delay.

### Contributing
Thank you for considering contributing!

#### Code Formatting
Python code is formatted with `black`, and C++ is formatted with `uncrustify`.

#### Pre-commit hooks
To ease the friction of linting, there are pre-commit hooks that you can install:

```bash
sudo apt install pre-commit

pre-commit run -a # Run on all files manually

pre-commit install # Run on commit automatically
```

which will reformat code automatically when you commit changes.

## Acknowledgements
This package was developed for use in the Indy Autonomous Challenge by the Purdue AI Racing team.  Inspiration was taken from mqtt_client (https://github.com/ika-rwth-aachen/mqtt_client/).
