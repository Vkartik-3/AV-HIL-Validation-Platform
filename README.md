---
## Attribution and Project Status

SensorForge is built on top of network_bridge
(https://github.com/brow1633/network_bridge) by brow1633,
developed for the Indy Autonomous Challenge / Purdue AI
Racing. Licensed under original project license.

## SensorForge extensions by Kartik Vadhawana

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

### Measured

Raw artifacts and the exact environment are in [`artifacts/`](artifacts/).
Headline figures from the end-to-end reference workload (`e2e_reference`), which
drives the real path with a realistic five-sensor mix:

- **Paced, 30 s**: 10,801 messages, 0 dropped, 0 CRC failures, 0 sequence gaps.
  8.5 MB/s framed and written. Capture-to-record p50 365 us, p99 1,470 us,
  p999 5,296 us. Peak RSS 5.3 MiB. All 10,801 records replayed and re-validated.
- **Saturated, 15 s**: 460.5 M capture attempts against a slower consumer.
  Buffers stayed bounded at 28 MB queued / 79 MiB peak RSS; camera overwrote
  2.58 M frames while LiDAR/IMU/GPS dropped newest and CAN blocked. Still 0 CRC
  failures and 0 rejected frames; 121,784 sequence gaps correctly detected.
- **TSan, 30 s**: 127.6 M ring operations including 63.4 M producer-side
  evictions. Zero races, zero corruption, zero sequence regressions.
- **Offload**: recording throughput was 355.5 msgs/s with a healthy destination
  and 355.7 msgs/s with a dead one, i.e. recording never blocks on offload. A
  backlog accumulated during a 15 s outage drained in 707 ms with no loss.

### Known limitations

These are real and deliberately stated rather than papered over.

- **The ROS2 side is written but was not built or tested in this environment.**
  No ROS2 distribution is available on the development host, so
  `src/network_bridge.cpp`, `src/subscription_manager.cpp`, the sensor
  publishers, the scenario runner and the launch tests were not compiled here.
  The `ros-humble` CI job builds them and now also runs `colcon test`.
  Everything in `artifacts/` comes from the ROS-free core.
- **The decoder's stream key is the frame's `sensor_type`, not the topic.** The
  topic name lives inside the compressed payload, so it is not available before
  validation. This is strictly better than the previous constant `0` (which
  merged every topic into one sequence space) but is still coarser than
  per-topic keying.
- **`fsync` bounds loss to the live segment, not to zero.** Only
  `every_record` gives per-record durability, and its cost is measured in
  `artifacts/fsync_cost.txt`.
- **Replay ordering assumes a single writer.** `stream_replay` delivers in
  (segment id, offset) order, which equals capture order for one writer. A
  multi-writer topology would need a merge step.
- **No fuzzing result is claimed locally** — Apple's clang ships no libFuzzer
  runtime. The harnesses compile; CI runs them.
- **QNX is not supported.** No SDP or toolchain was available; nothing was
  built or tested on QNX and no QNX support is claimed.
- **Real recorded AV data (rosbag2 / KITTI) was not ingested.** The ingestion
  seam exists — `Recorder::capture(stream_id, bytes, len)` accepts arbitrary
  serialized payloads, so a rosbag2 reader or a KITTI converter only has to call
  it — but writing and shipping that reader requires a ROS2 environment to
  compile and a dataset to verify against, neither of which is present. No
  real-data result is claimed. The intended command once ROS2 is available:
  `ros2 bag play <bag> --remap /points:=/sensors/lidar` against a bridge
  configured per `config/Recorder.yaml`.
- **This is not a real-time system.** No `SCHED_FIFO`, no CPU pinning, no
  `mlockall`, and the hot path allocates per message.

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
