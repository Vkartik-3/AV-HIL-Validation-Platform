#!/usr/bin/env bash
# =============================================================================
# SensorForge - ROS2 five-sensor end-to-end validation
#
# Drives the REAL integrated path, not a parallel one:
#
#   sensor publishers -> ROS2/DDS -> SubscriptionManager -> per-stream bounded
#   buffering -> SensorForge framing -> CRC -> WAL -> TCP transport -> receiver
#   -> frame validation -> deterministic replay
#
# TCP rather than UDP is deliberate: a 196 KB LiDAR frame and a 230 KB camera
# frame both exceed the ~64 KiB UDP datagram limit, and the frame protocol
# reserves kFlagFragmented but implements no fragmentation. Over UDP the OS
# rejects those sends outright ("Message too long"); TCP's length-prefixed
# framing carries them.
#
# CAN is carried out-of-band on SocketCAN (vcan0), not over DDS, so it needs a
# kernel with the vcan module. Where vcan0 is unavailable the script records
# that fact and continues with the four DDS sensors rather than silently
# reporting a five-sensor run.
#
# MEASURED: vcan0 has been unavailable on every host tried so far -- Docker
# Desktop's LinuxKit VM lacks the module, and a --privileged CI container did
# not resolve it either. Closing this needs modprobe vcan on a real host
# kernel (a non-containerised runner or a VM/EC2 instance).
#
# Usage: run_ros2_e2e.sh <workspace_install_dir> <config_dir> <output_dir> [duration_s]
# =============================================================================
set +u
set -o pipefail

WS_INSTALL="${1:?workspace install dir required}"
CFG_DIR="${2:?config dir required}"
OUT="${3:?output dir required}"
DUR="${4:-45}"

LOGS="$OUT/logs"; WAL="$OUT/wal"
mkdir -p "$LOGS" "$WAL"; rm -rf "${WAL:?}/"*

source "$WS_INSTALL/setup.bash"

# --- CAN: bring up vcan0 if the kernel allows it -----------------------------
CAN_STATUS="unavailable"
if modprobe vcan 2>/dev/null && ip link add dev vcan0 type vcan 2>/dev/null \
   && ip link set up vcan0 2>/dev/null; then
  CAN_STATUS="up"
elif ip link show vcan0 >/dev/null 2>&1; then
  CAN_STATUS="up"
fi
echo "vcan0=$CAN_STATUS" | tee "$LOGS/vcan.txt"

sed "s|@WAL_DIR@|$WAL|g" "$CFG_DIR/sender_tcp.yaml" > "$OUT/sender.yaml"
cp "$CFG_DIR/receiver_tcp.yaml" "$OUT/receiver.yaml"

echo "### receiver bridge"
ros2 run network_bridge network_bridge --ros-args -r __node:=sf_receiver \
  --params-file "$OUT/receiver.yaml" > "$LOGS/receiver.log" 2>&1 &
RX_PID=$!
sleep 3
echo "### sender bridge"
ros2 run network_bridge network_bridge --ros-args -r __node:=sf_sender \
  --params-file "$OUT/sender.yaml" > "$LOGS/sender.log" 2>&1 &
TX_PID=$!
sleep 3

echo "### sensor publishers"
PIDS=()
ros2 run network_bridge lidar_publisher  --ros-args -p rate_hz:=10.0  -p topic:=/sensors/lidar  > "$LOGS/lidar.log" 2>&1 &
PIDS+=($!)
ros2 run network_bridge camera_publisher --ros-args -p rate_hz:=30.0  -p topic:=/sensors/camera > "$LOGS/camera.log" 2>&1 &
PIDS+=($!)
ros2 run network_bridge imu_publisher    --ros-args -p rate_hz:=200.0 -p topic:=/sensors/imu    > "$LOGS/imu.log" 2>&1 &
PIDS+=($!)
ros2 run network_bridge gps_publisher    --ros-args -p rate_hz:=20.0  -p topic:=/sensors/gps    > "$LOGS/gps.log" 2>&1 &
PIDS+=($!)
if [ "$CAN_STATUS" = "up" ]; then
  ros2 run network_bridge can_publisher --ros-args -p rate_hz:=100.0 > "$LOGS/can.log" 2>&1 &
  PIDS+=($!)
fi
sleep 5

echo "### steady state for ${DUR}s"
START=$(date +%s.%N)
timeout $((DUR + 5)) ros2 topic hz /recv/sensors/imu --window 200 > "$LOGS/hz_imu.txt" 2>&1 &
sleep "$DUR"
END=$(date +%s.%N)
awk -v a="$START" -v b="$END" 'BEGIN{printf "DURATION_S=%.3f\n", b-a}' > "$LOGS/duration.txt"

echo "### scrape bridge metrics"
curl -s --max-time 5 http://127.0.0.1:9101/metrics > "$LOGS/metrics_sender.txt"
curl -s --max-time 5 http://127.0.0.1:9102/metrics > "$LOGS/metrics_receiver.txt"
ros2 topic list > "$LOGS/topics.txt" 2>&1
timeout 8 ros2 topic echo /recv/sensors/gps --once > "$LOGS/echo_gps.txt" 2>&1

if [ "$CAN_STATUS" = "up" ]; then
  timeout 5 candump -n 20 vcan0 > "$LOGS/candump.txt" 2>&1 || true
fi

echo "### shutdown"
# Kill only the processes this script started. `pkill -f network_bridge` also
# matched this script's own command line (its path contains "network_bridge"),
# which terminated the run before it could validate anything.
# `ros2 run` FORKS the real executable, so killing the wrapper PID leaves the
# bridge process alive and still appending to the WAL. Reading the log while
# that is happening produced two different digests from one directory -- which
# the determinism gate correctly rejected. Target the actual binaries.
#
# The patterns match the installed binary path (lib/network_bridge/...), which
# this script's own path (test/e2e/run_ros2_e2e.sh) does not contain, so this
# cannot kill the script itself the way a bare `pkill -f network_bridge` did.
for p in "${PIDS[@]}" "$TX_PID" "$RX_PID"; do
  [ -n "$p" ] && kill "$p" 2>/dev/null
done
pkill -f "lib/network_bridge/.*_publisher" 2>/dev/null
pkill -f "lib/network_bridge/network_bridge" 2>/dev/null

# Wait for every bridge/publisher process to actually be gone.
for _ in $(seq 1 30); do
  if ! pgrep -f "lib/network_bridge/" >/dev/null 2>&1; then break; fi
  sleep 1
done
pkill -9 -f "lib/network_bridge/" 2>/dev/null
sleep 1

# Belt and braces: require the WAL byte count to hold steady before reading it.
prev=""; cur=""
for _ in $(seq 1 20); do
  cur=$(du -sb "$WAL" 2>/dev/null | awk '{print $1}')
  [ -n "$prev" ] && [ "$cur" = "$prev" ] && break
  prev="$cur"; sleep 1
done
echo "WAL settled at ${cur} bytes; remaining bridge procs: $(pgrep -cf 'lib/network_bridge/' 2>/dev/null || echo 0)"

echo "### replay validation (twice: the digest must be identical)"
WR="$WS_INSTALL/network_bridge/lib/network_bridge/wal_replay"
"$WR" --dir "$WAL" --digest --verify > "$LOGS/replay1.txt" 2>&1; RC1=$?
"$WR" --dir "$WAL" --digest --verify > "$LOGS/replay2.txt" 2>&1; RC2=$?
cat "$LOGS/replay1.txt"
du -sb "$WAL" > "$LOGS/walsize.txt" 2>/dev/null

# --- gate --------------------------------------------------------------------
D1=$(grep -oE 'digest=[0-9a-f]+' "$LOGS/replay1.txt" | head -1)
D2=$(grep -oE 'digest=[0-9a-f]+' "$LOGS/replay2.txt" | head -1)
FAIL=0
[ "$RC1" -eq 0 ] || { echo "FAIL: replay returned $RC1"; FAIL=1; }
[ -n "$D1" ] && [ "$D1" = "$D2" ] || { echo "FAIL: replay not deterministic ($D1 vs $D2)"; FAIL=1; }
grep -q 'bad=0' "$LOGS/replay1.txt" || { echo "FAIL: replayed records failed frame validation"; FAIL=1; }

for m in sequence_gaps_total missing_sequences_total crc_failures_total frame_rejects_total; do
  v=$(grep -E "^sensorforge_bridge_$m " "$LOGS/metrics_receiver.txt" | awk '{print $2}')
  echo "receiver $m = ${v:-<absent>}"
  [ "${v:-0}" = "0" ] || { echo "FAIL: receiver reported $m=$v"; FAIL=1; }
done

grep -q "/recv/sensors/imu" "$LOGS/topics.txt" || { echo "FAIL: no republished topics"; FAIL=1; }

echo "RESULT: $([ $FAIL -eq 0 ] && echo PASS || echo FAIL)  (vcan0=$CAN_STATUS)"
exit $FAIL
