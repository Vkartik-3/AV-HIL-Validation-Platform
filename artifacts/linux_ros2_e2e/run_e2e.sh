set +u
DUR=${DUR:-45}
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
export RCUTILS_LOGGING_BUFFERED_STREAM=0
mkdir -p /out/wal /out/logs
rm -rf /out/wal/* 

# vcan0 for the CAN stream (needs --privileged).
modprobe vcan 2>/dev/null && ip link add dev vcan0 type vcan 2>/dev/null && ip link set up vcan0 2>/dev/null \
  && echo "vcan0 UP" > /out/logs/vcan.txt || echo "vcan0 UNAVAILABLE" > /out/logs/vcan.txt
cat /out/logs/vcan.txt

echo "### starting receiver bridge"
ros2 run network_bridge network_bridge --ros-args -r __node:=sf_receiver \
  --params-file /cfg/receiver_tcp.yaml > /out/logs/receiver.log 2>&1 &
RX=$!
sleep 3
echo "### starting sender bridge"
ros2 run network_bridge network_bridge --ros-args -r __node:=sf_sender \
  --params-file /cfg/sender_tcp.yaml > /out/logs/sender.log 2>&1 &
TX=$!
sleep 3

echo "### starting sensor publishers"
ros2 run network_bridge lidar_publisher  --ros-args -p rate_hz:=10.0  -p topic:=/sensors/lidar  > /out/logs/lidar.log 2>&1 &
ros2 run network_bridge camera_publisher --ros-args -p rate_hz:=30.0  -p topic:=/sensors/camera > /out/logs/camera.log 2>&1 &
ros2 run network_bridge imu_publisher    --ros-args -p rate_hz:=200.0 -p topic:=/sensors/imu    > /out/logs/imu.log 2>&1 &
ros2 run network_bridge gps_publisher    --ros-args -p rate_hz:=20.0  -p topic:=/sensors/gps    > /out/logs/gps.log 2>&1 &
ros2 run network_bridge can_publisher    --ros-args -p rate_hz:=100.0 > /out/logs/can.log 2>&1 &
sleep 5

echo "### counting republished messages on the RECEIVER side for ${DUR}s"
timeout $((DUR+5)) ros2 topic hz /recv/sensors/imu --window 200 > /out/logs/hz_imu.txt 2>&1 &
START=$(date +%s.%N)
sleep "$DUR"
END=$(date +%s.%N)

echo "### scraping bridge metrics"
curl -s --max-time 5 http://127.0.0.1:9101/metrics > /out/logs/metrics_sender.txt   || echo "sender metrics scrape FAILED"
curl -s --max-time 5 http://127.0.0.1:9102/metrics > /out/logs/metrics_receiver.txt || echo "receiver metrics scrape FAILED"

echo "### topic list / echo proof of republication"
ros2 topic list > /out/logs/topics.txt 2>&1
timeout 8 ros2 topic echo /recv/sensors/gps --once > /out/logs/echo_gps.txt 2>&1 || echo "gps echo timed out" >> /out/logs/echo_gps.txt

echo "### stopping"
kill $TX $RX 2>/dev/null; sleep 4
pkill -f "_publisher" 2>/dev/null; pkill -f network_bridge 2>/dev/null; sleep 2

echo "### WAL replay validation"
WALREPLAY=/ws/install/network_bridge/lib/network_bridge/wal_replay
$WALREPLAY --dir /out/wal --digest --verify > /out/logs/replay1.txt 2>&1; echo "replay1 rc=$?"
$WALREPLAY --dir /out/wal --digest --verify > /out/logs/replay2.txt 2>&1; echo "replay2 rc=$?"
cat /out/logs/replay1.txt

awk -v a="$START" -v b="$END" 'BEGIN{printf "DURATION_S=%.3f\n", b-a}' > /out/logs/duration.txt
du -sb /out/wal > /out/logs/walsize.txt
echo "### DONE"
