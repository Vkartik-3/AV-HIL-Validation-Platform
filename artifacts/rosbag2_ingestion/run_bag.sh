set +u
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
mkdir -p /out/bag /out/logs
rm -rf /out/bag/* /out/wal2; mkdir -p /out/wal2

echo "### 1. RECORD: capture live sensor streams into a rosbag2"
ros2 run network_bridge lidar_publisher  --ros-args -p rate_hz:=10.0  -p topic:=/sensors/lidar  > /dev/null 2>&1 &
ros2 run network_bridge camera_publisher --ros-args -p rate_hz:=30.0  -p topic:=/sensors/camera > /dev/null 2>&1 &
ros2 run network_bridge imu_publisher    --ros-args -p rate_hz:=200.0 -p topic:=/sensors/imu    > /dev/null 2>&1 &
ros2 run network_bridge gps_publisher    --ros-args -p rate_hz:=20.0  -p topic:=/sensors/gps    > /dev/null 2>&1 &
sleep 4
timeout 20 ros2 bag record -o /out/bag/sf_sample \
  /sensors/lidar /sensors/camera /sensors/imu /sensors/gps > /out/logs/bag_record.log 2>&1
pkill -f "_publisher"; sleep 2
echo "--- bag info ---"
ros2 bag info /out/bag/sf_sample 2>&1 | tee /out/logs/bag_info.txt | head -20

echo "### 2. REPLAY the recorded bag through the SensorForge bridge"
sed 's|/out/wal|/out/wal2|' /cfg/sender_tcp.yaml > /tmp/sender_bag.yaml
ros2 run network_bridge network_bridge --ros-args -r __node:=sf_receiver \
  --params-file /cfg/receiver_tcp.yaml > /out/logs/bag_receiver.log 2>&1 &
sleep 3
ros2 run network_bridge network_bridge --ros-args -r __node:=sf_sender \
  --params-file /tmp/sender_bag.yaml > /out/logs/bag_sender.log 2>&1 &
sleep 4
# Play the RECORDED data back; the bridge sees it exactly as live sensor input.
ros2 bag play /out/bag/sf_sample > /out/logs/bag_play.log 2>&1
sleep 4
curl -s --max-time 5 http://127.0.0.1:9101/metrics > /out/logs/bag_metrics_sender.txt
ros2 topic list > /out/logs/bag_topics.txt 2>&1
pkill -f network_bridge; sleep 3

echo "### 3. VALIDATE the WAL written from replayed bag data"
/ws/install/network_bridge/lib/network_bridge/wal_replay --dir /out/wal2 --digest --verify \
  > /out/logs/bag_replay_validate.txt 2>&1
cat /out/logs/bag_replay_validate.txt
echo "### DONE"
