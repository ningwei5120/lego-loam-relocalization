#!/bin/bash
# =============================================================================
# 三维地图重定位 + 可视化 + 数据集播放 一键脚本
# =============================================================================

set -e

export LD_LIBRARY_PATH=/opt/ros/noetic/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
source /opt/ros/noetic/setup.bash
source /home/hong/lego/catkin_ws/devel/setup.bash

BAG_FILE="/home/hong/lego/nky260525_2026-05-25-16-10-59.bag"
MAP_FILE="/home/hong/lego/map_results/finalCloud.pcd"
PLAY_RATE="1.0"
INIT_X="0.0"
INIT_Y="0.0"
INIT_Z="0.0"
INIT_YAW="0.0"

echo "========================================"
echo "  3D Relocalization + Visualization"
echo "========================================"
echo "Map: $MAP_FILE"
echo "Bag: $BAG_FILE"
echo "Initial guess: [$INIT_X, $INIT_Y, $INIT_Z], yaw=$INIT_YAW"
echo ""

# 1. 清理旧进程
echo "[1/5] Cleaning up..."
for p in $(ps aux | grep -E "roscore|roslaunch|ndt_icp_relocalize|rslidar_filter|rosbag|rviz" | grep -v grep | awk '{print $2}'); do
  kill -9 $p 2>/dev/null || true
done
sleep 2

# 2. 启动 ROS 核心
echo "[2/5] Starting roscore..."
roscore > /tmp/roscore_vis.log 2>&1 &
sleep 3

# 3. 启动重定位 + RViz 可视化
echo "[3/5] Starting relocalization node and RViz..."
roslaunch lego_relocalization relocalization_visualization.launch \
    pcd_map_path:="$MAP_FILE" \
    bag_rate:="$PLAY_RATE" \
    use_sim_time:=true \
    rviz:=true > /tmp/relocalize_vis.log 2>&1 &
LAUNCH_PID=$!
sleep 6

# 4. 发送初始位姿并播放 bag
echo "[4/5] Publishing initial pose and playing bag..."
QZ=$(python3 -c "import math; print(math.sin($INIT_YAW/2))")
QW=$(python3 -c "import math; print(math.cos($INIT_YAW/2))")

rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {position: {x: $INIT_X, y: $INIT_Y, z: $INIT_Z}, orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}},
    covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0.0685, 0, 0, 0, 0, 0, 0, 0.0685, 0, 0, 0, 0, 0, 0, 0.0685]
  }
}" > /dev/null 2>&1

rosbag play "$BAG_FILE" --clock -r "$PLAY_RATE" > /tmp/rosbag_vis.log 2>&1 &
BAG_PID=$!

echo ""
echo "========================================"
echo "Visualization is running!"
echo ""
echo "RViz displays:"
echo "  - Global Map (white):   /relocalization/global_map"
echo "  - Aligned Cloud (green):/relocalization/aligned_cloud"
echo "  - Raw LiDAR (red):      /velodyne_points"
echo "  - TF tree:              map -> base_link"
echo "  - Odometry arrow:       /relocalization/odometry"
echo ""
echo "You can also use RViz '2D Pose Estimate' tool"
echo "to re-trigger relocalization with a new guess."
echo ""
echo "Press Ctrl+C to stop."
echo "========================================"

# 等待 bag 播放完毕或用户中断
wait $BAG_PID
echo ""
echo "[5/5] Bag playback finished."

# 优雅关闭
kill $LAUNCH_PID 2>/dev/null || true
sleep 3
pkill -f roscore 2>/dev/null || true
sleep 2

echo "Done."
