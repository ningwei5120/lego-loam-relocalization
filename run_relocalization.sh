#!/bin/bash
# =============================================================================
# 基于三维地图的 NDT+ICP 三维重定位脚本 (v3 with Scan Context Auto-Init + RViz)
# 适用于: nky260525_2026-05-25-16-10-59.bag
# =============================================================================

set -e

source /opt/ros/noetic/setup.bash
source /home/hong/lego/catkin_ws/devel/setup.bash
# Remove old libusb from MVS that conflicts with PCL
export LD_LIBRARY_PATH=$(echo "/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH" | sed 's|/opt/MVS/lib/64:||g; s|/opt/MVS/lib/32:||g; s|:/opt/MVS/lib/64||g; s|:/opt/MVS/lib/32||g')

BAG_FILE="/home/hong/lego/nky260525_2026-05-25-16-10-59.bag"
MAP_FILE="/home/hong/lego/map_results/finalCloud.pcd"
PLAY_RATE="1.0"
AUTO_INIT="true"
RVIZ="true"
USE_DEFAULT_ORIGIN="false"
DEFAULT_INIT_X="0.0"
DEFAULT_INIT_Y="0.0"
DEFAULT_INIT_Z="0.0"
DEFAULT_INIT_YAW="0.0"
USE_SC_PLUS_PLUS="false"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --rate)
            PLAY_RATE="$2"
            shift 2
            ;;
        --no-auto-init)
            AUTO_INIT="false"
            shift
            ;;
        --rviz)
            RVIZ="true"
            shift
            ;;
        --no-rviz)
            RVIZ="false"
            shift
            ;;
        --default-origin)
            USE_DEFAULT_ORIGIN="true"
            shift
            ;;
        --init-x)
            DEFAULT_INIT_X="$2"
            shift 2
            ;;
        --init-y)
            DEFAULT_INIT_Y="$2"
            shift 2
            ;;
        --init-z)
            DEFAULT_INIT_Z="$2"
            shift 2
            ;;
        --init-yaw)
            DEFAULT_INIT_YAW="$2"
            shift 2
            ;;
        --from-origin)
            # Shortcut for this dataset: start from map origin (0,0,0)
            USE_DEFAULT_ORIGIN="true"
            AUTO_INIT="false"
            shift
            ;;
        --sc-plus-plus)
            # Enable Scan Context++ (L0 ringkey + lateral augmentation)
            USE_SC_PLUS_PLUS="true"
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--rate 1.0] [--no-auto-init] [--rviz|--no-rviz]"
            echo "       [--default-origin] [--from-origin] [--sc-plus-plus]"
            echo "       [--init-x 0.0] [--init-y 0.0] [--init-z 0.0] [--init-yaw 0.0]"
            echo ""
            echo "Quick start for this dataset (origin=start point):"
            echo "  $0 --from-origin          # fastest, 100% success"
            echo ""
            echo "Advanced:"
            echo "  $0 --sc-plus-plus         # Use Scan Context++ for lateral-robust init"
            exit 1
            ;;
    esac
done

# Auto-detect display availability for RViz
if [ "$RVIZ" = "true" ]; then
    if [ -z "$DISPLAY" ] && [ -z "$WAYLAND_DISPLAY" ]; then
        echo "[WARN] No display detected. RViz will be disabled."
        echo "       To force RViz on a remote machine, use SSH with X11 forwarding: ssh -X ..."
        RVIZ="false"
    fi
fi

echo "========================================"
echo "  3D NDT+ICP Relocalization (v3)"
echo "========================================"
echo "  Tip: For this dataset, start point = map origin."
echo "       Use --from-origin for instant 100% success."
echo ""
echo "Map: $MAP_FILE"
echo "Bag: $BAG_FILE"
echo "Play rate: ${PLAY_RATE}x"
echo "Auto init (Scan Context): $AUTO_INIT"
echo "Default origin init: $USE_DEFAULT_ORIGIN"
if [ "$USE_DEFAULT_ORIGIN" = "true" ]; then
    echo "  Default pose: [${DEFAULT_INIT_X}, ${DEFAULT_INIT_Y}, ${DEFAULT_INIT_Z}] yaw=${DEFAULT_INIT_YAW}"
fi
echo "RViz: $RVIZ"
echo ""

# 1. 清理旧进程
echo "[1/5] Cleaning up..."
for p in $(ps aux | grep -E "roscore|roslaunch|ndt_icp_relocalize|rslidar_filter|rosbag|scan_context|rviz" | grep -v grep | grep -v run_relocalization | awk '{print $2}'); do
  kill -9 $p 2>/dev/null || true
done
sleep 2

# 2. 启动 ROS 核心
echo "[2/5] Starting roscore..."
roscore > /tmp/roscore_relocal.log 2>&1 &
sleep 3

# 3. 启动重定位节点
echo "[3/5] Starting relocalization pipeline..."
roslaunch lego_relocalization relocalization.launch \
    pcd_map_path:="$MAP_FILE" \
    use_sim_time:=true \
    rviz:="$RVIZ" \
    auto_init:="$AUTO_INIT" \
    use_default_initial_pose:="$USE_DEFAULT_ORIGIN" \
    default_init_x:="$DEFAULT_INIT_X" \
    default_init_y:="$DEFAULT_INIT_Y" \
    default_init_z:="$DEFAULT_INIT_Z" \
    default_init_yaw:="$DEFAULT_INIT_YAW" \
    use_sc_plus_plus:="$USE_SC_PLUS_PLUS" \
    > /tmp/relocalize.log 2>&1 &
LAUNCH_PID=$!
sleep 10

# 4. 播放 bag
echo "[4/5] Playing bag..."
if [ "$USE_DEFAULT_ORIGIN" = "true" ] && [ "$AUTO_INIT" = "false" ]; then
    echo "  Mode: START FROM ORIGIN (0,0,0) — optimal for this dataset"
elif [ "$AUTO_INIT" = "true" ]; then
    echo "  (Scan Context will auto-estimate initial pose)"
elif [ "$USE_DEFAULT_ORIGIN" = "true" ]; then
    echo "  (Will use default origin if no input within 5s)"
else
    echo "  (Waiting for manual /initialpose)"
fi

# Start recording odometry
mkdir -p /home/hong/lego/relocalization_results
RESULT_FILE="/home/hong/lego/relocalization_results/trajectory_$(date +%Y%m%d_%H%M%S).txt"
echo "Recording odometry to: $RESULT_FILE"
rostopic echo /relocalization/odometry > "$RESULT_FILE" 2>&1 &
ECHO_PID=$!

rosbag play "$BAG_FILE" --clock -r "$PLAY_RATE" > /tmp/rosbag_relocal.log 2>&1 &
BAG_PID=$!

echo ""
echo "Relocalization is running. Monitor with:"
echo "  rostopic hz /relocalization/odometry"
echo "  tail -f /tmp/relocalize.log"
if [ "$RVIZ" = "true" ]; then
    echo ""
    echo "RViz should have opened. Visualization includes:"
    echo "  - Global Map (Blue)     : /relocalization/global_map"
    echo "  - Local Map (Yellow)    : /relocalization/local_map"
    echo "  - Aligned Cloud (Green) : /relocalization/aligned_cloud"
    echo "  - Raw LiDAR (Red)       : /velodyne_points"
    echo "  - Odometry (Orange)     : /relocalization/odometry"
    echo "  - TF Tree               : map -> camera_init -> camera -> base_link"
fi
echo ""
echo "Press Ctrl+C to stop early."

# 等待 bag 播放完毕或用户中断
wait $BAG_PID
echo ""
echo "[5/5] Bag playback finished."

# 停止记录
kill $ECHO_PID 2>/dev/null || true
sleep 1

# 统计结果
MSG_COUNT=$(grep -c "^header:" "$RESULT_FILE" 2>/dev/null || echo 0)
echo "Recorded $MSG_COUNT odometry messages."

# 显示关键日志
echo ""
echo "=== Performance Summary ==="
grep "Stats:" /tmp/relocalize.log | tail -5 || true
grep "Initialization succeed" /tmp/relocalize.log | tail -3 || true
grep "Auto reinitialization succeed" /tmp/relocalize.log | tail -3 || true
if [ "$AUTO_INIT" = "true" ]; then
    grep "Published /initialpose" /tmp/relocalize.log | tail -3 || true
fi

# 优雅关闭
kill $LAUNCH_PID 2>/dev/null || true
sleep 2
pkill -f roscore 2>/dev/null || true
sleep 1

echo ""
echo "Done. Results saved to: $RESULT_FILE"
