#!/bin/bash
# =============================================================================
# 基于 rslidar 点云的 LeGO-LOAM 一键建图脚本
# 数据包: /home/hong/lego/nky260525_2026-05-25-16-10-59.bag
# =============================================================================

set -e

# 0. 解决动态库冲突 (MVS libusb / GTSAM 版本问题)
export LD_LIBRARY_PATH=/opt/ros/noetic/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

source /opt/ros/noetic/setup.bash
source /home/hong/lego/catkin_ws/devel/setup.bash

# 再次确保 LD_LIBRARY_PATH 在前（setup.bash 可能覆盖）
export LD_LIBRARY_PATH=/opt/ros/noetic/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

BAG_FILE="/home/hong/lego/nky260525_2026-05-25-16-10-59.bag"
# BAG_FILE="/home/hong/lego/mihoutao_2025-09-24-16-51-56.bag"

RESULT_DIR="/home/hong/lego/map_results"
PLAY_RATE="7.0"   # 播放倍速，可根据需要调整

# 1. 清理旧进程与旧结果
echo "[1/5] 清理环境..."
for p in $(ps aux | grep -E "roscore|roslaunch|mapOptm|imageProj|featureAssoc|transformFus|rslidar_filter|rosbag" | grep -v grep | awk '{print $2}'); do
  kill -9 $p 2>/dev/null || true
done
sleep 2

rm -f /tmp/*.pcd
mkdir -p "$RESULT_DIR"

# 2. 启动 ROS 核心与 LeGO-LOAM
echo "[2/5] 启动 LeGO-LOAM..."
roscore > /tmp/roscore.log 2>&1 &
sleep 3

roslaunch lego_loam_adapter run_rslidar.launch > /tmp/lego_loam.log 2>&1 &
LAUNCH_PID=$!
sleep 5

# 3. 播放 bag
echo "[3/5] 开始播放数据包 (倍率 ${PLAY_RATE}x)..."
rosbag play "$BAG_FILE" --clock -r "$PLAY_RATE" > /tmp/rosbag.log 2>&1 &
BAG_PID=$!

# 等待 bag 播放完毕
while kill -0 $BAG_PID 2>/dev/null; do
    sleep 2
done
echo "[4/5] 数据包播放完毕，等待后端优化与地图构建..."
sleep 15

# 4. 优雅关闭节点，触发 PCD 自动保存
echo "[5/5] 保存地图并退出..."
rosnode kill -a 2>/dev/null || true
sleep 5

# 关闭 roscore
pkill -f roscore 2>/dev/null || true
sleep 2

# 5. 复制结果到指定目录
if ls /tmp/*.pcd 1> /dev/null 2>&1; then
    cp /tmp/*.pcd "$RESULT_DIR/"
    echo ""
    echo "============================================"
    echo "建图完成！生成的地图文件:"
    ls -lh "$RESULT_DIR"/*.pcd
    echo "============================================"
else
    echo "警告: 未找到 PCD 文件，建图可能失败。"
    echo "请检查 /tmp/lego_loam.log 获取详细日志。"
fi

echo ""
echo "查看地图命令示例:"
echo "  pcl_viewer ${RESULT_DIR}/finalCloud.pcd"
echo "  python3 -c \"import open3d as o3d; o3d.visualization.draw_geometries([o3d.io.read_point_cloud('${RESULT_DIR}/finalCloud.pcd')])\""
