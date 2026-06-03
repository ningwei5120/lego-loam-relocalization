#!/bin/bash
set -e
source /home/hong/lego/catkin_ws/devel/setup.bash
export LD_LIBRARY_PATH=/opt/ros/noetic/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

echo "[1/5] Cleaning old PCD files..."
rm -f /tmp/*.pcd

echo "[2/5] Starting LeGO-LOAM nodes..."
roslaunch lego_loam_adapter run_rslidar.launch &
ROSLAUNCH_PID=$!

echo "[3/5] Waiting for nodes to initialize..."
sleep 8

echo "[4/5] Playing bag file..."
rosbag play --clock /home/hong/lego/nky260525_2026-05-25-16-10-59.bag

echo "[5/5] Waiting for PCD save and shutting down..."
sleep 25

kill $ROSLAUNCH_PID 2>/dev/null || true
killall -9 rviz imageProjection featureAssociation mapOptmization transformFusion rslidar_filter_node 2>/dev/null || true

mkdir -p /home/hong/lego/map_results
cp /tmp/*.pcd /home/hong/lego/map_results/ 2>/dev/null || true

echo "Done. Generated PCD files:"
ls -lh /home/hong/lego/map_results/*.pcd 2>/dev/null || echo "No PCD files found"
