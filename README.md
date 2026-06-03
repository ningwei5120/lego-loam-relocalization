# LeGO-LOAM + NDT-ICP Relocalization

[![ROS](https://img.shields.io/badge/ROS-Noetic-blue)](https://wiki.ros.org/noetic)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange)](https://ubuntu.com)
[![PCL](https://img.shields.io/badge/PCL-1.10-green)](https://pointclouds.org)

> **建图** (LeGO-LOAM) → **重定位** (Scan Context + NDT + ICP) 完整管线  
> 适配 RoboSense 16线雷达，支持一键建图与实时三维重定位。

---

## 📋 目录

- [功能特性](#功能特性)
- [系统架构](#系统架构)
- [安装](#安装)
- [快速开始](#快速开始)
  - [建图](#一键建图)
  - [重定位](#一键重定位)
- [坐标系说明](#坐标系说明)
- [版本历史](#版本历史)
- [常见问题](#常见问题)

---

## 功能特性

### 建图 (LeGO-LOAM)
- ✅ 地面优化 LiDAR SLAM， lightweight 实时建图
- ✅ 适配 RoboSense 16线雷达（`/rslidar_points` → `/velodyne_points`）
- ✅ 修复无 RViz 订阅时的 PCD 保存崩溃问题
- ✅ 一键建图脚本，自动保存到 `map_results/`

### 重定位 (NDT + ICP + Scan Context)
- ✅ **Scan Context 自动初始化**：无需人工干预，丢失后自动恢复
- ✅ **10Hz 实时**：纯 ICP 跟踪阶段，NDT+ICP 初始化/恢复阶段
- ✅ **默认原点启动**：若车辆起点与 map 原点重合，`--from-origin` 秒级启动
- ✅ **手工选点**：支持命令行脚本或 RViz 2D Pose Estimate
- ✅ **完整可视化**：预配置 RViz，显示全局地图、对齐点云、轨迹、TF

### 实测性能

| 指标 | 数值 |
|------|------|
| Bag 时长 | 208 秒 |
| 点云帧数 | 2,088 帧 (~10 Hz) |
| 建图输出 | `finalCloud.pcd` (737,050 点) |
| 重定位成功率 | **100%** (`--from-origin`) / ~99% (SC 自动) |
| 处理速率 | **10 Hz** |
| SC 数据库 | 272 个地点 |

---

## 系统架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                              建图管线                                    │
│  ROS Bag ──► Adapter ──► LeGO-LOAM ──► PCD 地图                         │
│  /rslidar_points         imageProjection                                 │
│                          featureAssociation                                │
│                          mapOptmization  ──► finalCloud.pcd               │
│                          transformFusion                                   │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                             重定位管线                                   │
│  ROS Bag ──► Adapter ──► ┌──────────────────┐                           │
│  /rslidar_points         │ Scan Context     │ ──► /initialpose          │
│                          │ Auto-Initializer │                           │
│                          └────────┬─────────┘                           │
│                                   │                                     │
│                          ┌────────▼─────────┐                           │
│                          │ NDT+ICP Relocate │                           │
│                          │  - NDT coarse    │                           │
│                          │  - ICP fine      │                           │
│                          └───────┬──────────┘                           │
│                                  │                                      │
│  输出: TF(map→camera_init)  /relocalization/odometry  /relocalization/  │
│        aligned_cloud  global_map  local_map  trajectory_path            │
└─────────────────────────────────────────────────────────────────────────┘
```

### TF 树

```
map ──► camera_init (动态, relocalize节点发布)
         └──► camera (静态 identity)
                └──► base_link (静态 identity)
```

---

## 安装

### 1. 系统要求

| 项目 | 版本 |
|------|------|
| 操作系统 | Ubuntu 20.04 LTS |
| ROS | ROS1 Noetic |
| CMake | >= 3.16 |
| GCC | >= 9.3 |
| RAM | >= 8 GB (16 GB 推荐) |

### 2. 依赖安装

```bash
# ROS Noetic (已安装则跳过)
sudo apt install -y ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash

# 编译工具
sudo apt install -y python3-catkin-tools python3-rosdep build-essential

# PCL / OpenCV / Eigen3 (ROS桌面版已自带)
sudo apt install -y libpcl-dev libopencv-dev libeigen3-dev

# GTSAM (LeGO-LOAM 关键依赖)
sudo apt install -y libgtsam-dev libgtsam-unstable-dev ros-noetic-gtsam
```

### 3. 编译

```bash
cd /path/to/this/project/catkin_ws
catkin_make -j$(nproc)
source devel/setup.bash
```

> 💡 **建议** 将 `source devel/setup.bash` 添加到 `~/.bashrc`。

---

## 快速开始

### 一键建图

```bash
cd /path/to/this/project
bash run_mapping.sh
```

**输出文件**（保存到 `map_results/`）：

| 文件 | 说明 |
|------|------|
| `finalCloud.pcd` | 完整地图（corner + surface + outlier）|
| `cornerMap.pcd` | 角点特征地图 |
| `surfaceMap.pcd` | 面点特征地图 |
| `trajectory.pcd` | 关键帧轨迹 |

### 一键重定位

```bash
cd /path/to/this/project
bash run_relocalization.sh
```

**启动方式选择**：

```bash
# 方式 1: 从原点启动（最快，起点=map原点时推荐）
bash run_relocalization.sh --from-origin

# 方式 2: Scan Context 自动初始化（通用场景）
bash run_relocalization.sh --rate 2.0

# 方式 3: 无显示环境（SSH/服务器）
bash run_relocalization.sh --no-rviz

# 方式 4: 手工指定初始位姿（命令行）
rosrun lego_relocalization publish_initialpose.py --x 10.0 --y 20.0 --yaw 1.57
```

### 手动运行

```bash
# 终端 1: 启动重定位全套
source catkin_ws/devel/setup.bash
roslaunch lego_relocalization relocalization.launch auto_init:=true rviz:=true

# 终端 2: 播放 bag
rosbag play your_bag.bag --clock -r 1.0

# 终端 3: 手工发布初始位姿（可选）
rosrun lego_relocalization publish_initialpose.py --x 0.0 --y 0.0 --yaw 0.0
```

---

## 坐标系说明

LeGO-LOAM 的 `featureAssociation` 节点内部对点云做了隐式坐标交换 `(x,y,z) → (y,z,x)`。

| 阶段 | 坐标系 | 说明 |
|------|--------|------|
| 建图输入 | 原始雷达坐标系 | `rslidar` frame |
| 建图输出 PCD | 交换后坐标系 | `finalCloud.pcd` 中的点为交换后坐标 |
| 重定位输入 | 原始雷达坐标系 | `/velodyne_points` (frame=`camera_init`) |
| 重定位加载地图 | 已反向交换恢复 | `ndt_icp_relocalize::loadGlobalMap()` 中 `(z,x,y)→(x,y,z)` |
| TF 发布 | map → camera_init | 动态位姿，与建图体系一致 |

**重要**：不要手动修改 `ndt_icp_relocalize.cpp` 中的坐标交换逻辑，除非清楚 LeGO-LOAM 内部坐标变换细节。

---

## 版本历史

### v3.1 (2025-06-03) — 当前版本
- 新增 `--from-origin` / `--default-origin` 快速启动模式
- 新增 `publish_initialpose.py` 手工选点脚本
- 参考 SC-LeGO-LOAM：ICP 禁用 RANSAC、FFT 优化 Scan Context 匹配
- 修复 libusb 符号冲突（LD_LIBRARY_PATH 清理）

### v3.0 (2025-06-02)
- Scan Context 自动初始化 + NDT+ICP 重定位
- 自动重恢复机制（LOST → TRACKING）
- 一键脚本 `run_relocalization.sh`

### v2.0
- NDT+ICP 三维重定位核心优化至 10Hz

### v1.0
- LeGO-LOAM 建图 + RoboSense 适配
- 修复 mapOptmization PCD 保存崩溃

---

## 常见问题

### Q1: `undefined symbol: libusb_set_option` 或 GTSAM 符号错误

A: `/opt/MVS/lib/64/libusb-1.0.so.0` 版本过旧。启动前执行：

```bash
export LD_LIBRARY_PATH=$(echo "/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH" | sed 's|/opt/MVS/lib/64:||g; s|/opt/MVS/lib/32:||g; s|:/opt/MVS/lib/64||g; s|:/opt/MVS/lib/32||g')
```

> `run_relocalization.sh` 和 `run_mapping.sh` 已内置此修复。

### Q2: 建图完成后没有 PCD 文件？

A: `mapOptmization` 在 `ros::shutdown()` 后才保存。等待 bag 播放完毕后**额外等待 20-30 秒**。

### Q3: 重定位初始化失败（Fitness 过高）？

A: 检查初始位姿是否大致正确。若起点与 map 原点重合，使用 `--from-origin`。

### Q4: 如何在其他 LiDAR 上使用？

A: 修改 `LeGO-LOAM/include/utility.h`：
- `N_SCAN`：线数
- `useCloudRing`：有 `ring` 字段设 `true`，否则 `false`
- 适配 `lego_loam_adapter/scripts/rslidar_filter_node.py` 中的 topic 名称

---

## 引用

- LeGO-LOAM: [Shan & Englot, IROS 2018](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)
- Scan Context: [Kim & Kim, IROS 2018](https://github.com/irapkaist/scancontext)
- SC-LeGO-LOAM: [Kim, 2020](https://github.com/gisbi-kim/SC-LeGO-LOAM)

---

## License

本项目基于 LeGO-LOAM (BSD-3-Clause) 和 Scan Context 修改，遵循原许可证。
