# Changelog

All notable changes to this project will be documented in this file.

## [3.2.0] - 2025-06-03

### Added
- **Scan Context++ support** (`--sc-plus-plus`)
  - Lateral Augmentation: query cloud shifted ±2m/±4m for lane-level offset robustness
  - L0-norm Ring Key: counts non-zero bins per ring (robust to height variations)
  - Noise filtering: bins with < 5 points set to zero
  - Backward compatible: original Scan Context remains default
- `use_sc_plus_plus` parameter in launch file and initializer node

## [3.1.0] - 2025-06-03

### Added
- `--from-origin` / `--default-origin` quick start mode for relocalization
- `publish_initialpose.py` manual initial pose publisher
- Default initial pose timeout mechanism in `ndt_icp_relocalize`
- Support `--init-x/y/z/yaw` command line arguments in `run_relocalization.sh`

### Changed
- ICP: disabled RANSAC (`setRANSACIterations(0)`) following SC-LeGO-LOAM best practice
- Scan Context distance: FFT-based circulant shift search (13x faster)
- `ros::Rate` → `ros::WallRate` to avoid blocking when sim time is paused
- LD_LIBRARY_PATH cleanup to fix `/opt/MVS/lib/64` libusb conflict

### Fixed
- TF tree: unified to `map → camera_init → camera → base_link`
- Aligned cloud frame_id: set to `map` with `transformPointCloud` for correct RViz display
- `run_relocalization.sh` now properly filters old libusb from LD_LIBRARY_PATH

## [3.0.0] - 2025-06-02

### Added
- Scan Context auto-initializer for global pose estimation
- NDT+ICP two-stage alignment for initialization
- Auto-recovery: LOST → TRACKING via random perturbation reinitialization
- `run_relocalization.sh` one-click script with `--rate`, `--rviz`, `--no-auto-init` options
- Pre-configured RViz layout for relocalization visualization

## [2.0.0] - 2025-05

### Added
- NDT+ICP 3D relocalization core (C++)
- Real-time tracking at ~10 Hz
- `/relocalization/odometry`, `/tf`, `/relocalization/aligned_cloud` topics

## [1.0.0] - 2025-05

### Added
- LeGO-LOAM mapping with RoboSense 16-line LiDAR adapter
- `run_mapping.sh` one-click mapping script
- Fixed `mapOptmization.cpp` PCD save crash when no RViz subscriber
