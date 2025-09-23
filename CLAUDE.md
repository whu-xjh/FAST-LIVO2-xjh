# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FAST-LIVO2 is a fast, direct LiDAR-Inertial-Visual odometry system for real-time 3D reconstruction and robotic localization. The system tightly couples LiDAR, IMU, and visual data using EKF-based fusion with a 19-dimensional state vector.

## Build Commands

### Standard Build
```bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

### Clean Build
```bash
cd ~/catkin_ws
catkin_clean
catkin_make
```

## Running the System

### Basic Usage
```bash
# Terminal 1: Launch mapping node
roslaunch fast_livo mapping_avia.launch

# Terminal 2: Play rosbag
rosbag play YOUR_DOWNLOADED.bag
```

### Available Launch Files
- `mapping_avia.launch` - Livox AVIA LiDAR
- `mapping_avia_marslvig.launch` - Livox AVIA with MarsLVIG camera setup
- `mapping_mid360.launch` - Livox Mid360 LiDAR
- `mapping_hap.launch` - Hesai PandarXT32 + Hilti mapping
- `mapping_livox_multi_lidar.launch` - Multiple Livox LiDARs
- `mapping_ouster_ntu.launch` - Ouster LiDAR
- `mapping_hesaixt32_hilti22.launch` - Hesai XT32 + Hilti setup

## Architecture

### Core Components

**Main Processing Pipeline:**
- `LIVMapper.cpp/h` - Main fusion node handling LiDAR-Inertial-Visual odometry (7.2K lines)
- `voxel_map.cpp/h` - LiDAR point cloud management and voxel-based mapping
- `IMU_Processing.cpp/h` - IMU preintegration and bias estimation with Kalman filtering
- `preprocess.cpp/h` - LiDAR point cloud preprocessing and deskewing with motion compensation

**Visual Processing:**
- `vio.cpp` - Visual-inertial odometry with direct image alignment
- `frame.cpp/h` - Visual frame management and feature tracking
- `visual_point.cpp/h` - Visual feature point representation and management

**Multi-LiDAR Support:**
- `livox_multi_lidar.cpp` - Multi-LiDAR data fusion and synchronization for heterogeneous sensors
- `imu_filter.cpp` - IMU data filtering and preprocessing

**Utilities and Math:**
- `utils/so3_math.h` - SO(3) Lie group operations and rotations
- `utils/types.h` - Custom data types and Eigen aliases
- `utils/color.h` - Point cloud coloring utilities
- `common_lib.h` - Common includes and system-wide constants

### Key Data Flow

1. **Sensor Input**: LiDAR points, IMU measurements, and camera images
2. **Preprocessing**: Point cloud deskewing, IMU bias estimation
3. **State Estimation**: EKF-based fusion with 19-dimensional state vector
4. **Mapping**: Voxel-based point cloud management with incremental updates
5. **Visual Alignment**: Direct image alignment for pose refinement

### Important Implementation Details

**High-Frequency IMU Propagation (`imu_rate_odom` parameter):**
- When `uav/imu_rate_odom: true`, enables 250Hz IMU propagation for high-speed scenarios
- Publishes high-frequency odometry on `/LIVO2/imu_propagate` topic
- Reduces position error accumulation by 625x compared to LiDAR-rate updates
- Critical for UAV applications and high-speed ground vehicles (>5 m/s)

**Neighbor Voxel Search Algorithm:**
- Implements adaptive neighbor voxel selection for robust point cloud registration
- Uses `voxel_center_ ± quater_length_` for boundary detection
- Distance-based selection: skips 1 or 2 voxels based on point position relative to voxel boundaries
- Located in `voxel_map.cpp:782-807` within `BuildResidualListOMP` function

**Motion Distortion Compensation:**
- Forward propagation in `IMU_Processing.cpp:300` undistorts point clouds using IMU integration
- Each point is transformed to common reference frame using corresponding IMU state
- Essential for high-speed scenarios where motion occurs during LiDAR scan

### Dependencies

**Required ROS Packages:**
- `roscpp`, `rospy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`
- `tf`, `pcl_ros`, `cv_bridge`, `image_transport`
- `vikit_common`, `vikit_ros` (from specific fork: `https://github.com/xuankuzcr/rpg_vikit.git`)

**External Libraries:**
- PCL (>=1.8) - Point cloud processing
- Eigen3 (>=3.3.4) - Linear algebra
- OpenCV (>=4.2) - Computer vision
- Sophus - Lie group operations
- Boost - Thread management
- LAStools - Point cloud compression (specific paths: `/usr/local/include/LASlib`, `/usr/local/lib/liblaszip.so.8`)

### Key Configuration Files

**Sensor Configurations** in `config/`:
- `avia.yaml` - Livox AVIA configuration
- `mid360.yaml` - Livox Mid360 configuration
- `hap.yaml` - Hesai + Hilti configuration
- `livox_multi_lidar.yaml` - Multi-LiDAR setup
- `camera_*.yaml` - Camera intrinsic/extrinsic parameters

**Important Parameters:**
- `uav/imu_rate_odom: false` - Enable high-frequency IMU propagation
- `uav/gravity_align_en: false` - Gravity alignment enable
- `common/extrinsic_T` - LiDAR to IMU translation
- `common/extrinsic_R` - LiDAR to IMU rotation

## Build System Details

### Architecture-Specific Optimizations
- **ARM (32-bit)**: `-O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math`
- **ARM64 (RK3588, Jetson)**: `-O3 -mcpu=native -mtune=native -ffast-math`
- **x86-64**: `-O2 -march=native` (conservative for stability)
- Automatic detection via CMAKE_SYSTEM_PROCESSOR

### Multithreading Configuration
- OpenMP support enabled when found
- Maximum 4 cores by default (configurable via MP_PROC_NUM)
- Processor count detection via `include(ProcessorCount)`

### Memory Optimization
- Conditional mimalloc support when available
- Release build type by default for performance
- Debug builds use `-O0 -g` flags

## Development Notes

### Code Style and Architecture
- Uses C++17 standard with architecture-specific optimizations
- Multithreading with OpenMP (configurable via MP_PROC_NUM)
- Memory optimization with conditional mimalloc support
- Separate libraries for modularity: `vio`, `lio`, `pre`, `imu_proc`, `laser_mapping`

### Key Data Structures
- `M3D` / `V3D` - 3x3 matrices and 3D vectors (Eigen)
- `PointCloudXYZI` - LiDAR point clouds with intensity
- `PointCloudXYZINormal` - Point clouds with surface normals
- `ImuData` - IMU measurements with timestamps
- `ImageFrame` - Visual frames with features and pose
- `VOXEL_LOCATION` - Discrete 3D voxel coordinates in grid-based map

### Important Constants
- `DIM_STATE`: 19-dimensional state vector for EKF
- `G_m_s2`: Gravity constant (9.81 m/s²)
- `ROOT_DIR`: Project root directory defined at compile time
- Build type defaults to Release for performance

## Common Issues

### Build Issues
- Ensure all dependencies are installed (PCL, Eigen, OpenCV, Sophus, vikit)
- Use specific vikit fork: `https://github.com/xuankuzcr/rpg_vikit.git`
- Verify LAStools library paths for point cloud compression
- Check ARM/x86 architecture detection in CMakeLists.txt

### Runtime Issues
- Verify sensor configuration files match hardware
- Check timestamp synchronization between sensors
- For high-speed scenarios, enable `imu_rate_odom: true`
- Ensure sufficient memory for voxel map operations
- Monitor CPU usage - system is computationally intensive