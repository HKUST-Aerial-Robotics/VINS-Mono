# VINS-Mono → ROS2 Migration Plan

## Target Environment
- **ROS2 Distro**: Jazzy Jalisco (installed at `/opt/ros/jazzy`)
- **Hardware**: Jetson Orin NX (aarch64, 8 cores, 9.5 GB RAM)
- **Build System**: `colcon build --symlink-install` in `~/ros2_ws`
- **C++ Standard**: C++17 (required by ROS2 Jazzy / rclcpp)

---

## 1. Key Findings from Analysis

### 1.1 Environment Status
| Component | Status |
|-----------|--------|
| ROS2 Jazzy | ✅ Installed |
| rclcpp, sensor_msgs, nav_msgs, geometry_msgs, std_msgs, visualization_msgs | ✅ Available |
| tf2_ros, tf2_geometry_msgs, tf2_eigen | ✅ Available |
| cv_bridge, image_transport, message_filters | ✅ Available |
| ament_cmake, ament_index_cpp | ✅ Available |
| Ceres 2.2.0, OpenCV 4.6, Eigen 3.4, Boost 1.83, yaml-cpp 0.8 | ✅ Available |
| `sensor_msgs::msg::PointCloud` + `ChannelFloat32` | ✅ Available (deprecated but functional) |

### 1.2 Critical Decision: PointCloud Message
`sensor_msgs::msg::PointCloud` is **deprecated but still available** in ROS2 Jazzy.
**Decision**: Use `sensor_msgs::msg::PointCloud` as-is for initial migration to minimize code changes. This avoids rewriting all the ChannelFloat32 encoding/decoding logic. Can migrate to PointCloud2 or custom messages later if needed.

### 1.3 Package Summary (6 packages)
| Package | ROS API Density | Migration Effort |
|---------|----------------|-----------------|
| `camera_model` | Minimal (library) | Low — mostly CMake changes |
| `feature_tracker` | High | Medium |
| `vins_estimator` | Very High | High — most complex |
| `pose_graph` | Very High | High — includes ThirdParty libs |
| `benchmark_publisher` | Medium | Low — simplest node |
| `vins_bringup` | Low | Low — launch-only package |

---

## 2. Migration Architecture

### 2.1 New Package Structure
```
~/ros2_ws/src/VINS-Mono-inno/
├── vins_msgs/                    # NEW: shared custom messages (optional, future)
├── camera_model/                 # Library + calibration tool
├── feature_tracker/              # Node
├── vins_estimator/               # Node (main estimator)
├── pose_graph/                   # Node (loop closure)
├── benchmark_publisher/          # Node (benchmark)
├── vins_bringup/                 # Launch package
├── config/                       # Shared config (installed to share dir)
├── support_files/                # Shared support files (installed to share dir)
└── ...                           # Remaining repo files
```

### 2.2 Dependency Graph (build order)
```
camera_model (library, no ROS deps)
    ↓
feature_tracker, vins_estimator, pose_graph (depend on camera_model)
    ↓
benchmark_publisher (independent)
    ↓
vins_bringup (launch files, depends on all nodes)
```

---

## 3. Per-Package Migration Steps

### 3.1 `camera_model` (Library Package)

**Changes needed:**
1. **package.xml**: format 3, `<buildtool_depend>ament_cmake</buildtool_depend>`
2. **CMakeLists.txt**:
   - Remove `catkin` macros
   - Use `add_library(camera_model ...)` + `ament_export_targets(export_camera_model HAS_LIBRARY_TARGET)`
   - Install headers to `include/camodocal/`
   - Install library
3. **Source code**: No changes needed (pure C++ with Ceres/OpenCV/Boost)
4. **Dependencies**: Remove `roscpp`, `std_msgs` (unused in library code)

### 3.2 `feature_tracker`

**package.xml changes:**
```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<depend>rclcpp</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
<depend>cv_bridge</depend>
<depend>camera_model</depend>
```

**CMakeLists.txt changes:**
- `find_package(ament_cmake REQUIRED)`
- `find_package(rclcpp REQUIRED)`
- `find_package(sensor_msgs REQUIRED)`
- `find_package(cv_bridge REQUIRED)`
- `find_package(camera_model REQUIRED)`
- `add_executable(feature_tracker_node src/feature_tracker_node.cpp ...)`
- `ament_target_dependencies(feature_tracker_node rclcpp sensor_msgs cv_bridge camera_model)`
- `install(TARGETS feature_tracker_node DESTINATION lib/${PROJECT_NAME})`

**Source code changes:**

`feature_tracker_node.cpp`:
```cpp
// ROS1 → ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "std_msgs/msg/bool.hpp"
#include "cv_bridge/cv_bridge.h"

// Node class instead of global callbacks
class FeatureTrackerNode : public rclcpp::Node {
    // subscriptions, publishers as members
    // img_callback as member function
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FeatureTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
```

`parameters.cpp`:
```cpp
// readParam<T>(n, "key") → node->declare_parameter<T>("key", default)
// cv::FileStorage usage stays unchanged
```

**Key API mappings:**
| ROS1 | ROS2 |
|------|------|
| `ros::init(argc, argv, "name")` | `rclcpp::init(argc, argv)` |
| `ros::NodeHandle n("~")` | `rclcpp::Node` (use `~` via remapping or NodeOptions) |
| `n.subscribe(topic, q, cb)` | `node->create_subscription<T>(topic, qos, cb)` |
| `n.advertise<T>(topic, q)` | `node->create_publisher<T>(topic, qos)` |
| `pub.publish(msg)` | `pub->publish(msg)` |
| `ros::spin()` | `rclcpp::spin(node)` |
| `ros::Time::now().toSec()` | `node->now().seconds()` |
| `ROS_INFO(...)` | `RCLCPP_INFO(node->get_logger(), ...)` |
| `ROS_WARN_THROTTLE(2.0, ...)` | `RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000ms, ...)` |
| `sensor_msgs::ImageConstPtr` | `sensor_msgs::msg::Image::ConstSharedPtr` |
| `sensor_msgs::PointCloud` | `sensor_msgs::msg::PointCloud` |
| `cv_bridge::toCvCopy` | `cv_bridge::toCvCopy` (same API) |

### 3.3 `vins_estimator` (Most Complex)

**Major changes:**
1. **estimator_node.cpp**: Convert to `rclcpp::Node` class
   - 5 subscribers, 12+ publishers
   - `ros::TransportHints().tcpNoDelay()` → `rclcpp::QoS` with `BestEffort` reliability
   - `std::thread measurement_process` stays the same
   - `std::mutex` / `std::condition_variable` stays the same
2. **estimator.h**: `std_msgs::Header` → `std_msgs::msg::Header`
3. **estimator.cpp**: `ros::Time::now().toSec()` → pass clock or use `rclcpp::Clock`
4. **visualization.cpp**:
   - `tf::TransformBroadcaster` → `tf2_ros::TransformBroadcaster`
   - `tf::Transform` → `geometry_msgs::msg::TransformStamped`
   - All publishers → `rclcpp::Publisher<...>::SharedPtr`
5. **CameraPoseVisualization.cpp**: `ros::Publisher` → `rclcpp::Publisher`
6. **parameters.cpp**: Same pattern as feature_tracker

**QoS for IMU subscriber:**
```cpp
rclcpp::QoS imu_qos(2000);
imu_qos.best_effort();  // replaces tcpNoDelay
auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(
    IMU_TOPIC, imu_qos, imu_callback);
```

**TF broadcasting:**
```cpp
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

// In visualization:
static tf2_ros::TransformBroadcaster br(node);
geometry_msgs::msg::TransformStamped transform;
transform.header.stamp = node->now();
transform.header.frame_id = "world";
transform.child_frame_id = "body";
// ... set translation/rotation
br.sendTransform(transform);
```

### 3.4 `pose_graph`

**Major changes:**
1. **pose_graph_node.cpp**: Convert to `rclcpp::Node` class
   - 7 subscribers, 5+ publishers
   - `ros::package::getPath("pose_graph")` → `ament_index_cpp::get_package_share_directory("pose_graph")`
   - `std::thread process()` and `std::thread command()` stay the same
2. **pose_graph.cpp**:
   - TF broadcaster → `tf2_ros::TransformBroadcaster`
   - All publishers → `rclcpp::Publisher`
   - `ros::Time(double)` → `rclcpp::Time(double * 1e9)` or manual conversion
3. **ThirdParty/** (DBoW, DUtils, DVision, VocabularyBinary): **No changes** — pure C++
4. **Support files**: Install `brief_k10L6.bin` and `brief_pattern.yml` to share directory
5. **parameters.h**: Remove `ros/ros.h`, update includes

**Support files installation:**
```cmake
install(DIRECTORY ../support_files/
    DESTINATION share/${PROJECT_NAME}/support_files
    FILES_MATCHING PATTERN "*.bin" PATTERN "*.yml")
```

### 3.5 `ar_demo` (removed from current scope)

This package is intentionally out of scope for the current migration and has been removed from the repository tree.

### 3.6 `benchmark_publisher`

**Changes:** Minimal — convert to `rclcpp::Node`, remove unused `tf` dependency.

### 3.7 `data_generator` (removed from current scope)

This package is intentionally out of scope for the current migration and has been removed from the repository tree.

---

## 4. Launch File Migration

All VINS launch XML files → `.launch.py` (Python) in new `vins_bringup` package.

**Example ROS2 launch:**
```python
# vins_bringup/launch/euroc.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('vins_bringup'), 'config', 'euroc', 'euroc_config.yaml')
    
    return LaunchDescription([
        Node(
            package='feature_tracker',
            executable='feature_tracker_node',
            name='feature_tracker',
            output='screen',
            parameters=[{
                'config_file': config_path,
                'vins_folder': os.path.dirname(config_path),
            }],
            remappings=[],
        ),
        Node(
            package='vins_estimator',
            executable='vins_estimator_node',
            name='vins_estimator',
            output='screen',
            parameters=[{
                'config_file': config_path,
                'vins_folder': os.path.dirname(config_path),
            }],
        ),
        Node(
            package='pose_graph',
            executable='pose_graph_node',
            name='pose_graph',
            output='screen',
            parameters=[{
                'config_file': config_path,
                'visualization_shift_x': 0,
                'visualization_shift_y': 0,
                'skip_cnt': 0,
                'skip_dis': 0.0,
            }],
        ),
    ])
```

---

## 5. Config & Support Files Installation

**New `vins_bringup` package** will install shared resources:
```cmake
install(DIRECTORY ../config/
    DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY ../support_files/
    DESTINATION share/${PROJECT_NAME}/support_files)
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch)
```

---

## 6. Jetson Orin NX Optimization Notes

### 6.1 Build Optimization
- Use `-O2` or `-O3` optimization (already in original CMakeLists)
- Enable NEON SIMD: `-mfpu=neon` (arm64 has NEON by default)
- Use `-j$(nproc)` for parallel build (8 cores available)
- Consider `-flto` for link-time optimization

### 6.2 Runtime Optimization
- **QoS profiles**: Use `BestEffort` for sensor data (IMU, images) to reduce overhead
- **Thread affinity**: Original `taskset -c N` in black_box.launch can be preserved
- **Memory**: 9.5 GB RAM — sufficient for VINS-Mono
- **Feature count**: Already optimized in this fork (150-200 features)

### 6.3 CMake Optimization Flags
```cmake
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG -fopenmp")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
```

---

## 7. Migration Order (Recommended)

1. **camera_model** — library, no ROS deps, foundation for others
2. **benchmark_publisher** — simplest node, good test of build system
3. **feature_tracker** — medium complexity, core pipeline
4. **vins_estimator** — most complex, depends on camera_model
5. **pose_graph** — complex, depends on camera_model + ThirdParty
6. **vins_bringup** — launch files, depends on all nodes

---

## 8. Common ROS1 → ROS2 API Mapping (Quick Reference)

| ROS1 | ROS2 |
|------|------|
| `#include <ros/ros.h>` | `#include "rclcpp/rclcpp.hpp"` |
| `#include <sensor_msgs/Image.h>` | `#include "sensor_msgs/msg/image.hpp"` |
| `#include <nav_msgs/Odometry.h>` | `#include "nav_msgs/msg/odometry.hpp"` |
| `#include <geometry_msgs/Point.h>` | `#include "geometry_msgs/msg/point.hpp"` |
| `#include <std_msgs/Bool.h>` | `#include "std_msgs/msg/bool.hpp"` |
| `#include <visualization_msgs/Marker.h>` | `#include "visualization_msgs/msg/marker.hpp"` |
| `#include <tf/transform_broadcaster.h>` | `#include "tf2_ros/transform_broadcaster.h"` |
| `#include <cv_bridge/cv_bridge.h>` | `#include "cv_bridge/cv_bridge.h"` (same) |
| `ros::init(argc, argv, "node")` | `rclcpp::init(argc, argv)` |
| `ros::NodeHandle n("~")` | `rclcpp::Node::make_shared("node")` |
| `n.getParam("key", var)` | `node->declare_parameter("key", default); node->get_parameter("key", var)` |
| `n.subscribe(topic, q, cb)` | `node->create_subscription<T>(topic, QoS, cb)` |
| `n.advertise<T>(topic, q)` | `node->create_publisher<T>(topic, QoS)` |
| `ros::spin()` | `rclcpp::spin(node)` |
| `ros::ok()` | `rclcpp::ok()` |
| `ros::Rate r(hz)` | `rclcpp::Rate r(hz)` |
| `ros::Time::now().toSec()` | `node->now().seconds()` |
| `ros::Time(t).toSec()` | `rclcpp::Time(t).seconds()` |
| `ros::Duration(d).sleep()` | `rclcpp::sleep_for(std::chrono::duration<double>(d))` |
| `ROS_INFO(...)` | `RCLCPP_INFO(rclcpp::get_logger("name"), ...)` |
| `ROS_WARN(...)` | `RCLCPP_WARN(rclcpp::get_logger("name"), ...)` |
| `ROS_DEBUG(...)` | `RCLCPP_DEBUG(rclcpp::get_logger("name"), ...)` |
| `ROS_ASSERT(cond)` | `assert(cond)` |
| `ROS_BREAK()` | `std::abort()` |
| `sensor_msgs::ImageConstPtr` | `sensor_msgs::msg::Image::ConstSharedPtr` |
| `sensor_msgs::PointCloudPtr` | `sensor_msgs::msg::PointCloud::SharedPtr` |
| `msg->header.stamp.toSec()` | `rclcpp::Time(msg->header.stamp).seconds()` |
| `msg->header.stamp = ros::Time(t)` | `msg->header.stamp = rclcpp::Time(t).to_msg()` or manual |
| `ros::package::getPath("pkg")` | `ament_index_cpp::get_package_share_directory("pkg")` |
| `catkin_package(...)` | `ament_export_*` macros |
| `add_dependencies(target catkin_package)` | Not needed in ament |

---

## 9. Potential Issues & Mitigations

| Issue | Mitigation |
|-------|------------|
| `sensor_msgs::msg::PointCloud` deprecated | Works in Jazzy; plan future migration to PointCloud2 |
| `ros::Time::now()` in library code (estimator.cpp) | Pass `rclcpp::Clock` or node pointer |
| `std_msgs::Header` in estimator.h | Change to `std_msgs::msg::Header` |
| TF static broadcaster in visualization.cpp | Make it a class member or shared pointer |
| `ros::TransportHints().tcpNoDelay()` | Use `rclcpp::QoS` with `BestEffort` |
| Global callback functions with global state | Convert to class member functions |
| `ROS_WARN_THROTTLE(rate, ...)` | `RCLCPP_WARN_THROTTLE(logger, clock, ms, ...)` |
| Support files path (`pkg_path + "/../support_files/"`) | Install to share dir, use `get_package_share_directory` |
| `cv::FileStorage` YAML with `!!opencv-matrix` | Works unchanged (ROS-independent) |
| Multi-threaded spin + processing thread | Keep `std::thread` pattern, use `rclcpp::spin` in main |
