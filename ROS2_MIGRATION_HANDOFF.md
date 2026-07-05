# VINS-Mono ROS1 → ROS2 Migration Handoff Document

## Для агента-преемника

Этот документ содержит полную информацию о состоянии миграции VINS-Mono из ROS1 в ROS2 Jazzy на Jetson Orin NX. Передавайся от одного агента к другому для продолжения работы.

---

## 1. Окружение

| Параметр | Значение |
|-----------|----------|
| **ROS2** | Jazzy Jalisco (`/opt/ros/jazzy`) |
| **Hardware** | Jetson Orin NX, aarch64, 8 ядер, 9.5 GB RAM |
| **Workspace** | `~/ros2_ws` |
| **Source** | `~/ros2_ws/src/VINS-Mono-inno` |
| **Git branch** | `ros2` (ответвлён от `UAV_perfom`) |
| **Build command** | `cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash 2>/dev/null; colcon build --symlink-install --packages-select <PKG> --base-paths src` |
| **C++ standard** | C++17 (обязательно для ROS2 Jazzy) |

### Системные библиотеки (все установлены)
- Ceres 2.2.0 (`libceres-dev`) — **Важно: `ceres::LocalParameterization` удалён, используется `ceres::Manifold`**
- OpenCV 4.6 (`libopencv-dev`)
- Eigen 3.4 (`libeigen3-dev`)
- Boost 1.83 (`libboost-all-dev`)
- yaml-cpp 0.8

### ROS2 пакеты (все установлены)
- `rclcpp`, `std_msgs`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `visualization_msgs`
- `tf2_ros`, `tf2_geometry_msgs`, `tf2_eigen`
- `cv_bridge` (header: `cv_bridge/cv_bridge.hpp`, не `.h`!)
- `image_transport`, `message_filters`
- `ament_cmake`, `ament_index_cpp`

### Важно: `sensor_msgs::msg::PointCloud` + `ChannelFloat32`
В ROS2 Jazzy эти сообщения **доступны, но deprecated**. Решено использовать их как есть для минимизации изменений кода.

---

## 2. Структура пакетов (6 пакетов)

```
~/ros2_ws/src/VINS-Mono-inno/
├── camera_model/          # ✅ ГОТОВО — библиотека + Calibration tool
├── benchmark_publisher/  # ✅ ГОТОВО — простейший узел
├── feature_tracker/       # ✅ ГОТОВО — трекинг фич
├── vins_estimator/        # ✅ ГОТОВО — основной оценщик (самый сложный)
├── pose_graph/            # ✅ ГОТОВО — loop closure + pose graph
├── vins_bringup/          # ✅ ГОТОВО — launch-пакет
├── config/                # Общие YAML конфиги (не требуют изменений)
├── support_files/         # brief_k10L6.bin, brief_pattern.yml
├── ROS2_MIGRATION_PLAN.md # Общий план миграции
└── ROS2_MIGRATION_HANDOFF.md # Этот документ
```

### Статус сборки
| Пакет | Статус | Собран |
|-------|--------|--------|
| `camera_model` | ✅ Полностью мигрирован | Да |
| `benchmark_publisher` | ✅ Полностью мигрирован | Да |
| `feature_tracker` | ✅ Полностью мигрирован | Да |
| `vins_estimator` | ✅ Полностью мигрирован | Да |
| `pose_graph` | ✅ Полностью мигрирован | Да |
| `vins_bringup` | ✅ Создан и собран | Да |

`ar_demo` и `data_generator` намеренно исключены из текущего scope и удалены из дерева репозитория.

---

## 3. Ключевые архитектурные решения (уже реализованные)

### 3.1 Ceres 2.2 Compatibility Shim
Файл: `camera_model/include/camodocal/gpl/ceres_local_parameterization_compat.h`

Ceres 2.2 полностью удалил `ceres::LocalParameterization`. Создан shim-класс, который наследует `ceres::Manifold` и предоставляет старый API `LocalParameterization` (`Plus`, `ComputeJacobian`, `GlobalSize`, `LocalSize`).

**Использование в коде:**
```cpp
#include "camodocal/gpl/ceres_local_parameterization_compat.h"

// Старый код продолжает работать:
class PoseLocalParameterization : public ceres::LocalParameterization { ... };

// Но тип указателя меняется:
ceres::Manifold* local_parameterization = new PoseLocalParameterization();  // НЕ LocalParameterization*
problem.SetManifold(para_Pose[i], local_parameterization);  // НЕ AddParameterBlock(..., local_parameterization)
```

**Также доступно:**
```cpp
// ceres::QuaternionManifold() — встроенный в Ceres 2.2
ceres::Manifold* q_manifold = new ceres::QuaternionManifold();

// AutoDiffManifold заменяет AutoDiffLocalParameterization
static ceres::Manifold* Create() {
    return new ceres::AutoDiffManifold<AngleLocalParameterization, 1, 1>();
}
```

### 3.2 Helper функция для timestamp
Добавлена в `vins_estimator/src/utility/utility.h`:
```cpp
inline double stampToSec(const builtin_interfaces::msg::Time &stamp) {
    return (double)stamp.sec + (double)stamp.nanosec * 1e-9;
}
```
Используется вместо `msg->header.stamp.toSec()`.

### 3.3 Паттерн миграции ROS1 → ROS2 (уже отработанный)

#### Includes
```cpp
// ROS1                                    → ROS2
#include <ros/ros.h>                       → #include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/Image.h>             → #include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/PointCloud.h>        → #include "sensor_msgs/msg/point_cloud.hpp"
#include <sensor_msgs/Imu.h>               → #include "sensor_msgs/msg/imu.hpp"
#include <sensor_msgs/image_encodings.h>   → #include "sensor_msgs/image_encodings.hpp"
#include <std_msgs/Bool.h>                 → #include "std_msgs/msg/bool.hpp"
#include <std_msgs/Header.h>               → #include "std_msgs/msg/header.hpp"
#include <nav_msgs/Odometry.h>             → #include "nav_msgs/msg/odometry.hpp"
#include <nav_msgs/Path.h>                 → #include "nav_msgs/msg/path.hpp"
#include <geometry_msgs/PointStamped.h>     → #include "geometry_msgs/msg/point_stamped.hpp"
#include <geometry_msgs/Point32.h>         → #include "geometry_msgs/msg/point32.hpp"
#include <geometry_msgs/Point.h>           → #include "geometry_msgs/msg/point.hpp"
#include <geometry_msgs/TwistStamped.h>    → #include "geometry_msgs/msg/twist_stamped.hpp"
#include <visualization_msgs/Marker.h>      → #include "visualization_msgs/msg/marker.hpp"
#include <visualization_msgs/MarkerArray.h> → #include "visualization_msgs/msg/marker_array.hpp"
#include <std_msgs/ColorRGBA.h>            → #include "std_msgs/msg/color_rgba.hpp"
#include <tf/transform_broadcaster.h>       → #include "tf2_ros/transform_broadcaster.h"
#include <cv_bridge/cv_bridge.h>           → #include "cv_bridge/cv_bridge.hpp"  // ВАЖНО: .hpp, не .h!
```

#### API замены
```cpp
// ROS1                                    → ROS2
ros::init(argc, argv, "node")              → rclcpp::init(argc, argv)
ros::NodeHandle n("~")                     → auto node = std::make_shared<rclcpp::Node>("node")
ros::console::set_logger_level(...)        → (не нужно, rclcpp по умолчанию Info)
readParameters(n)                          → readParameters(node)  // signature: rclcpp::Node::SharedPtr

// Parameters
n.getParam("key", var)                    → node->declare_parameter<T>("key", default); node->get_parameter("key", var)
readParam<T>(n, "key")                    → (см. parameters.cpp в feature_tracker/vins_estimator для шаблона)

// Subscribers
ros::Subscriber sub = n.subscribe(topic, q, cb)
  → auto sub = node->create_subscription<MsgType>(topic, q, cb)

// Publishers
ros::Publisher pub = n.advertise<MsgType>(topic, q)
  → rclcpp::Publisher<MsgType>::SharedPtr pub = node->create_publisher<MsgType>(topic, q)
pub.publish(msg)                           → pub->publish(msg)  // для shared_ptr: pub->publish(*msg)

// Spin
ros::spin()                                → rclcpp::spin(node)
ros::ok()                                  → rclcpp::ok()

// Time
ros::Time::now().toSec()                   → node->now().seconds()
msg->header.stamp.toSec()                  → stampToSec(msg->header.stamp)  // или rclcpp::Time(msg->header.stamp).seconds()
ros::Time(double_t)                         → см. helper ниже

// Logging
ROS_INFO(...)                              → RCLCPP_INFO(rclcpp::get_logger("node_name"), ...)
ROS_WARN(...)                              → RCLCPP_WARN(rclcpp::get_logger("node_name"), ...)
ROS_DEBUG(...)                             → RCLCPP_DEBUG(rclcpp::get_logger("node_name"), ...)
ROS_INFO_STREAM(...)                       → RCLCPP_INFO_STREAM(rclcpp::get_logger("node_name"), ...)
ROS_ASSERT(cond)                           → assert(cond)
ROS_BREAK()                                → std::abort()
ROS_WARN_THROTTLE(2.0, ...)                → RCLCPP_WARN_SKIPFIRST_THROTTLE(logger, clock, 2000, ...)

// Message types
sensor_msgs::ImageConstPtr                 → sensor_msgs::msg::Image::ConstSharedPtr
sensor_msgs::PointCloudConstPtr            → sensor_msgs::msg::PointCloud::ConstSharedPtr
sensor_msgs::PointCloudPtr                 → sensor_msgs::msg::PointCloud::SharedPtr
nav_msgs::Odometry::ConstPtr               → nav_msgs::msg::Odometry::ConstSharedPtr
std_msgs::BoolConstPtr                     → std_msgs::msg::Bool::ConstSharedPtr
geometry_msgs::TwistStamped::ConstPtr      → geometry_msgs::msg::TwistStamped::ConstSharedPtr
sensor_msgs::ChannelFloat32                → sensor_msgs::msg::ChannelFloat32
geometry_msgs::Point32                    → geometry_msgs::msg::Point32
geometry_msgs::Point                       → geometry_msgs::msg::Point
nav_msgs::Path                             → nav_msgs::msg::Path
nav_msgs::Odometry                         → nav_msgs::msg::Odometry
visualization_msgs::Marker                 → visualization_msgs::msg::Marker
visualization_msgs::MarkerArray            → visualization_msgs::msg::MarkerArray
std_msgs::Header                          → std_msgs::msg::Header
std_msgs::ColorRGBA                        → std_msgs::msg::ColorRGBA
ros::Duration()                            → builtin_interfaces::msg::Duration()

// TF
tf::TransformBroadcaster                   → tf2_ros::TransformBroadcaster
tf::Transform                              → geometry_msgs::msg::TransformStamped
tf::StampedTransform(...)                  → (заполнить TransformStamped полями)
br.sendTransform(StampedTransform(...))     → br->sendTransform(transform_stamped)

// Package path
ros::package::getPath("pkg")               → ament_index_cpp::get_package_share_directory("pkg")

// cv_bridge
cv_bridge::toCvCopy(msg, encoding)         → cv_bridge::toCvCopy(msg, encoding)  // тот же API
cv_bridge::CvImageConstPtr                 → cv_bridge::CvImageConstPtr  // тот же тип
ptr->toImageMsg()                          → ptr->toImageMsg()  // возвращает SharedPtr, нужно *разыменовать
```

#### CMakeLists.txt паттерн
```cmake
cmake_minimum_required(VERSION 3.8)
project(PACKAGE_NAME)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_model REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(NODE_NAME src/file1.cpp src/file2.cpp ...)

ament_target_dependencies(NODE_NAME rclcpp std_msgs sensor_msgs cv_bridge camera_model)
target_link_libraries(NODE_NAME camera_model::camera_model ${OpenCV_LIBS})
target_include_directories(NODE_NAME PRIVATE ${EIGEN3_INCLUDE_DIR} ${camera_model_INCLUDE_DIRS})

install(TARGETS NODE_NAME DESTINATION lib/${PROJECT_NAME})
ament_package()
```

#### package.xml паттерн
```xml
<?xml version="1.0"?>
<package format="3">
  <name>PACKAGE_NAME</name>
  <version>0.0.0</version>
  <description>...</description>
  <maintainer email="email@example.com">Name</maintainer>
  <license>TODO</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <!-- ... другие зависимости ... -->
  <export><build_type>ament_cmake</build_type></export>
</package>
```

---

## 4. Детали по каждому ГОТОВОМУ пакету

### 4.1 camera_model ✅
- **Тип**: Библиотека (без ROS зависимостей в коде)
- **Изменения**: CMakeLists.txt (catkin → ament), package.xml, `SetParameterization` → `SetManifold` в CameraCalibration.cc
- **Новый файл**: `include/camodocal/gpl/ceres_local_parameterization_compat.h` — shim для Ceres 2.2
- **Изменён**: `include/camodocal/gpl/EigenQuaternionParameterization.h` — include shim вместо удалённого `ceres/local_parameterization.h`
- **Собирается**: Да

### 4.2 benchmark_publisher ✅
- **Тип**: Простой узел (1 subscriber, 2 publishers)
- **Изменения**: Полная переработка `benchmark_publisher_node.cpp`, CMakeLists.txt, package.xml
- **Особенности**: `ros::Time(double)` → ручная конверсия в `builtin_interfaces::msg::Time`
- **Собирается**: Да

### 4.3 feature_tracker ✅
- **Тип**: Узел (1 subscriber, 3 publishers)
- **Изменения**: `feature_tracker_node.cpp` (полная переработка), `parameters.cpp/h`, `feature_tracker.cpp` (ROS_DEBUG → RCLCPP_DEBUG)
- **Особенности**:
  - `pub_img` и `pub_match` — разные типы (`PointCloud` и `Image`), разделены на отдельные объявления
  - `pub->publish(*feature_points)` — разыменование SharedPtr при публикации
  - `cv_bridge/cv_bridge.hpp` (не `.h`)
  - `cv::COLOR_GRAY2RGB` вместо `CV_GRAY2RGB`
  - `g_clock` — глобальный `rclcpp::Clock::SharedPtr` для `RCLCPP_WARN_THROTTLE`
- **Собирается**: Да

### 4.4 vins_estimator ✅
- **Тип**: Самый сложный узел (5 subscribers, 13 publishers, TF, потоки)
- **Изменения**: 19 файлов модифицированы
- **Ключевые решения**:
  - `estimator.h`: `std_msgs::Header` → `std_msgs::msg::Header` (включая member `Headers[WINDOW_SIZE+1]`)
  - `estimator.cpp`: `ros::Time::now().toSec()` → `std::chrono::system_clock` (wall clock, не зависит от ROS)
  - `visualization.cpp/h`: Все 13 publishers → `rclcpp::Publisher<...>::SharedPtr`, TF → `tf2_ros::TransformBroadcaster`
  - `CameraPoseVisualization.cpp/h`: `ros::Publisher&` → `rclcpp::Publisher<...>::SharedPtr`
  - `feature_manager.h`: `#include <ros/console.h>` + `<ros/assert.h>` → `#include <cassert>` + `#include "rclcpp/rclcpp.hpp"`
  - `initial/initial_alignment.h`, `initial_ex_rotation.h`, `solve_5pts.h`: ROS includes заменены
  - `factor/imu_factor.h`, `marginalization_factor.h`: добавлен `#include "rclcpp/rclcpp.hpp"` для ROS_ASSERT
  - `pose_local_parameterization.h`: include compat shim
  - `estimator.cpp`: `ceres::LocalParameterization*` → `ceres::Manifold*`, `AddParameterBlock(..., local_param)` → `SetManifold(...)`
  - `initial_sfm.cpp`: `ceres::LocalParameterization*` → `ceres::Manifold*`
  - `utility.h`: добавлена `stampToSec()` helper функция
  - QoS для IMU: `rclcpp::QoS(rclcpp::KeepLast(2000)).best_effort()` (замена `tcpNoDelay`)
- **Собирается**: Да

---

## 5. Детали по НЕЗАВЕРШЁННЫМ пакетам

### 5.1 pose_graph ✅ (ГОТОВО)

> Обновлено 2026-07-04: перечисленный ниже исторический чек-лист выполнен. Пакет
> переведён на ROS2 Jazzy и успешно собран через `colcon`. Мигрированы node API,
> сообщения и publishers, TF2, `ament_index_cpp`, Ceres 2.2 `Manifold`; BRIEF-файлы
> теперь устанавливаются в `share/pose_graph/support_files`.

**Уже мигрированы (3 файла):**
- `src/parameters.h` — includes заменены, `ros::Publisher` → `rclcpp::Publisher<...>::SharedPtr`
- `src/pose_graph.h` — includes заменены, `nav_msgs::Path` → `nav_msgs::msg::Path`, `tf::TransformBroadcaster` → `tf2_ros::TransformBroadcaster`, `AngleLocalParameterization::Create()` → `ceres::AutoDiffManifold`, `registerPub(ros::NodeHandle&)` → `registerPub(rclcpp::Node::SharedPtr)`, publishers → `rclcpp::Publisher<...>::SharedPtr`
- `src/utility/CameraPoseVisualization.h` — includes заменены, типы обновлены, `publish_by` и `publish_image_by` сигнатуры обновлены

**НЕ мигрированы (нужно сделать):**

#### `src/utility/CameraPoseVisualization.cpp`
Заменить:
- `geometry_msgs::Point` → `geometry_msgs::msg::Point`
- `visualization_msgs::Marker` → `visualization_msgs::msg::Marker`
- `visualization_msgs::MarkerArray` → `visualization_msgs::msg::MarkerArray`
- `visualization_msgs::Marker::LINE_LIST` → `visualization_msgs::msg::Marker::LINE_LIST` (и т.д.)
- `ros::Duration()` → `builtin_interfaces::msg::Duration()`
- `ros::Publisher &pub` → `rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub` (в `publish_by`)
- `ros::Publisher &pub` → `rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub` (в `publish_image_by`)
- `const std_msgs::Header &header` → `const std_msgs::msg::Header &header`
- `pub.publish(...)` → `pub->publish(...)`

#### `src/pose_graph.cpp`
Заменить:
- `#include <tf/transform_broadcaster.h>` → удалить (уже в pose_graph.h)
- `registerPub(ros::NodeHandle &n)` → `registerPub(rclcpp::Node::SharedPtr n)`
- `n.advertise<T>(topic, q)` → `n->create_publisher<T>(topic, q)`
- `tf_broadcaster.reset(new tf::TransformBroadcaster())` → `tf_broadcaster.reset(new tf2_ros::TransformBroadcaster(n))`
- `ros::Time(cur_kf->time_stamp)` → helper: заполнить `builtin_interfaces::msg::Time` из double
- TF broadcasting: `tf::Transform`, `tf::Vector3`, `tf::Quaternion`, `tf::StampedTransform` → `geometry_msgs::msg::TransformStamped`
- `pub.publish(...)` → `pub->publish(...)`
- `ceres::LocalParameterization*` → `ceres::Manifold*`
- `problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization)` → `problem.AddParameterBlock(euler_array[i], 1); problem.SetManifold(euler_array[i], angle_local_parameterization)`

#### `src/pose_graph_node.cpp`
Заменить:
- Все ROS1 includes → ROS2 (см. паттерн выше)
- `ros::init` → `rclcpp::init`
- `ros::NodeHandle n("~")` → `auto node = std::make_shared<rclcpp::Node>("pose_graph")`
- `n.getParam(...)` → `node->declare_parameter(...)` + `node->get_parameter(...)`
- `ros::package::getPath("pose_graph")` → `ament_index_cpp::get_package_share_directory("pose_graph")`
- Все `ros::Subscriber` → `rclcpp::Subscription<...>::SharedPtr` через `node->create_subscription<>()`
- Все `ros::Publisher` → `rclcpp::Publisher<...>::SharedPtr` через `node->create_publisher<>()`
- `sensor_msgs::ImageConstPtr` → `sensor_msgs::msg::Image::ConstSharedPtr`
- `sensor_msgs::PointCloudConstPtr` → `sensor_msgs::msg::PointCloud::ConstSharedPtr`
- `nav_msgs::Odometry::ConstPtr` → `nav_msgs::msg::Odometry::ConstSharedPtr`
- `cv_bridge::toCvCopy` — тот же API
- `ROS_WARN(...)` → `RCLCPP_WARN(node->get_logger(), ...)`
- `ROS_BREAK()` → `std::abort()`
- `ros::Duration()` → `builtin_interfaces::msg::Duration()`
- `visualization_msgs::Marker::SPHERE_LIST` → `visualization_msgs::msg::Marker::SPHERE_LIST`
- `pub.publish(...)` → `pub->publish(...)`
- `ros::spin()` → `rclcpp::spin(node)`
- `std::thread process` и `std::thread command` — остаются без изменений

**Подписчики (7 штук):**
| Topic | Message Type | Callback |
|-------|-------------|----------|
| `/vins_estimator/imu_propagate` | `nav_msgs::msg::Odometry` | `imu_forward_callback` |
| `/vins_estimator/odometry` | `nav_msgs::msg::Odometry` | `vio_callback` |
| `IMAGE_TOPIC` (из конфига) | `sensor_msgs::msg::Image` | `image_callback` |
| `/vins_estimator/keyframe_pose` | `nav_msgs::msg::Odometry` | `pose_callback` |
| `/vins_estimator/extrinsic` | `nav_msgs::msg::Odometry` | `extrinsic_callback` |
| `/vins_estimator/keyframe_point` | `sensor_msgs::msg::PointCloud` | `point_callback` |
| `/vins_estimator/relo_relative_pose` | `nav_msgs::msg::Odometry` | `relo_relative_pose_callback` |

**Издатели (5 в node + 9 в PoseGraph):**
| Variable | Topic | Message Type |
|----------|-------|-------------|
| `pub_match_img` | `match_image` | `sensor_msgs::msg::Image` |
| `pub_camera_pose_visual` | `camera_pose_visual` | `visualization_msgs::msg::MarkerArray` |
| `pub_key_odometrys` | `key_odometrys` | `visualization_msgs::msg::Marker` |
| `pub_vio_path` | `no_loop_path` | `nav_msgs::msg::Path` |
| `pub_match_points` | `match_points` | `sensor_msgs::msg::PointCloud` |
| `pub_pg_path` | `pose_graph_path` | `nav_msgs::msg::Path` |
| `pub_base_path` | `base_path` | `nav_msgs::msg::Path` |
| `pub_pose_graph` | `pose_graph` | `visualization_msgs::msg::MarkerArray` |
| `pub_path[1-9]` | `path_1`...`path_9` | `nav_msgs::msg::Path` |

#### `src/keyframe.cpp`
Заменить:
- `sensor_msgs::ImagePtr` → `sensor_msgs::msg::Image::SharedPtr`
- `cv_bridge::CvImage(std_msgs::Header(), "bgr8", thumbimage).toImageMsg()` → `cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", thumbimage).toImageMsg()`
- `msg->header.stamp = ros::Time(time_stamp)` → ручная конверсия double → `builtin_interfaces::msg::Time`
- `sensor_msgs::PointCloud` → `sensor_msgs::msg::PointCloud`
- `geometry_msgs::Point32` → `geometry_msgs::msg::Point32`
- `sensor_msgs::ChannelFloat32` → `sensor_msgs::msg::ChannelFloat32`
- `pub_match_img.publish(msg)` → `pub_match_img->publish(*msg)`
- `pub_match_points.publish(msg_match_points)` → `pub_match_points->publish(msg_match_points)`

#### `src/keyframe.h`
- Не имеет прямых ROS includes, но включает `parameters.h` (уже мигрирован)
- Проверить, что нет косвенных ROS зависимостей

#### CMakeLists.txt
Заменить по паттерну. Дополнительно:
- `find_package(ament_index_cpp REQUIRED)` для `get_package_share_directory`
- Установить support_files: `install(DIRECTORY ../../support_files/ DESTINATION share/${PROJECT_NAME}/support_files FILES_MATCHING PATTERN "*.bin" PATTERN "*.yml")`
- ThirdParty source files включаются в `add_executable` (DBoW, DUtils, DVision, VocabularyBinary)

#### package.xml
Заменить по паттерну. Зависимости: `rclcpp`, `std_msgs`, `nav_msgs`, `sensor_msgs`, `visualization_msgs`, `geometry_msgs`, `cv_bridge`, `camera_model`, `tf2_ros`, `ament_index_cpp`

### 5.2 vins_bringup ✅

**Файлы:** `CMakeLists.txt`, `package.xml`, `launch/*.launch.py`, `launch/common.py`

**Что делает пакет:**
- Поднимает `feature_tracker`, `vins_estimator` и `pose_graph` в нужной конфигурации.
- Поддерживает сценарии `euroc`, `3dm`, `usb_cam`, `realsense`, `cla`, `black_box`, `termal_cam`.
- Отдельно запускает `rviz2` с готовым конфигом.

**Что уже сделано:**
- Создан `vins_bringup/CMakeLists.txt`.
- Создан `vins_bringup/package.xml`.
- Добавлены launch-файлы для основных сценариев.
- Добавлен bag-specific launch `innospector_cam.launch.py` с `image_transport republish` для входа `/image_raw/compressed`.
- Добавлен конфиг `config/fisheye_bag/fisheye_bag_config.yaml` под камеру `1280x1024` и topic `"/mavros/imu/data"`.
- Конфиги из `config/` устанавливаются в `share/vins_bringup/config`.

**Что осталось:**
- Пакет собран.
- Launch-файлы синтаксически проверены.

---

## 6. Создание vins_bringup (launch package) — ГОТОВО

Пакет создан и собран; далее остаётся только финальная проверка всего workspace при необходимости.

---

## 7. Порядок оставшихся работ

1. **Финальная сборка** — `colcon build --symlink-install --base-paths src` (все пакеты)
2. **Тестирование** — запуск с реальными данными

---

## 8. Известные проблемы и решения

| Проблема | Решение |
|----------|---------|
| `ceres::LocalParameterization` удалён в Ceres 2.2 | Использовать `ceres_local_parameterization_compat.h` shim или `ceres::Manifold` напрямую |
| `cv_bridge/cv_bridge.h` не найден | В ROS2 Jazzy: `cv_bridge/cv_bridge.hpp` (не `.h`) |
| `pub->publish(shared_ptr)` не компилируется | Разыменовать: `pub->publish(*msg)` |
| `ros::Time(double)` нет прямого аналога | Ручная конверсия: `sec = (int32_t)t; nanosec = (uint32_t)((t - sec) * 1e9)` |
| `ros::Duration()` для marker lifetime | `builtin_interfaces::msg::Duration()` |
| `CV_GRAY2RGB` удалён в OpenCV 4.x | `cv::COLOR_GRAY2RGB` |
| `AddParameterBlock(..., local_parameterization)` | `AddParameterBlock(...); SetManifold(..., local_parameterization)` |
| `camera_model` не линкуется | Использовать imported target: `camera_model::camera_model` (не просто `camera_model`) |
| Boost targets не найдены при линковке | Добавить `find_package(Boost REQUIRED COMPONENTS filesystem program_options system)` в downstream пакеты |
| colcon не видит пакеты | Использовать `--base-paths src` (пакеты в подпапках `src/VINS-Mono-inno/`) |
| `ROS_WARN_THROTTLE(rate, ...)` | `RCLCPP_WARN_SKIPFIRST_THROTTLE(logger, clock, ms, ...)` — нужен `rclcpp::Clock` |

---

## 9. Ссылки на эталонные файлы

Для миграции оставшихся пакетов используй уже мигрированные файлы как образцы:

| Что посмотреть | Эталонный файл |
|---------------|---------------|
| CMakeLists.txt (библиотека) | `camera_model/CMakeLists.txt` |
| CMakeLists.txt (узел) | `feature_tracker/CMakeLists.txt` или `vins_estimator/CMakeLists.txt` |
| package.xml | `feature_tracker/package.xml` |
| Node main() | `feature_tracker/src/feature_tracker_node.cpp` |
| parameters.cpp/h | `feature_tracker/src/parameters.cpp` или `vins_estimator/src/parameters.cpp` |
| visualization с TF | `vins_estimator/src/utility/visualization.cpp` |
| CameraPoseVisualization | `vins_estimator/src/utility/CameraPoseVisualization.cpp` |
| Ceres Manifold usage | `vins_estimator/src/estimator.cpp` (строки ~710-718) |
| stampToSec helper | `vins_estimator/src/utility/utility.h` |

---

## 10. Инновационные дополнения этого форка (сохранить при миграции)

Эти функции добавлены в `UAV_perfom` branch и должны быть сохранены:

1. **AGAST corner detector** + grid-based feature selection (`use_advanced_flow`)
2. **Bidirectional optical flow** (`use_bidirectional_flow`)
3. **Velocity-based adaptive feature tracking** (`enable_velocity_check`, `max_velocity_threshold`, `velocity_boost_features`)
4. **IMU accelerometer filtering** (outlier rejection + exponential smoothing)
5. **Zero Velocity Update (ZUPT)** (`enable_zupt`, `zupt_vel_threshold`, `zupt_acc_threshold`)
6. **Altitude-based scale correction** via MAVLink velocity
7. **Periodic td estimation** (`estimate_td_period`)
8. **Thermal camera support** (`config/termal_cam_config.yaml`)
9. **TF broadcasting** of loop-closed trajectory (`world → camera_pose_graph`)
10. **Failure detection thresholds** tuned for high-speed UAV

Все эти функции реализованы в коде и не требуют отдельных изменений — они используют те же ROS API, что и остальной код, и мигрируются автоматически.

---

## 11. Команды для проверки

```bash
# Сборка одного пакета
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash 2>/dev/null
colcon build --symlink-install --packages-select <PACKAGE_NAME> --base-paths src

# Сборка всех пакетов VINS-Mono
colcon build --symlink-install --packages-select camera_model benchmark_publisher feature_tracker vins_estimator pose_graph vins_bringup --base-paths src

# Проверка ошибок компиляции (без сборки)
colcon build --packages-select <PACKAGE_NAME> --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | grep "error:"

# Запуск узла
ros2 run feature_tracker feature_tracker --ros-args -p config_file:=/path/to/config.yaml -p vins_folder:=/path/to/
```
