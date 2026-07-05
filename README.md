# VINS-Mono-Inno for ROS 2

VINS-Mono-Inno is a ROS 2 port and UAV-oriented fork of
[VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) by the HKUST
Aerial Robotics Group. It provides monocular visual-inertial odometry, feature
tracking, optional loop closure, and pose-graph optimization.

This repository uses ROS 2 packages, `ament_cmake`, Python launch files, and
`colcon`. The old ROS 1 `catkin_make`, `roslaunch`, and `rosbag play` commands do
not apply to this version.

The current migration is developed and tested on Ubuntu 24.04 with ROS 2 Jazzy.

## Packages

- `camera_model` — pinhole and omnidirectional camera models.
- `feature_tracker` — image decoding, feature detection, optical flow, masks,
  and debug image publication.
- `vins_estimator` — sliding-window visual-inertial estimator.
- `pose_graph` — loop detection and pose-graph optimization.
- `vins_bringup` — ROS 2 launch files and installed configuration files.
- `benchmark_publisher` — optional benchmark visualization support.

The repository also contains UAV-specific additions such as adaptive feature
tracking, optional bidirectional optical flow, IMU filtering, ZUPT, temporal
offset estimation, and MAVROS integration.

## Requirements

Install ROS 2 Jazzy and the required ROS packages:

```bash
sudo apt update
sudo apt install \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-compressed-image-transport \
  ros-jazzy-rviz2 \
  ros-jazzy-tf2-ros
```

The project also requires OpenCV, Eigen, Boost, and Ceres Solver. On Ubuntu,
the development packages can normally be installed with:

```bash
sudo apt install libopencv-dev libeigen3-dev libboost-all-dev libceres-dev
```

## Build

Place the repository inside a ROS 2 workspace, then build it with `colcon`:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Innopolis-UAV-Team/VINS-Mono-inno.git

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If the repository is already located in `~/ros2_ws/src`, only the last four
commands are needed. Add the ROS and workspace setup files to `.bashrc` if you
want new terminals to source them automatically.

## Run the Innospector fisheye bag configuration

The launch file `innospector_cam.launch.py` starts:

- `feature_tracker`;
- `vins_estimator`;
- `pose_graph`.

Its parameters are stored in
`config/fisheye_bag/fisheye_bag_config.yaml`. The supplied configuration expects:

- compressed camera images on `/image_raw/compressed`;
- IMU messages on `/mavros/imu/data`.

The YAML uses the image-transport base topic `/image_raw` together with
`image_input_format: "compressed"`. Do not change the base topic to
`/image_raw/compressed` when compressed transport is enabled.

Terminal 1 — start VINS:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch vins_bringup innospector_cam.launch.py
```

Terminal 2 — play the ROS 2 bag:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 bag play /path/to/bag
```

Only the required bag topics may be played:

```bash
ros2 bag play /path/to/bag \
  --topics /image_raw/compressed /mavros/imu/data
```

Terminal 3 — start the supplied RViz configuration:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch vins_bringup vins_rviz.launch.py
```

RViz is recommended for viewing the tracked-feature image. Some versions of
`rqt_image_view` perform poorly with large, high-rate images.

## Other launch configurations

All supported ROS 2 launch files are owned by `vins_bringup`. The
`vins_estimator` package contains the estimator executable and algorithm; it is
not a duplicate bringup package and no longer contains legacy ROS 1 XML launch
files. Always launch a complete configuration with:

```bash
ros2 launch vins_bringup <name>.launch.py
```

For the thermal camera configuration, start the estimator pipeline with:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch vins_bringup termal_cam.launch.py
```

Then start RViz in another terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch vins_bringup vins_rviz.launch.py
```

The thermal launch reads `config/termal_cam_config.yaml` and starts
`feature_tracker`, `vins_estimator`, and `pose_graph` together.

```bash
ros2 launch vins_bringup euroc.launch.py
ros2 launch vins_bringup euroc_no_extrinsic_param.launch.py
ros2 launch vins_bringup realsense_color.launch.py
ros2 launch vins_bringup realsense_fisheye.launch.py
ros2 launch vins_bringup usb_cam.launch.py
ros2 launch vins_bringup termal_cam.launch.py
ros2 launch vins_bringup 3dm.launch.py
```

Review the corresponding file under `config/` before using a launch file. Topic
names, camera calibration, camera-to-IMU extrinsics, IMU noise parameters, and
time synchronization must match the actual sensor system.

## Verify the pipeline

Check that the source topics are active:

```bash
ros2 topic hz /image_raw/compressed
ros2 topic hz /mavros/imu/data
```

Check each processing stage:

```bash
ros2 topic hz /feature_tracker/feature
ros2 topic hz /feature_tracker/feature_img
ros2 topic hz /vins_estimator/odometry
ros2 topic hz /vins_estimator/path
```

Inspect endpoint types and QoS when a publisher and subscriber do not connect:

```bash
ros2 topic info -v /image_raw/compressed
ros2 topic info -v /feature_tracker/feature
```

Expected types include:

- `/image_raw/compressed`: `sensor_msgs/msg/CompressedImage`;
- `/feature_tracker/feature`: `sensor_msgs/msg/PointCloud`;
- `/feature_tracker/feature_img`: `sensor_msgs/msg/Image`;
- `/vins_estimator/odometry`: `nav_msgs/msg/Odometry`.

## Camera masks and image formats

`feature_tracker` supports ROS 2 raw and compressed image transport. Select the
transport in the camera configuration:

```yaml
image_topic: "/image_raw"
image_input_format: "compressed"  # use "raw" for sensor_msgs/msg/Image
```

A custom feature mask can restrict tracking to valid image pixels:

```yaml
fisheye: 1
fisheye_mask: "config/mask.jpg"
```

The mask must have the same width and height as the configured camera. White
pixels enable feature detection and black pixels exclude an area. This mask
does not select or replace the camera projection model.

## Параметры feature tracker

Эти параметры задаются в конфигурационном YAML камеры. Они управляют
детектированием углов и их сопровождением оптическим потоком. Это не параметры
BRIEF-дескриптора: BRIEF используется отдельно модулем `pose_graph` для loop
closure.

| Параметр | Назначение и влияние |
|---|---|
| `max_cnt` | Целевое максимальное число сопровождаемых точек в кадре. Уже существующие треки сохраняются с приоритетом и равномерно распределяются по изображению, свободные места заполняются новыми точками. Большее значение обычно повышает устойчивость, но увеличивает нагрузку на CPU, RANSAC и оптимизатор. Это верхняя граница, а не гарантия фактического количества точек. |
| `min_dist` | Минимальное пространственное расстояние между выбранными точками, в пикселях. Большое значение даёт более редкое и равномерное покрытие; слишком большое не позволит набрать `max_cnt`. Малое значение позволяет набрать больше точек, но повышает вероятность кластеров на одной текстурной области. |
| `freq` | Целевая частота публикации и пополнения набора признаков, в Гц. Входные изображения продолжают использоваться оптическим потоком, но RANSAC, детектирование новых точек и публикация выполняются только для выбранных кадров. `0` в конфиге заменяется значением `100`. |
| `F_threshold` | Порог ошибки RANSAC при проверке фундаментальной матрицей, в пикселях виртуального недисторсированного изображения. Меньшее значение строже удаляет выбросы, но при шуме и размытии может удалить хорошие треки. Большее сохраняет больше точек, включая потенциальные выбросы. |
| `gftt_quality_level` | Относительный порог качества Shi–Tomasi `goodFeaturesToTrack`, диапазон `(0, 1]`. Используется для новых точек только при `use_advanced_flow: 0`. Меньше — больше слабых углов; больше — меньше, но обычно качественнее. |
| `fast_threshold` | Порог AGAST для новых точек. Используется только при `use_advanced_flow: 1`. Меньше — больше кандидатов, включая слабые и шумные; больше — меньше кандидатов с более выраженным контрастом. |
| `enable_feature_auto_tuning` | Включает (`1`) периодическую корректировку порога активного детектора. Для AGAST меняется `fast_threshold`, для Shi–Tomasi — `gftt_quality_level`. `max_cnt`, `min_dist`, `F_threshold` и параметры оптического потока автоматически не меняются. |
| `feature_auto_tune_interval` | Интервал автоподбора в секундах; минимально допустимое значение — `0.5`. Статистика заполнения и сохранения треков накапливается между обновлениями. Действует только при включённом автоподборе. |
| `feature_auto_tune_log` | При значении `1` выводит после каждого интервала строку `[AUTO_TUNE]` в ROS-консоль и лог. В ней указаны заполнение (`fill`), доля сохранившихся треков (`retention`), число кандидатов и текущие пороги. Не включает сам автоподбор. |
| `fast_threshold_min`, `fast_threshold_max` | Нижняя и верхняя границы, между которыми автоподбор может менять порог AGAST. При выключенном автоподборе границы не изменяют `fast_threshold`, кроме начального ограничения значения допустимым диапазоном при чтении конфигурации. |
| `gftt_quality_min`, `gftt_quality_max` | Нижняя и верхняя границы автоматического изменения `gftt_quality_level` для Shi–Tomasi. При `use_advanced_flow: 1` этот диапазон не участвует в выборе новых точек. |
| `use_bidirectional_flow` | При `1` после прямого LK-потока выполняет обратный поток и удаляет точки с плохой согласованностью. Повышает надёжность, но почти удваивает стоимость оптического потока. Реализован только в ветке `use_advanced_flow: 1`. |
| `use_advanced_flow` | Выбирает режим трекера. `1`: AGAST, пространственный отбор, уточнение части точек до субпиксельной точности и расширенные настройки LK. `0`: Shi–Tomasi и базовый LK. Этот параметр одновременно определяет, какой порог регулирует автоподбор. |
| `show_track` | При `1` публикует изображение с визуализацией треков в `/feature_tracker/feature_img`. Не отключает публикацию самих признаков в `/feature_tracker/feature`. Визуализация добавляет расход CPU и пропускной способности ROS. |
| `equalize` | При `1` применяет CLAHE к каждому серому кадру перед сопровождением и детектированием. Помогает при слабом или неравномерном освещении, но может усилить шум и добавляет вычислительную нагрузку. |
| `fisheye` | При `1` включает применение `fisheye_mask` к выбору точек. Параметр не выбирает модель камеры и не включает коррекцию fisheye-дисторсии; модель задаётся отдельно через `model_type`. |
| `fisheye_mask` | Путь к одноканальной маске размера изображения. Белые пиксели разрешают признаки, чёрные запрещают. Относительный путь разрешается относительно каталога пакета `vins_bringup`; абсолютный используется напрямую. |

Пример ручного режима AGAST:

```yaml
max_cnt: 250
min_dist: 10
fast_threshold: 20
use_advanced_flow: 1
enable_feature_auto_tuning: 0
```

В этом режиме `gftt_quality_level`, `gftt_quality_min` и `gftt_quality_max` не
влияют на детектирование новых точек. `feature_auto_tune_interval` и
`feature_auto_tune_log` начнут действовать только после установки
`enable_feature_auto_tuning: 1`.

## Calibration notes

Reliable VIO requires all of the following:

1. Camera intrinsics and distortion parameters for the exact image resolution.
2. An accurate camera-to-IMU rotation and translation.
3. Correct IMU noise and bias random-walk parameters.
4. Monotonic timestamps and a known or estimated camera/IMU time offset.
5. Sufficient motion and parallax during estimator initialization.

Use `estimate_extrinsic: 0` only when the supplied extrinsic transform is known
to be accurate. Use `estimate_td: 1` for unsynchronized sensors; otherwise fix
the calibrated offset with `estimate_td: 0` and `td: <seconds>`.

For a physical fisheye or very wide-angle camera, calibrate an appropriate
camodocal model such as `KANNALA_BRANDT` or `MEI`. Setting `fisheye: 1` only
enables the feature mask; it does not change the projection model.

## Loop closure and pose graph parameters

Loop closure is configured in the camera YAML file:

```yaml
loop_closure: 1
load_previous_pose_graph: 0
fast_relocalization: 0
pose_graph_save_path: "/tmp/vins_thermal_output/pose_graph/"
```

`loop_closure` enables image-based place recognition and pose graph
optimization:

- `0` runs local visual-inertial odometry only. Accumulated drift is not
  corrected by previously visited places.
- `1` loads the BRIEF vocabulary, searches for previously observed keyframes,
  verifies candidates geometrically, and optimizes the pose graph after an
  accepted loop.

`load_previous_pose_graph` controls map reuse at startup:

- `0` starts a new pose graph.
- `1` loads a previously saved graph from `pose_graph_save_path` and attempts
  to align the current sequence with it. The saved graph must be compatible
  with the current camera calibration and configuration.

`fast_relocalization` sends geometrically verified loop matches back to the
VINS estimator:

- `0` keeps loop correction primarily inside the pose graph.
- `1` publishes matched points to the estimator for faster correction of its
  local state. Enable this only after ordinary loop closure has been verified
  as stable.

`pose_graph_save_path` is the directory used to save and load keyframes,
BRIEF descriptors, graph constraints, and trajectory data. Paths under `/tmp`
are normally removed after a reboot. Use a persistent location for map reuse,
for example:

```yaml
pose_graph_save_path: "/home/jetson/vins_maps/thermal_pose_graph/"
```

For a first run, the recommended settings are:

```yaml
loop_closure: 1
load_previous_pose_graph: 0
fast_relocalization: 0
```

The console reports loop processing with `[LOOP]` messages. A real closure is
confirmed by `[LOOP] accepted`; a DBoW image candidate may still be rejected by
geometric verification.

## Tests

Build and run the package tests with:

```bash
cd ~/ros2_ws
source install/setup.bash
colcon test --packages-select feature_tracker
colcon test-result --verbose
```

Additional migration and runtime checks are documented in
[ROS2_FISHEYE_CHECKLIST.md](ROS2_FISHEYE_CHECKLIST.md),
[ROS2_MIGRATION_PLAN.md](ROS2_MIGRATION_PLAN.md), and
[ROS2_MIGRATION_HANDOFF.md](ROS2_MIGRATION_HANDOFF.md).

## Original project and citation

VINS-Mono is an optimization-based monocular visual-inertial SLAM framework by
Tong Qin, Peiliang Li, Zhenfei Yang, and Shaojie Shen. If this software is used
for academic research, cite the original publications:

- T. Qin, P. Li, Z. Yang, and S. Shen, “VINS-Mono: A Robust and Versatile
  Monocular Visual-Inertial State Estimator,” IEEE Transactions on Robotics.
- T. Qin and S. Shen, “Online Temporal Calibration for Monocular
  Visual-Inertial Systems,” IROS 2018.

This fork uses Ceres Solver, DBoW2, OpenCV, Eigen, Boost, and camodocal.

## License

The source code is released under the GPLv3 license. This ROS 2 fork is
maintained by the Innopolis UAV Team; the original VINS-Mono authors retain
credit for the underlying system.
