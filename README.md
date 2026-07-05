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
