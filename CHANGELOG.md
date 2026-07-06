# CHANGELOG

## [Unreleased] - 2026-01-04

### Added - Unified Feature Detection & Optical Flow Control
- **Single configuration flag (`use_advanced_flow`)** to switch between original VINS-Fusion and enhanced UAV implementation
  - `use_advanced_flow: 0` — Original VINS-Fusion style:
    - Shi-Tomasi corner detector (`goodFeaturesToTrack`)
    - Basic Lucas-Kanade optical flow (default parameters)
    - No forward-backward consistency check
  - `use_advanced_flow: 1` — Enhanced UAV implementation (default):
    - AGAST corner detector with grid-based spatial distribution
    - Advanced LK with strict termination criteria (40 iterations, 0.001 eps)
    - Optional forward-backward consistency check (if `use_bidirectional_flow: 1`)
    - Subpixel refinement for top 50 features
- **Rationale**: Single toggle for complete feature tracking pipeline — useful for benchmarking and fallback to proven VINS-Fusion behavior
- **Configuration**:
  ```yaml
  use_advanced_flow: 1                # 1=enhanced (AGAST), 0=original (Shi-Tomasi)
  use_bidirectional_flow: 0           # Only active when use_advanced_flow=1
  fast_threshold: 20                  # AGAST response threshold (use_advanced_flow=1)
  ```

## [Previous] - 2026-01-02

### Added - Velocity-Based Adaptive Feature Tracking
- **Dynamic feature count adjustment** based on estimated velocity for high-speed navigation stability
  - Estimates velocity from optical flow magnitude between frames
  - Automatically boosts feature count when velocity exceeds threshold
  - Default: 150 features → 200 features (thermal) or 150 features (USB) at high speed
  - Configurable velocity threshold (default: 2.5-3.0 m/s)
  - Prevents odometry failure during fast UAV motion
- **Configuration parameters**:
  ```yaml
  enable_velocity_check: 1            # Enable/disable adaptive tracking
  max_velocity_threshold: 2.5         # Velocity threshold (m/s)
  velocity_boost_features: 50         # Extra features to add at high speed
  min_parallax_threshold: 3.0         # Minimum parallax (pixels)
  ```

### Modified - System Robustness & Error Handling
- **Negative timestamp handling**: System now gracefully handles timestamp synchronization issues after reboot
  - Detects negative `dt_1` values and skips frame with warning instead of assertion failure
  - Prevents crashes during system recovery after failure detection
- **Increased failure detection thresholds** for high-speed navigation:
  - Horizontal position jump: 5m → 10m
  - Vertical position jump: 1m → 3m
  - Reduces false positive reboots during aggressive maneuvers

### Modified - Performance Optimization
- **Bidirectional optical flow disabled by default** in both camera configs for ~40-50% speedup
  - Trade-off: slightly reduced tracking accuracy for real-time performance
  - Can be re-enabled via `use_bidirectional_flow: 1` if CPU allows
- **Reduced solver complexity**:
  - `max_solver_time`: 0.035s → 0.03s
  - `max_num_iterations`: 20-25 → 15 iterations
  - `keyframe_parallax`: 5-8 → 10 pixels (fewer keyframes)
- **Optimized feature tracking**:
  - Thermal camera: 200 → 150 features base count
  - USB camera: maintained at 150 features
  - `min_dist`: 10-12 → 15 pixels (thermal), 10 pixels (USB)
  - `fast_threshold`: 15-20 → 20 (fewer but stronger corners)

### Modified - Camera-Specific Tuning
- **Thermal camera (384×288) optimizations**:
  - `F_threshold`: 2.0 (increased noise tolerance)
  - `keyframe_parallax`: 10.0 pixels
  - `min_parallax_threshold`: 3.0 pixels
  - Stricter ZUPT: `vel_threshold=0.15`, `acc_threshold=0.25`
- **USB camera (640×480) optimizations**:
  - `F_threshold`: 1.0 (higher resolution = tighter threshold)
  - `min_parallax_threshold`: 5.0 pixels
  - `max_velocity_threshold`: 3.0 m/s

### Added - Velocity Logging
- **Real-time velocity output** in estimator logs
  - Horizontal velocity: `sqrt(vx² + vy²)` in m/s
  - Vertical velocity: `vz` in m/s
  - Format: `Pose: x=... y=... z=... | Vel: horiz=... vert=... m/s`
  - Helps diagnose IMU drift and odometry issues

### Technical Details
- Velocity estimation formula: `velocity = (avg_optical_flow / (FOCAL_LENGTH * dt)) * depth_scale`
- Depth scale factor: 3.0 (assumes ~3m average scene depth)
- Flow outlier rejection: ignores features with >100px displacement
- Adaptive feature count updated at start of each `readImage()` call
- Feature boost only applied when sufficient tracked points available (>10 features)

## [Previous] - 2026-01-01/02

### Added - IMU Data Filtering & Scale Correction

### Added - Pose Graph TF Publishing (Loop-Closed Frame)
- **TF transform `world -> camera_pose_graph`** now published from `pose_graph` after loop-closure correction
  - Uses loop-closed pose (`r_drift`, `t_drift`) exactly matching `/pose_graph/pose_graph_path`
  - Frame id: `world`; Child frame: `camera_pose_graph`
  - Broadcast created post-`ros::init` to avoid initialization crashes
- **Dependencies updated**: added `tf` to `pose_graph` package (CMake/package.xml)

### Modified - Time Offset Estimation (td)
- **Periodic td optimization** to reduce runtime cost: td parameter is optimized in short bursts every `estimate_td_period` seconds (default 7s)
- When idle between bursts, td stays constant and projection factors fall back to fast version without td term
- Optional config key `estimate_td_period` added (seconds)
- **Logging improvements**:
  - `td` value printed only when optimization occurs (not every frame)
  - Optimized trajectory position (`x, y, z`) printed every frame for monitoring
  - Format: `Optimized pose: x=... y=... z=...` and `td updated: ...` (when changed)

#### Accelerometer Outlier Rejection
- **Spike detection and filtering** for noisy 9-DOF IMU data
  - Detects sudden acceleration changes exceeding configurable threshold (default: 50 m/s²)
  - Rejects outlier measurements and uses previous filtered value
  - Prevents incorrect velocity/position estimates from bad IMU readings
  - Applied in both `predict()` and main processing loop

#### Exponential Smoothing Filter
- **Low-pass filtering** for accelerometer noise reduction
  - Exponential moving average (EMA) with configurable alpha (default: 0.8)
  - Reduces high-frequency noise while maintaining responsiveness
  - Works in conjunction with outlier rejection
  - Formula: `filtered = alpha * prev_filtered + (1-alpha) * current`

#### Altitude-Based Scale Correction
- **MAVLink velocity integration** for scale drift prevention
  - Subscribes to `/mavros/local_position/velocity_local` (geometry_msgs/TwistStamped)
  - Uses **only Z-component** of normalized global velocity vector
  - **Z-axis always points up** regardless of IMU orientation (global frame)
  - X and Y velocities ignored (unreliable in local position estimate)
  - Integrates vertical velocity to estimate altitude
  - Corrects monocular VIO scale drift using altitude comparison
  - Gentle correction with configurable strength (default: 10% per update)
  - Safety bounds: only corrects scale ratio between 0.8-1.2
  - Automatically scales positions, velocities, and feature depths
  - Only active after successful VIO initialization

#### Configuration Parameters
```yaml
# IMU Filtering
enable_imu_acc_filter: 1           # Enable/disable accelerometer filtering
imu_acc_max_change: 50.0           # Outlier detection threshold (m/s²)
imu_acc_filter_alpha: 0.8          # Smoothing factor (0.0-1.0)

# Altitude Scale Correction
enable_altitude_scale_correction: 1 # Enable/disable scale correction
altitude_correction_factor: 0.1     # Correction strength (0.0-1.0)
```

### Added - Feature Detection & Tracking

#### AGAST Corner Detector
- **Replaced Shi-Tomasi with AGAST** (Adaptive Generic Accelerated Segment Test)
  - Modern, faster corner detection algorithm
  - Configurable threshold via `fast_threshold` parameter (default: 20)
  - Adaptive feature detection for varying lighting conditions

#### Grid-Based Feature Selection
- **Uniform spatial distribution** of features across image
  - Divides image into grid cells of size `MIN_DIST`
  - Selects strongest feature per grid cell
  - Memory-efficient implementation using `std::unordered_map`
  - Prevents feature clustering in high-texture areas

#### Bidirectional Optical Flow
- **Forward-Backward consistency check** for robust tracking
  - Forward flow: current → next frame
  - Backward flow: next → current frame  
  - Outlier rejection when error > 0.7 pixels
  - Configurable via `use_bidirectional_flow` parameter (0/1)
  - Can be disabled for performance on resource-constrained systems

#### Sub-Pixel Refinement Optimization
- **Smart refinement strategy** to reduce computational cost
  - Applies sub-pixel refinement only to top-50 strongest features
  - Reduces processing time while maintaining accuracy
  - Uses OpenCV `cornerSubPix` with TermCriteria (40 iterations, 0.001 epsilon)

#### CLAHE Optimization
- **Contrast Limited Adaptive Histogram Equalization** performance improvement
  - CLAHE object created once in constructor, reused per frame
  - Eliminates redundant object allocation overhead
  - Critical for thermal imaging with low contrast

### Added - Visual-Inertial Odometry

#### Zero Velocity Update (ZUPT)
- **Drift prevention during stationary periods**
  - Detects when camera/IMU is stationary
  - Resets velocity to zero to prevent IMU integration drift
  - Configurable parameters:
    - `enable_zupt`: 0/1 to disable/enable
    - `zupt_vel_threshold`: velocity threshold (m/s) for detection
    - `zupt_acc_threshold`: acceleration deviation threshold (m/s²)
  - Applied in `processIMU()` when velocity < threshold AND acceleration ≈ gravity

### Added - Configuration & Launch Files

#### Thermal Camera Configuration
- **Complete thermal camera support** (`config/termal_cam_config.yaml`)
  - Resolution: 384×288 pixels
  - Camera model: PINHOLE
  - Intrinsics: fx=382.17, fy=381.92, cx=177.60, cy=137.18
  - Distortion coefficients: k1=-0.4137, k2=0.1618, p1=-0.0004, p2=-0.0022
  - IMU-Camera extrinsics (from Kalibr calibration)
  - Rotation matrix: 90° CCW around Z-axis with axis corrections
  - Feature tracker tuning for thermal imagery:
    - max_cnt: 180 features
    - min_dist: 30 pixels (reduced clustering)
    - fast_threshold: 40 (stronger corners only)
    - F_threshold: 2.0 pixels (RANSAC outlier rejection)
    - equalize: 1 (CLAHE enabled)
    - use_bidirectional_flow: 1
  - IMU parameters:
    - acc_n: 0.1 (measurement noise)
    - gyr_n: 0.05 (measurement noise)
    - acc_w: 0.002 (bias random walk)
    - gyr_w: 4.0e-5 (bias random walk)
  - ZUPT configuration:
    - enable_zupt: 1
    - zupt_vel_threshold: 0.2 m/s
    - zupt_acc_threshold: 0.2 m/s²

#### Launch File for Thermal Camera
- **Created** `vins_estimator/launch/termal_cam.launch`
  - Configured for `/thermal_cam/image_raw` topic
  - Uses thermal camera config file
  - Ready for deployment

### Modified - Feature Tracking Visualization

#### Depth-Based Feature Size Rendering
- **Visual debugging enhancement** in `feature_tracker_node.cpp`
  - Circle radius (1-4 pixels) based on optical flow velocity magnitude
  - Velocity magnitude as proxy for depth (closer features move faster)
  - Mapping: velocity / 10.0 → radius increment
  - Helps diagnose:
    - Scale/depth estimation issues
    - Feature motion patterns
    - Parallax distribution
  - Color still indicates track length (blue=new, red=long-tracked)

### Modified - Configuration Files

#### USB Camera Configuration
- **Updated** `config/usb_cam_config.yaml`
  - Added `use_bidirectional_flow` parameter
  - Added ZUPT parameters (vel_threshold: 0.05, acc_threshold: 0.1)
  - Changed `estimate_td` from 0 to 1 for time offset estimation

#### RVIZ Configuration
- **Updated** `config/vins_rviz_config.rviz`
  - Enabled `history_point` cloud visualization
  - Adjusted point cloud size and rendering
  - Modified camera view distance and angle
  - Updated window geometry

### Performance Improvements

#### Memory Efficiency
- **Grid allocation**: Replaced full `vector` with `unordered_map` for sparse grid storage
- **CLAHE reuse**: Single object creation instead of per-frame allocation
- **Sub-pixel refinement**: Limited to top-50 features instead of all detected

#### Processing Speed
- **Optional bidirectional flow**: Can be disabled for faster tracking
- **Grid-based selection**: Faster than sorting all features by response
- **AGAST detector**: Faster than Shi-Tomasi for equivalent quality

### Debugging & Diagnostics

#### Feature Visualization
- Circle size indicates estimated distance/depth
- Color indicates tracking age
- Helps identify:
  - Scale drift
  - Initialization problems  
  - Feature quality distribution

#### Console Output
- ZUPT status messages when velocity reset occurs
- CLAHE processing time tracking
- Feature detection timing breakdown

### Technical Details

#### File Changes
- `vins_estimator/src/estimator_node.cpp`: IMU filtering, MAVLink odometry subscription
- `vins_estimator/src/estimator.h/cpp`: Scale correction method `correctScaleWithAltitude()`
- `vins_estimator/src/feature_manager.h/cpp`: Depth scaling method `scaleDepth()`
- `feature_tracker/src/feature_tracker.cpp`: Core detection and tracking logic
- `feature_tracker/src/feature_tracker.h`: Added CLAHE member variable
- `feature_tracker/src/feature_tracker_node.cpp`: Visualization modifications
- `feature_tracker/src/parameters.h/cpp`: Added FAST_THRESHOLD, USE_BIDIRECTIONAL_FLOW
- `vins_estimator/src/parameters.h/cpp`: Added ZUPT parameters
- `config/termal_cam_config.yaml`: Thermal camera complete configuration
- `config/usb_cam_config.yaml`: Parameter updates
- `vins_estimator/launch/termal_cam.launch`: New launch file

#### Commit History
- `37418d2`: Bidirectional optical flow implementation
- `6710bdb`: FAST corner detector (replaced by AGAST later)
- `a51f8bd`: AGAST feature detection
- `a7fdcab`: AGAST + Grid selection + Lucas-Kanade optimization
- `9024f49`: Thermal camera configuration

### Dependencies
- OpenCV 4.x (required for AGAST, CLAHE APIs)
- ROS (Kinetic/Melodic/Noetic)
- Eigen 3
- Ceres Solver

### Configuration Parameters Reference

#### Feature Tracker Parameters
```yaml
max_cnt: 150-180          # Maximum number of tracked features
min_dist: 10-30           # Minimum distance between features (pixels)
fast_threshold: 20-40     # AGAST corner detection threshold
use_bidirectional_flow: 0/1  # Enable forward-backward tracking
F_threshold: 1.0-2.0      # RANSAC threshold for fundamental matrix
equalize: 0/1             # Enable CLAHE histogram equalization
```

#### ZUPT Parameters
```yaml
enable_zupt: 0/1                 # Enable/disable zero velocity update
zupt_vel_threshold: 0.05-0.2     # Velocity threshold (m/s)
zupt_acc_threshold: 0.1-0.2      # Acceleration deviation threshold (m/s²)
```

### Known Issues & Future Work

#### Addressing 9-DOF IMU Noise
- **Problem**: 9-DOF IMU occasionally produces incorrect velocity readings
- **Solution Implemented**: 
  - Accelerometer spike detection and rejection (threshold: 50 m/s²)
  - Exponential smoothing filter (alpha: 0.8) 
  - Altitude-based scale correction using barometer-fused odometry
- **Result**: Robust to intermittent IMU errors while maintaining accurate orientation from gyroscope

#### Pending Optimizations
- Multi-threading for AGAST detection (parallel processing)
- Memory management improvements (unique_ptr migration in estimator.cpp)

#### System Stability
- Camera "flying away" issue under investigation
- Depth-based visualization added for diagnosis
- Potential causes:
  - Initialization quality (requires sufficient motion)
  - IMU calibration accuracy
  - Scale observability in monocular VIO

### IMU Requirements

**Supported IMU Types:**
- **6-DOF IMU required** (3-axis accelerometer + 3-axis gyroscope)
- Magnetometer NOT used (9-DOF sensors work but mag data ignored)
- Standard ROS `sensor_msgs/Imu` message format
- Required calibration parameters:
  - Measurement noise: `acc_n`, `gyr_n`
  - Bias random walk: `acc_w`, `gyr_w`

### References & Credits

- Original VINS-Mono: HKUST Aerial Robotics Group
- AGAST detector: Elmar Mair et al.
- CLAHE: Karel Zuiderveld
- Kalibr calibration toolkit: ASL ETH Zurich
- Branch: `UAV_perfom`
- Author: Devitt Dmitry

---

**Note**: This changelog tracks improvements made to the UAV performance optimization branch. For production deployment, thorough testing on target hardware is recommended.
