#pragma once

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 10;
const int NUM_OF_CAM = 1;
const int NUM_OF_F = 1000;
//#define UNIT_SPHERE_ERROR

extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string VINS_RESULT_PATH;
extern std::string IMU_TOPIC;
extern double TD;
extern double TR;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;
extern double ROW, COL;

// Hover-aware extensions.
//
// Design principle: the YAML holds only *calibrated* or *flight-intent*
// values — camera/IMU intrinsics, extrinsics, td, and on/off switches for
// fork-specific behaviors. Everything else (thresholds, scale factors,
// physical envelopes) is either a compile-time constant or derived at
// runtime from the calibrated values. This removes the "dozens of magic
// YAML numbers" surface and keeps per-flight config minimal.
//
// YAML-driven (user switches):
//   enable_zupt, enable_rotation_zupt, static_init_bias_priming,
//   soften_failure_on_hover
// Derived / constexpr: everything else.

extern int    ENABLE_ZUPT;
extern int    ENABLE_ROTATION_ZUPT;
extern int    STATIC_INIT_BIAS_PRIMING;
extern int    SOFTEN_FAILURE_ON_HOVER;

// Stationarity thresholds — auto-derived from calibrated IMU noise on
// startup so a fresh calibration flows through without touching any
// thresholds manually. Users who need to override a specific platform
// can set `static_acc_thr` / `static_gyr_thr` / `static_flow_thr` in
// YAML; otherwise defaults are computed as 15 × {ACC_N, GYR_N}.
extern double STATIC_ACC_THR;
extern double STATIC_GYR_THR;
extern double STATIC_FLOW_THR;

// Fork-wide universal constants — multirotor-agnostic, never needed per-
// calibration. Defined as constexpr so there's one source of truth in the
// repo. Want a different value? Edit this header and rebuild.
namespace hover
{
// Rolling window & hysteresis.
constexpr double STATIC_WINDOW_SEC      = 0.4;   // s, rolling window length
constexpr int    STATIC_CONFIRM_CYCLES  = 3;     // cycles (~15 ms @200Hz)

// Rotation-only classifier. Universal — depends on gyro physics + camera
// geometry normalized through FOCAL_LENGTH, not on a particular platform.
constexpr double ROTATION_ZUPT_GYR_MIN       = 0.3;   // rad/s   (~17 °/s)
constexpr double ROTATION_ZUPT_FLOW_RATIO    = 1.3;
constexpr double ROTATION_ZUPT_FLOW_BASELINE = 30.0;  // px/s

// Adaptive-threshold sigma multipliers. Scale the derived noise floor by
// this factor to get the reject threshold. Larger = looser, more tolerant.
constexpr double GRAVITY_CHECK_SIGMA      = 6.0;
constexpr double ROTATION_DISAGREE_SIGMA  = 8.0;
constexpr double RUNAWAY_IMU_SIGMA        = 6.0;
constexpr double INIT_MAX_VEL_COEF        = 1.5;
constexpr double ZUPT_WEIGHT_SCALE        = 1.0;

// Init parallax in normalized-FOCAL pixel units (VINS convention, 460).
// Resolution-independent: feature_tracker normalizes features before
// publishing, so a different camera resolution needs no change here.
//
// Lowered from the fork's previous 18 px to 10 px to match stock VINS-Mono.
// Lower threshold = init succeeds with less motion, so the drone needs to
// move only a small amount at takeoff for the SfM-based init to find
// enough parallax. The strict checks downstream in visualInitialAlign
// (gravity / |V| / rotation disagreement) still catch the few low-parallax
// alignments that would actually be wrong.
constexpr double INIT_PARALLAX_PX = 10.0;

// Safety reset — re-initialize the estimator when all_image_frame grows
// due to low-parallax idle. The reset attempt count NEVER prevents the
// system from continuing; after it's exceeded we switch to in-place
// trimming and the estimator remains operational indefinitely. This
// means a drone can sit stationary "forever" without needing a human
// to restart the node.
constexpr double IDLE_RESET_SECONDS      = 15.0;
constexpr int    IDLE_RESET_MAX_ATTEMPTS = 5;

// Vehicle physics envelope. Universal for multirotors — no credible
// airframe exceeds these, so they serve as "state is diverged" kick-out
// criteria, not per-flight tuning.
constexpr double VEHICLE_MAX_ACCEL = 40.0;   // m/s² — ~4g, extreme maneuver
constexpr double VEHICLE_MAX_SPEED = 30.0;   // m/s  — 108 km/h, racing drone
}  // namespace hover


void readParameters(ros::NodeHandle &n);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};
