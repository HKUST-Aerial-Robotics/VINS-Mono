#pragma once

#include "rclcpp/rclcpp.hpp"
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
extern double TD_ESTIMATION_PERIOD; // seconds between td optimization bursts
extern int ROLLING_SHUTTER;
extern double ROW, COL;
extern int ENABLE_ZUPT;
extern double ZUPT_VEL_THRESHOLD;
extern double ZUPT_ACC_THRESHOLD;

// Altitude scale correction (MAVLink)
extern int ENABLE_ALTITUDE_SCALE_CORRECTION;
extern double ALTITUDE_CORRECTION_FACTOR;

// Lidar odometry initialization & scale correction
extern int ENABLE_LIDAR_INIT;
extern std::string LIDAR_ODOM_TOPIC;
extern double LIDAR_INIT_TIMEOUT;          // seconds to wait for lidar odom at startup
extern double LIDAR_INIT_POS_THRESHOLD;    // min displacement (m) before using lidar for scale init
extern double LIDAR_SCALE_CORRECTION_FACTOR; // gentle ongoing scale correction strength
extern double LIDAR_SYNC_MAX_DT;           // max time diff (s) for lidar-vins sync

extern std::string WORLD_FRAME_ID;        // TF/frame_id for published odometry and visualization

void readParameters(rclcpp::Node::SharedPtr n);

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
