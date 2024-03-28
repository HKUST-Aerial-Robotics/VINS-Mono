#pragma once

#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include "rclcpp/rclcpp.hpp"

#include "utility/utility.h"

#define COLOR_BLK "\x1b[30m"
#define COLOR_RED "\x1b[31m"
#define COLOR_GRN "\x1b[32m"
#define COLOR_YLW "\x1b[33m"
#define COLOR_BLE "\x1b[34m"
#define COLOR_MGT "\x1b[35m"
#define COLOR_CYN "\x1b[36m"
#define COLOR_WHT "\x1b[37m"
#define COLOR_RST "\x1b[0m"

#define BLACK_STRIPE "\x1b[40m"
#define RED_STRIPE "\x1b[41m"
#define GREEN_STRIPE "\x1b[42m"
#define YELLOW_STRIPE "\x1b[43m"
#define BLUE_STRIPE "\x1b[44m"
#define MAGENTA_STRIPE "\x1b[45m"
#define CYAN_STRIPE "\x1b[46m"
#define WHITE_STRIPE "\x1b[47m"

#define UNDERLINE "\x1b[21m"
#define LINE "================================================================================================="


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
