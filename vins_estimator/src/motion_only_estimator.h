#pragma once
#include <ros/ros.h>
#include "parameters.h"
#include "utility/utility.h"
#include "utility/tic_toc.h"
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include <ceres/ceres.h>
#include "factor/imu_factor_pnp.h"
#include "factor/pose_local_parameterization.h"
#include "factor/perspective_factor.h"

#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace Eigen;

struct VINS_RESULT {
    double header;
    Vector3d Ba;
    Vector3d Bg;
    Vector3d P;
    Matrix3d R;
    Vector3d V;
    Vector3d tic;
    Matrix3d ric;
    Vector3d g;
};

struct IMG_MSG_LOCAL {
    int id;
    Vector2d observation;
    Vector3d position;
    int track_num;
};

struct IMU_MSG_LOCAL {
    double header;
    Vector3d acc;
    Vector3d gyr;
};

class MotionOnlyEstimator
{
public:
    
    typedef IMUFactorPnP IMUFactor_t;
    
    MotionOnlyEstimator();
    int frame_count;
    
    Vector3d g;
    Matrix3d ric;
    Vector3d tic;
    
    Vector3d Ps[(PNP_SIZE + 1)];
    Vector3d Vs[(PNP_SIZE + 1)];
    Matrix3d Rs[(PNP_SIZE + 1)];
    Vector3d Bas[(PNP_SIZE + 1)];
    Vector3d Bgs[(PNP_SIZE + 1)];
    
    double para_Pose[PNP_SIZE + 1][SIZE_POSE];
    double para_Speed[PNP_SIZE + 1][SIZE_SPEED];
    double para_Bias[PNP_SIZE + 1][SIZE_BIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];
    
    IntegrationBase *pre_integrations[(PNP_SIZE + 1)];
    vector<IMG_MSG_LOCAL> features[PNP_SIZE + 1];  //condition
    bool first_imu;
    Vector3d acc_0, gyr_0;
    vector<double> dt_buf[(PNP_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(PNP_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(PNP_SIZE + 1)];
    double Headers[(PNP_SIZE + 1)];
    bool find_solved[PNP_SIZE + 1];
    VINS_RESULT find_solved_vins[PNP_SIZE + 1];
    
    void solve_ceres();
    void old2new();
    void new2old();
    void clearState();
    void setIMUModel();
    void setInit(VINS_RESULT vins_result);
    void slideWindow();
    void updateFeatures(vector<IMG_MSG_LOCAL> &feature_msg);
    void processImage(vector<IMG_MSG_LOCAL> &feature_msg, double header);
    void processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void changeState();
};