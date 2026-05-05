#include "parameters.h"

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string IMU_TOPIC;
double ROW, COL;
double TD, TR;

int    ENABLE_ZUPT;
int    ENABLE_ROTATION_ZUPT;
int    STATIC_INIT_BIAS_PRIMING;
int    SOFTEN_FAILURE_ON_HOVER;
double STATIC_ACC_THR;
double STATIC_GYR_THR;
double STATIC_FLOW_THR;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["imu_topic"] >> IMU_TOPIC;

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    std::string OUTPUT_PATH;
    fsSettings["output_path"] >> OUTPUT_PATH;
    VINS_RESULT_PATH = OUTPUT_PATH + "/vins_result_no_loop.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;

    // create folder if not exists
    FileSystemHelper::createDirectoryIfNotExists(OUTPUT_PATH.c_str());

    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %f COL: %f ", ROW, COL);

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";

    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
        ROS_INFO_STREAM("Extrinsic_R : " << std::endl << RIC[0]);
        ROS_INFO_STREAM("Extrinsic_T : " << std::endl << TIC[0].transpose());
        
    } 

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = fsSettings["rolling_shutter_tr"];
        ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << TR);
    }
    else
    {
        TR = 0;
    }

    // Hover-aware extensions.
    //
    // Only four YAML keys are supported here — feature switches. Everything
    // else is compile-time (see namespace hover in parameters.h) or derived
    // from the IMU noise floor just below.
    auto readOrInt = [&](const char *key, int def) -> int {
        cv::FileNode n = fsSettings[key];
        if (n.empty() || !n.isInt()) return def;
        return static_cast<int>(n);
    };

    ENABLE_ZUPT              = readOrInt("enable_zupt",              1);
    ENABLE_ROTATION_ZUPT     = readOrInt("enable_rotation_zupt",     1);
    STATIC_INIT_BIAS_PRIMING = readOrInt("static_init_bias_priming", 1);
    SOFTEN_FAILURE_ON_HOVER  = readOrInt("soften_failure_on_hover",  1);

    // Derived stationarity thresholds — computed from the just-loaded
    // calibrated IMU noise values. 15σ absorbs typical motor vibration
    // without bleeding into real-motion territory.
    //
    // Floors ensure we don't pick an absurdly low threshold when a
    // calibration reports suspiciously clean noise (sub-mg noise on a
    // consumer IMU is almost always mis-calibrated). Flow threshold is
    // in normalized-FOCAL pixels/sec and is camera-independent because
    // feature_tracker normalizes by the real camera intrinsics before
    // publishing.
    STATIC_ACC_THR  = std::max(15.0 * ACC_N, 0.3);    // m/s²
    STATIC_GYR_THR  = std::max(15.0 * GYR_N, 0.03);   // rad/s
    STATIC_FLOW_THR = 3.0;                            // px/s at FOCAL=460

    ROS_INFO("hover-aware: zupt=%d rot_zupt=%d prime=%d soften=%d",
             ENABLE_ZUPT, ENABLE_ROTATION_ZUPT, STATIC_INIT_BIAS_PRIMING,
             SOFTEN_FAILURE_ON_HOVER);
    ROS_INFO("hover-aware: derived thresholds acc=%.3f m/s² gyr=%.3f rad/s flow=%.1f px/s "
             "(from ACC_N=%.4f GYR_N=%.5f)",
             STATIC_ACC_THR, STATIC_GYR_THR, STATIC_FLOW_THR, ACC_N, GYR_N);

    fsSettings.release();
}
