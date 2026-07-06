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
double TD_ESTIMATION_PERIOD = 7.0; // seconds between td optimization bursts
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string IMU_TOPIC;
double ROW, COL;
double TD, TR;
int ENABLE_ZUPT;
double ZUPT_VEL_THRESHOLD;
double ZUPT_ACC_THRESHOLD;

// Altitude scale correction (MAVLink)
int ENABLE_ALTITUDE_SCALE_CORRECTION;
double ALTITUDE_CORRECTION_FACTOR;

// Lidar odometry initialization & scale correction
int ENABLE_LIDAR_INIT;
std::string LIDAR_ODOM_TOPIC;
double LIDAR_INIT_TIMEOUT;
double LIDAR_INIT_POS_THRESHOLD;
double LIDAR_SCALE_CORRECTION_FACTOR;
double LIDAR_SYNC_MAX_DT;
std::string WORLD_FRAME_ID;

template <typename T>
T readParam(rclcpp::Node::SharedPtr &n, std::string name)
{
    T ans;
    n->declare_parameter<T>(name, T{});
    n->get_parameter(name, ans);
    RCLCPP_INFO_STREAM(n->get_logger(), "Loaded " << name << ": " << ans);
    return ans;
}

void readParameters(rclcpp::Node::SharedPtr n)
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
    RCLCPP_INFO(n->get_logger(), "ROW: %f COL: %f ", ROW, COL);

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        RCLCPP_WARN(n->get_logger(), "have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";

    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            RCLCPP_WARN(n->get_logger(), " Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            RCLCPP_WARN(n->get_logger(), " fix extrinsic param ");

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
        RCLCPP_INFO_STREAM(n->get_logger(), "Extrinsic_R : " << std::endl << RIC[0]);
        RCLCPP_INFO_STREAM(n->get_logger(), "Extrinsic_T : " << std::endl << TIC[0].transpose());
        
    } 

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (!fsSettings["estimate_td_period"].empty())
        TD_ESTIMATION_PERIOD = (double)fsSettings["estimate_td_period"];
    if (ESTIMATE_TD)
        RCLCPP_INFO_STREAM(n->get_logger(), "Unsynchronized sensors, online estimate time offset, initial td: " << TD << ", period: " << TD_ESTIMATION_PERIOD << " s");
    else
        RCLCPP_INFO_STREAM(n->get_logger(), "Synchronized sensors, fix time offset: " << TD);

    ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = fsSettings["rolling_shutter_tr"];
        RCLCPP_INFO_STREAM(n->get_logger(), "rolling shutter camera, read out time per line: " << TR);
    }
    else
    {
        TR = 0;
    }
    
    // Zero Velocity Update parameters
    ENABLE_ZUPT = fsSettings["enable_zupt"].empty() ? 0 : (int)fsSettings["enable_zupt"];
    ZUPT_VEL_THRESHOLD = fsSettings["zupt_vel_threshold"].empty() ? 0.05 : (double)fsSettings["zupt_vel_threshold"];
    ZUPT_ACC_THRESHOLD = fsSettings["zupt_acc_threshold"].empty() ? 0.1 : (double)fsSettings["zupt_acc_threshold"];
    if (ENABLE_ZUPT)
        RCLCPP_INFO_STREAM(n->get_logger(), "Zero Velocity Update enabled, vel_threshold: " << ZUPT_VEL_THRESHOLD << ", acc_threshold: " << ZUPT_ACC_THRESHOLD);

    // Altitude scale correction (MAVLink)
    ENABLE_ALTITUDE_SCALE_CORRECTION = fsSettings["enable_altitude_scale_correction"].empty() ? 0 : (int)fsSettings["enable_altitude_scale_correction"];
    ALTITUDE_CORRECTION_FACTOR = fsSettings["altitude_correction_factor"].empty() ? 0.1 : (double)fsSettings["altitude_correction_factor"];
    if (ENABLE_ALTITUDE_SCALE_CORRECTION)
        RCLCPP_INFO_STREAM(n->get_logger(), "Altitude scale correction enabled, factor: " << ALTITUDE_CORRECTION_FACTOR);

    // Lidar odometry initialization & scale correction
    ENABLE_LIDAR_INIT = fsSettings["enable_lidar_init"].empty() ? 0 : (int)fsSettings["enable_lidar_init"];
    fsSettings["lidar_odom_topic"] >> LIDAR_ODOM_TOPIC;
    if (LIDAR_ODOM_TOPIC.empty()) LIDAR_ODOM_TOPIC = "/lio/odom";
    LIDAR_INIT_TIMEOUT = fsSettings["lidar_init_timeout"].empty() ? 30.0 : (double)fsSettings["lidar_init_timeout"];
    LIDAR_INIT_POS_THRESHOLD = fsSettings["lidar_init_pos_threshold"].empty() ? 1.0 : (double)fsSettings["lidar_init_pos_threshold"];
    LIDAR_SCALE_CORRECTION_FACTOR = fsSettings["lidar_scale_correction_factor"].empty() ? 0.05 : (double)fsSettings["lidar_scale_correction_factor"];
    LIDAR_SYNC_MAX_DT = fsSettings["lidar_sync_max_dt"].empty() ? 0.1 : (double)fsSettings["lidar_sync_max_dt"];
    if (ENABLE_LIDAR_INIT)
        RCLCPP_INFO_STREAM(n->get_logger(), "Lidar odometry init enabled, topic: " << LIDAR_ODOM_TOPIC
            << ", timeout: " << LIDAR_INIT_TIMEOUT << "s, pos_threshold: " << LIDAR_INIT_POS_THRESHOLD
            << "m, scale_factor: " << LIDAR_SCALE_CORRECTION_FACTOR << ", sync_dt: " << LIDAR_SYNC_MAX_DT << "s");

    // World frame ID for published odometry and visualization
    fsSettings["world_frame_id"] >> WORLD_FRAME_ID;
    if (WORLD_FRAME_ID.empty()) WORLD_FRAME_ID = "world";
    RCLCPP_INFO_STREAM(n->get_logger(), "World frame_id: " << WORLD_FRAME_ID);

    fsSettings.release();
}
