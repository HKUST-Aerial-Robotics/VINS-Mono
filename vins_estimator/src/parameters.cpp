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
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
int LOOP_CLOSURE = 0;
int MIN_LOOP_NUM;
std::string CAM_NAMES;
std::string PATTERN_FILE;
std::string VOC_FILE;
std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
int IMAGE_ROW, IMAGE_COL;
std::string VINS_FOLDER_PATH;
int MAX_KEYFRAME_NUM;

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

    VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");
    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["imu_topic"] >> IMU_TOPIC;

    IMAGE_COL = fsSettings["image_width"];
    IMAGE_ROW = fsSettings["image_height"];

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    fsSettings["output_path"] >> VINS_RESULT_PATH;
    VINS_RESULT_PATH = VINS_FOLDER_PATH + VINS_RESULT_PATH;
    std::ofstream foutC(VINS_RESULT_PATH, std::ios::out);
    foutC.close();

    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        fsSettings["ex_calib_result_path"] >> EX_CALIB_RESULT_PATH;
        EX_CALIB_RESULT_PATH = VINS_FOLDER_PATH + EX_CALIB_RESULT_PATH;

    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            fsSettings["ex_calib_result_path"] >> EX_CALIB_RESULT_PATH;
            EX_CALIB_RESULT_PATH = VINS_FOLDER_PATH + EX_CALIB_RESULT_PATH;
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



    LOOP_CLOSURE = fsSettings["loop_closure"];
    if (LOOP_CLOSURE == 1)
    {
        fsSettings["voc_file"] >> VOC_FILE;;
        fsSettings["pattern_file"] >> PATTERN_FILE;
        VOC_FILE = VINS_FOLDER_PATH + VOC_FILE;
        PATTERN_FILE = VINS_FOLDER_PATH + PATTERN_FILE;
        MIN_LOOP_NUM = fsSettings["min_loop_num"];
        CAM_NAMES = config_file;
    }


    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;
    MAX_KEYFRAME_NUM = 1000;
    
    fsSettings.release();
}
