#include "parameters.h"
#include "rclcpp/rclcpp.hpp"

std::string IMAGE_TOPIC;
std::string IMAGE_INPUT_FORMAT;
std::string IMU_TOPIC;
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;
int FAST_THRESHOLD;
int USE_BIDIRECTIONAL_FLOW;
int USE_ADVANCED_FLOW;
double GFTT_QUALITY_LEVEL = 0.01;

// Adaptive feature tracking
double MAX_VELOCITY_THRESHOLD = 5.0;  // m/s
int VELOCITY_BOOST_FEATURES = 50;     // extra features at high speed
double MIN_PARALLAX_THRESHOLD = 5.0;  // pixels
int ENABLE_VELOCITY_CHECK = 1;

void readParameters(rclcpp::Node::SharedPtr &n)
{
    std::string config_file;
    n->declare_parameter<std::string>("config_file", "");
    n->get_parameter("config_file", config_file);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    std::string VINS_FOLDER_PATH;
    n->declare_parameter<std::string>("vins_folder", "");
    n->get_parameter("vins_folder", VINS_FOLDER_PATH);

    fsSettings["image_topic"] >> IMAGE_TOPIC;
    if (!fsSettings["image_input_format"].empty())
        fsSettings["image_input_format"] >> IMAGE_INPUT_FORMAT;
    else
        IMAGE_INPUT_FORMAT = "raw";
    fsSettings["imu_topic"] >> IMU_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];
    
    // Optional parameter for FAST threshold, default to 20 if not set
    if (!fsSettings["fast_threshold"].empty())
        FAST_THRESHOLD = fsSettings["fast_threshold"];
    else
        FAST_THRESHOLD = 20;
    
    // Optional parameter for bidirectional flow, default to 1 (enabled) if not set
    if (!fsSettings["use_bidirectional_flow"].empty())
        USE_BIDIRECTIONAL_FLOW = fsSettings["use_bidirectional_flow"];
    else
        USE_BIDIRECTIONAL_FLOW = 1;

    // Switch between basic and advanced optical flow (default: advanced)
    if (!fsSettings["use_advanced_flow"].empty())
        USE_ADVANCED_FLOW = fsSettings["use_advanced_flow"];
    else
        USE_ADVANCED_FLOW = 1;

    if (!fsSettings["gftt_quality_level"].empty())
        GFTT_QUALITY_LEVEL = (double)fsSettings["gftt_quality_level"];
    else
        GFTT_QUALITY_LEVEL = 0.01;
    
    // Adaptive tracking parameters
    ENABLE_VELOCITY_CHECK = fsSettings["enable_velocity_check"].empty() ? 1 : (int)fsSettings["enable_velocity_check"];
    if (!fsSettings["max_velocity_threshold"].empty())
        MAX_VELOCITY_THRESHOLD = (double)fsSettings["max_velocity_threshold"];
    if (!fsSettings["velocity_boost_features"].empty())
        VELOCITY_BOOST_FEATURES = (int)fsSettings["velocity_boost_features"];
    if (!fsSettings["min_parallax_threshold"].empty())
        MIN_PARALLAX_THRESHOLD = (double)fsSettings["min_parallax_threshold"];
    
    if (ENABLE_VELOCITY_CHECK)
        RCLCPP_INFO(n->get_logger(), "Velocity-adaptive tracking enabled: max_vel=%.1f m/s, boost=%d features",
                 MAX_VELOCITY_THRESHOLD, VELOCITY_BOOST_FEATURES);

    RCLCPP_INFO(n->get_logger(), "Image input mode: %s", IMAGE_INPUT_FORMAT.c_str());
    RCLCPP_INFO(n->get_logger(), "Image topic: %s", IMAGE_TOPIC.c_str());
    RCLCPP_INFO(n->get_logger(),
                "Feature detector: %s, quality=%.4f, min_dist=%d, max_cnt=%d",
                USE_ADVANCED_FLOW ? "AGAST" : "Shi-Tomasi", GFTT_QUALITY_LEVEL,
                MIN_DIST, MAX_CNT);

    if (FISHEYE == 1)
    {
        std::string mask_path;
        if (!fsSettings["fisheye_mask"].empty())
            fsSettings["fisheye_mask"] >> mask_path;
        else
            mask_path = "config/fisheye_mask.jpg";

        if (!mask_path.empty() && mask_path.front() == '/')
            FISHEYE_MASK = mask_path;
        else
            FISHEYE_MASK = VINS_FOLDER_PATH + mask_path;
        RCLCPP_INFO(n->get_logger(), "Feature mask: %s", FISHEYE_MASK.c_str());
    }
    CAM_NAMES.push_back(config_file);

    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();


}
