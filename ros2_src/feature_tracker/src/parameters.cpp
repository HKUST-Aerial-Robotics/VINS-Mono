#include "parameters_.hpp"

std::string IMAGE_TOPIC;
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


void readParameters(std::string config_file, std::string VINS_FOLDER_PATH){
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["image_topic"] >> IMAGE_TOPIC;
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
    if (FISHEYE == 1)
        FISHEYE_MASK = VINS_FOLDER_PATH + "config/fisheye_mask.jpg";
    CAM_NAMES.push_back(config_file);

    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("parameters"), 
        COLOR_GRN 
        << "\n" << LINE
        << "\n- IMU_TOPIC: " << IMU_TOPIC 
        << "\n- IMAGE_TOPIC: " << IMAGE_TOPIC << std::endl
        << "\n- ROW: " << ROW
        << "\n- COL: " << COL
        << "\n- FREQ: " << FREQ
        << "\n- FISHEYE: " << FISHEYE
        << "\n- MAX_CNT: " << MAX_CNT
        << "\n- EQUALIZE: " << EQUALIZE
        << "\n- MIN_DIST: " << MIN_DIST
        << "\n- SHOW_TRACK: " << SHOW_TRACK
        << "\n- F_THRESHOLD: " << F_THRESHOLD
        << "\n- WINDOW_SIZE: " << WINDOW_SIZE
        << "\n- STEREO_TRACK: " << STEREO_TRACK
        << "\n- FOCAL_LENGTH: " << FOCAL_LENGTH
        << "\n- PUB_THIS_FRAME: " << PUB_THIS_FRAME << std::endl
        << "\n- VINS_FOLDER_PATH: " << VINS_FOLDER_PATH
        << "\n- config_file: " << config_file
        << "\n" << LINE << std::endl
        << COLOR_RST
    );
}

double toSec(headerMsg header_time){
    return static_cast<double>(header_time.stamp.sec + (header_time.stamp.nanosec /  1.0e9));
}