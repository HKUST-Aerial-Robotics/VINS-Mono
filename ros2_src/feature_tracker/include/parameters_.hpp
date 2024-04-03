#ifndef __PARAMETERS_HPP__
#define __PARAMETERS_HPP__

#include "rclcpp/rclcpp.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "std_msgs/msg/header.hpp"

#define COLOR_BLK "\x1b[30m"
#define COLOR_RED "\x1b[31m"
#define COLOR_GRN "\x1b[32m"
#define COLOR_YLW "\x1b[33m"
#define COLOR_BLE "\x1b[34m"
#define COLOR_MGT "\x1b[35m"
#define COLOR_CYN "\x1b[36m"
#define COLOR_WHT "\x1b[37m"
#define COLOR_RST "\x1b[0m"

#define UNDERLINE "\x1b[21m"
#define LINE "================================================================================================="

using headerMsg = std_msgs::msg::Header;

extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
const int NUM_OF_CAM = 1;

extern std::string IMAGE_TOPIC;
extern std::string IMU_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;

void readParameters(std::string config_file, std::string VINS_FOLDER_PATH);
double toSec(headerMsg header_time);
#endif