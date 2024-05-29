#ifndef __PARAMETERS_H_
#define __PARAMETERS_H_

#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/header.hpp"

#include <cv_bridge/cv_bridge.h>

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

extern camodocal::CameraPtr m_camera;
extern Eigen::Vector3d tic;
extern Eigen::Matrix3d qic;
extern rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match_img;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_match_points;
extern int VISUALIZATION_SHIFT_X;
extern int VISUALIZATION_SHIFT_Y;
extern std::string BRIEF_PATTERN_FILE;
extern std::string POSE_GRAPH_SAVE_PATH;
extern int ROW;
extern int COL;
extern std::string VINS_RESULT_PATH;
extern int DEBUG_IMAGE;
extern int FAST_RELOCALIZATION;
// extern std::string IMAGE_TOPIC;

double toSec(headerMsg header_time);

#endif