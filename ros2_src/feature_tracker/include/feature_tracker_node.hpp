#ifndef  __FEATURE_TRACKER_NODE_HPP__
#define __FEATURE_TRACKER_NODE_HPP__
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/channel_float32.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "cv_bridge/cv_bridge.h"
#include "message_filters/subscriber.h"

#include "feature_tracker.hpp"

#define SHOW_UNDISTORTION 0
#define MY_VARIABLE_STRING "${MY_VARIABLE}"

using imuMsg = sensor_msgs::msg::Imu;
using imageMsg = sensor_msgs::msg::Image;
using pointCloudMsg = sensor_msgs::msg::PointCloud;
using boolMsg = std_msgs::msg::Bool;

class FeatureTrackerNode : public rclcpp::Node{
public:
    FeatureTrackerNode();
    void imgCallback(const imageMsg::SharedPtr img_msg);
    void initTopic();
    void getParams();
private:
    std::vector<uchar> r_status;
    std::vector<float> r_err;
    std::queue<imageMsg::SharedPtr> img_buf;

    rclcpp::Publisher<pointCloudMsg>::SharedPtr pub_img;
    rclcpp::Publisher<imageMsg>::SharedPtr pub_match;
    rclcpp::Publisher<boolMsg>::SharedPtr pub_restart;
    rclcpp::Subscription<imageMsg>::SharedPtr sub_img;

    FeatureTracker trackerData[NUM_OF_CAM];
    double first_image_time;
    int pub_count = 1;
    bool first_image_flag = true;
    double last_image_time = 0.0;
    bool init_pub = 0;
    double current_time = 0.0;
};

#endif