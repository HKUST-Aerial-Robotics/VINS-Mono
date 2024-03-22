#ifndef __POSE_GRAPH_NODE_HPP__
#define __POSE_GRAPH_NODE_HPP__

#include <vector>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/bool.hpp"
#include "cv_bridge/cv_bridge.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "pose_graph.h"
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "parameters.h"
#include "utility/CameraPoseVisualization.h"

#define SKIP_FIRST_CNT 10

using namespace std;
using namespace DVision;
using namespace DBoW2;

using odometryMsg = nav_msgs::msg::Odometry;
using imageMsg = sensor_msgs::msg::Image;
using pointCloudMsg =  sensor_msgs::msg::PointCloud;

class PoseGraphNode: public rclcpp::Node{
public:
    PoseGraphNode();
    void newSequence();
    void imageCallback(const imageMsg::SharedPtr image_msg);
    void pointCallback(const pointCloudMsg::SharedPtr point_msg);
    void poseCallback(const odometryMsg::SharedPtr pose_msg);
    void imuForwardCallback(const odometryMsg::SharedPtr forward_msg);
    void reloRelativePoseCallback(const odometryMsg::SharedPtr pose_msg);
    void vioCallback(const odometryMsg::SharedPtr pose_msg);
    void extrinsicCallback(const odometryMsg::SharedPtr pose_msg);
    void process();
    void command();
    void initTopic();
    void getParams();
private:
    queue<imageMsg::SharedPtr> image_buf;
    queue<pointCloudMsg::SharedPtr> point_buf;
    queue<odometryMsg::SharedPtr> pose_buf;
    queue<Eigen::Vector3d> odometry_buf;
    nav_msgs::msg::Path no_loop_path;

    std::mutex m_buf;
    std::mutex m_process;
    int frame_index  = 0;
    int sequence = 1;
    // PoseGraph posegraph;
    int skip_first_cnt = 0;
    double last_image_time = -1;
    int SKIP_CNT;
    int skip_cnt = 0;
    bool load_flag = 0;
    bool start_flag = 0;
    double SKIP_DIS = 0;
    int VISUALIZE_IMU_FORWARD;
    int LOOP_CLOSURE;
    std::string IMAGE_TOPIC;
    std::string config_file;

    std::thread measurement_process;
    std::thread keyboard_command_process;

    rclcpp::Publisher<markerArrayMsg>::SharedPtr pub_camera_pose_visual;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_key_odometrys;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_vio_path;

    rclcpp::Subscription<odometryMsg>::SharedPtr sub_imu_forward;
    rclcpp::Subscription<odometryMsg>::SharedPtr sub_vio;
    rclcpp::Subscription<imageMsg>::SharedPtr sub_image;
    rclcpp::Subscription<odometryMsg>::SharedPtr sub_pose;
    rclcpp::Subscription<odometryMsg>::SharedPtr sub_extrinsic;
    rclcpp::Subscription<pointCloudMsg>::SharedPtr sub_point;
    rclcpp::Subscription<odometryMsg>::SharedPtr sub_relo_relative_pose;
    
};

#endif