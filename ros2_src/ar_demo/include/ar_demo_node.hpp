#ifndef __AR_DEMO_NODE_HPP__
#define __AR_DEMO_NODE_HPP__

#include <cmath>
#include <vector>
#include <queue>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/PinholeCamera.h"


using markerArrayMsg = visualization_msgs::msg::MarkerArray;
using pointCloudMsg = sensor_msgs::msg::PointCloud;
using odometryMsg = nav_msgs::msg::Odometry;
using imageMsg = sensor_msgs::msg::Image;

class ArDemo: public rclcpp::Node{
public:
    ArDemo();
    void axisGenerate(visualization_msgs::msg::Marker &line_list, Eigen::Vector3d &origin, int id);
    void cubeGenerate(visualization_msgs::msg::Marker &marker, Eigen::Vector3d &origin, int id);
    void addObject();
    void projectObject(Eigen::Vector3d camera_p, Eigen::Quaterniond camera_q);
    void drawObject(cv::Mat &AR_image);
    void callback(const imageMsg::SharedPtr img_msg, const odometryMsg::SharedPtr pose_msg);
    void pointCallback(const pointCloudMsg::SharedPtr point_msg);
    void imgCallback(const imageMsg::SharedPtr img_msg);
    void poseCallback(const odometryMsg::SharedPtr pose_msg);
    void setup();
    void getParam();
    void initTopic();
private:
    int row;
    int column;
    double focal_length;

    const int axis_num = 0;
    const int cube_num = 1;
    std::string calib_file;

    int img_cnt = 0;
    bool use_undistored_img = false;
    bool pose_init          = false;
    double box_length = 0.8;
    camodocal::CameraPtr m_camera;
    
    Eigen::Vector3d axis[6];
    Eigen::Vector3d cube_center[3];
    std::vector<Eigen::Vector3d> cube_corner[3];
    std::vector<Eigen::Vector3d> output_axis[6];
    std::vector<Eigen::Vector3d> output_cube[3];
    std::vector<double> output_corner_dis[3];
    double cube_center_depth[3];
    std::queue< imageMsg::SharedPtr> img_buf;
    bool look_ground = 0;
    std_msgs::msg::ColorRGBA line_color_r;
    std_msgs::msg::ColorRGBA line_color_g;
    std_msgs::msg::ColorRGBA line_color_b;

    rclcpp::Publisher<markerArrayMsg>::SharedPtr object_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_ARimage;
    rclcpp::Subscription<pointCloudMsg>::SharedPtr sub_point_cloud;        
    rclcpp::Subscription<odometryMsg>::SharedPtr sub_pose_img;
    rclcpp::Subscription<imageMsg>::SharedPtr sub_img;
};

#endif