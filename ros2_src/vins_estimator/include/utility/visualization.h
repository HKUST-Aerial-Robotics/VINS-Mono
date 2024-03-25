#ifndef __VISUALIZATION_HPP__
#define __VISUALIZATION_HPP__

#include <eigen3/Eigen/Dense>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"

#include "utility/CameraPoseVisualization.h"
#include "../estimator.h"
#include "../parameters_.h"

using navOdometryMsg = nav_msgs::msg::Odometry;
using navPathMsg = nav_msgs::msg::Path;
using pointCloudMsg =  sensor_msgs::msg::PointCloud;
using markerMsg = visualization_msgs::msg::Marker;
using markerArrayMsg = visualization_msgs::msg::MarkerArray;

extern rclcpp::Publisher<navOdometryMsg>::SharedPtr pub_odometry;
extern rclcpp::Publisher<navPathMsg>::SharedPtr pub_path, pub_pose;
extern rclcpp::Publisher<pointCloudMsg>::SharedPtr pub_cloud, pub_map;
extern rclcpp::Publisher<markerMsg>::SharedPtr pub_key_poses;

// extern rclcpp::Publisher<>::SharedPtr pub_ref_pose, pub_cur_pose;
// extern rclcpp::Publisher<>::SharedPtr pub_key;
// extern rclcpp::Publisher<>::SharedPtr pub_pose_graph;

// std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
// rclcpp::Node::SharedPtr node;

extern nav_msgs::msg::Path path;
extern int IMAGE_ROW, IMAGE_COL;

void registerPub(rclcpp::Node::SharedPtr n);

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, const std_msgs::msg::Header &header);

void printStatistics(const Estimator &estimator, double t);

void pubOdometry(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubInitialGuess(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubKeyPoses(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubCameraPose(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubPointCloud(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubTF(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubKeyframe(const Estimator &estimator);

void pubRelocalization(const Estimator &estimator);

#endif