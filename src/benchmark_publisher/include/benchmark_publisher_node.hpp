#ifndef __BENCHMARK_PUBLISHER_NODE_HPP__
#define __BENCHMARK_PUBLISHER_NODE_HPP__

#include <cstdio>
#include <vector>
#include <fstream>
#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_ros/transform_broadcaster.h"

using odometryMsg = nav_msgs::msg::Odometry;
using pathMsg = nav_msgs::msg::Path;

const int SKIP = 50;

// template <typename T>
// T readParam(rclcpp::Node &n, std::string name){
//     T ans;
//     if (n.get_parameter(name, ans)) {
//         std::cout << "Loaded " << name << ": " << ans << std::endl;
//     }
//     else {
//         std::cout << "Failed to load " << name << std::endl;
//         rclcpp::shutdown();
//     }
//     return ans;
// }

struct Data{
    Data(FILE *f) {
        if (fscanf(f, " %lf,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &t,
               &px, &py, &pz,
               &qw, &qx, &qy, &qz,
               &vx, &vy, &vz,
               &wx, &wy, &wz,
               &ax, &ay, &az) != EOF){
            t /= 1e9;
        }
    }
    double t;
    float px, py, pz;
    float qw, qx, qy, qz;
    float vx, vy, vz;
    float wx, wy, wz;
    float ax, ay, az;
};

class BenchmarkPublisherNode: public rclcpp::Node{
public:
    BenchmarkPublisherNode();
    void initTopic();
    void odomCallback(const odometryMsg::SharedPtr odom_msg);
    void readFile();

private:
    int idx = 1;
    std::vector<Data> benchmark;
    std::string csv_file;

    rclcpp::Publisher<pathMsg>::SharedPtr pub_path;
    rclcpp::Publisher<odometryMsg>::SharedPtr pub_odom;
    rclcpp::Subscription<odometryMsg>::SharedPtr sub_odom;
    std::unique_ptr<tf2_ros::TransformBroadcaster> trans;
    pathMsg path;

    int init = 0;
    Eigen::Quaterniond baseRgt;
    Eigen::Vector3d baseTgt;
};

#endif