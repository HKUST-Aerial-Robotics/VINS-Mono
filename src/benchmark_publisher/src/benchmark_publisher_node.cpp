#include "benchmark_publisher_node.hpp"

BenchmarkPublisherNode::BenchmarkPublisherNode(): Node("benchmark_publisher_node"){
    initTopic();
    trans = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    csv_file = readParam<std::string>(*this, "data_name");
    std::cout << "load ground truth " << csv_file << std::endl;
    readFile();
}

void BenchmarkPublisherNode::initTopic(){
    pub_odom = this->create_publisher<odometryMsg>("odometry", 100);
    pub_path = this->create_publisher<pathMsg>("path", 100);

    sub_odom = this->create_subscription<odometryMsg>("estimated_odometry", 100, 
                            std::bind(&BenchmarkPublisherNode::odomCallback, this, std::placeholders::_1));
}

void BenchmarkPublisherNode::readFile(){
    FILE *f = fopen(csv_file.c_str(), "r");
    if(f == NULL){
        RCLCPP_ERROR(this->get_logger(), "can't load ground truth; wrong path %s", csv_file);
        return;
    }

    char tmp[10000];
    if (fgets(tmp, 10000, f) == NULL){
        RCLCPP_WARN(this->get_logger(), "can't load ground truth; no data available");
    }

     while (!feof(f)){
        benchmark.emplace_back(f);
    }

    fclose(f);
    benchmark.pop_back();
    RCLCPP_INFO(this->get_logger(), "Data Loaded: %d", (int)benchmark.size());
}

void BenchmarkPublisherNode::odomCallback(const odometryMsg::SharedPtr odom_msg){
    if (odom_msg->header.stamp.sec > benchmark.back().t)
      return;
  
    for (; idx < static_cast<int>(benchmark.size()) && benchmark[idx].t <= odom_msg->header.stamp.sec; idx++);

    if (init++ < SKIP) {
        baseRgt = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                              odom_msg->pose.pose.orientation.x,
                              odom_msg->pose.pose.orientation.y,
                              odom_msg->pose.pose.orientation.z) *
                  Eigen::Quaterniond(benchmark[idx - 1].qw,
                              benchmark[idx - 1].qx,
                              benchmark[idx - 1].qy,
                              benchmark[idx - 1].qz).inverse();
        baseTgt = Eigen::Vector3d{odom_msg->pose.pose.position.x,
                           odom_msg->pose.pose.position.y,
                           odom_msg->pose.pose.position.z} -
                  baseRgt * Eigen::Vector3d{benchmark[idx - 1].px, benchmark[idx - 1].py, benchmark[idx - 1].pz};
        return;
    }

    odometryMsg odometry;
    double time_sec = benchmark[idx - 1].t;
    double time_nsec = (time_sec - floor(time_sec)) * 1e9; 

    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(time_sec); 
    time_msg.nanosec = static_cast<uint32_t>(time_nsec);
    odometry.header.stamp = time_msg; // this->get_clock()->now()
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";

    Eigen::Vector3d tmp_T = baseTgt + baseRgt * Eigen::Vector3d{benchmark[idx - 1].px, benchmark[idx - 1].py, 
                                                                                            benchmark[idx - 1].pz};
    odometry.pose.pose.position.x = tmp_T.x();
    odometry.pose.pose.position.y = tmp_T.y();
    odometry.pose.pose.position.z = tmp_T.z();

    Eigen::Quaterniond tmp_R = baseRgt * Eigen::Quaterniond{benchmark[idx - 1].qw,
                                              benchmark[idx - 1].qx,
                                              benchmark[idx - 1].qy,
                                              benchmark[idx - 1].qz};
    odometry.pose.pose.orientation.w = tmp_R.w();
    odometry.pose.pose.orientation.x = tmp_R.x();
    odometry.pose.pose.orientation.y = tmp_R.y();
    odometry.pose.pose.orientation.z = tmp_R.z();

    Eigen::Vector3d tmp_V = baseRgt * Eigen::Vector3d{benchmark[idx - 1].vx,
                                        benchmark[idx - 1].vy,
                                        benchmark[idx - 1].vz};
    odometry.twist.twist.linear.x = tmp_V.x();
    odometry.twist.twist.linear.y = tmp_V.y();
    odometry.twist.twist.linear.z = tmp_V.z();
    pub_odom->publish(odometry);

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = odometry.header;
    pose_stamped.pose = odometry.pose.pose;
    path.header = odometry.header;
    path.poses.push_back(pose_stamped);
    pub_path->publish(path);
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BenchmarkPublisherNode>());
    rclcpp::shutdown();
    return 0;
}