#include <cstdio>
#include <cstdlib>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

const int SKIP = 50;
string benchmark_output_path;
string estimate_output_path;
template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

struct Data
{
    Data(FILE *f)
    {
        fscanf(f, " %lf,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &t,
               &px, &py, &pz,
               &qw, &qx, &qy, &qz,
               &vx, &vy, &vz,
               &wx, &wy, &wz,
               &ax, &ay, &az);
        t /= 1e9;
    }
    double t;
    float px, py, pz;
    float qw, qx, qy, qz;
    float vx, vy, vz;
    float wx, wy, wz;
    float ax, ay, az;
};
int idx = 1;
vector<Data> benchmark;

ros::Publisher pub_odom;
ros::Publisher pub_path;
nav_msgs::Path path;

int init = 0;
Quaterniond baseRgt;
Vector3d baseTgt;
tf::Transform trans;

void odom_callback(const nav_msgs::OdometryConstPtr &odom_msg)
{
    //ROS_INFO("odom callback!");
    if (odom_msg->header.stamp.toSec() > benchmark.back().t)
      return;
  
    for (; idx < static_cast<int>(benchmark.size()) && benchmark[idx].t <= odom_msg->header.stamp.toSec(); idx++)
        ;


    if (init++ < SKIP)
    {
        baseRgt = Quaterniond(odom_msg->pose.pose.orientation.w,
                              odom_msg->pose.pose.orientation.x,
                              odom_msg->pose.pose.orientation.y,
                              odom_msg->pose.pose.orientation.z) *
                  Quaterniond(benchmark[idx - 1].qw,
                              benchmark[idx - 1].qx,
                              benchmark[idx - 1].qy,
                              benchmark[idx - 1].qz).inverse();
        baseTgt = Vector3d{odom_msg->pose.pose.position.x,
                           odom_msg->pose.pose.position.y,
                           odom_msg->pose.pose.position.z} -
                  baseRgt * Vector3d{benchmark[idx - 1].px, benchmark[idx - 1].py, benchmark[idx - 1].pz};
        return;
    }

    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(benchmark[idx - 1].t);
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";

    Vector3d tmp_T = baseTgt + baseRgt * Vector3d{benchmark[idx - 1].px, benchmark[idx - 1].py, benchmark[idx - 1].pz};
    odometry.pose.pose.position.x = tmp_T.x();
    odometry.pose.pose.position.y = tmp_T.y();
    odometry.pose.pose.position.z = tmp_T.z();

    Quaterniond tmp_R = baseRgt * Quaterniond{benchmark[idx - 1].qw,
                                              benchmark[idx - 1].qx,
                                              benchmark[idx - 1].qy,
                                              benchmark[idx - 1].qz};
    odometry.pose.pose.orientation.w = tmp_R.w();
    odometry.pose.pose.orientation.x = tmp_R.x();
    odometry.pose.pose.orientation.y = tmp_R.y();
    odometry.pose.pose.orientation.z = tmp_R.z();

    Vector3d tmp_V = baseRgt * Vector3d{benchmark[idx - 1].vx,
                                        benchmark[idx - 1].vy,
                                        benchmark[idx - 1].vz};
    odometry.twist.twist.linear.x = tmp_V.x();
    odometry.twist.twist.linear.y = tmp_V.y();
    odometry.twist.twist.linear.z = tmp_V.z();
    pub_odom.publish(odometry);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = odometry.header;
    pose_stamped.pose = odometry.pose.pose;
    path.header = odometry.header;
    path.poses.push_back(pose_stamped);
    pub_path.publish(path);

    // write result to file
    {
        ofstream fout(benchmark_output_path, ios::app);
        fout.setf(ios::fixed, ios::floatfield);
        fout.precision(0);
        fout << benchmark[idx - 1].t * 1e9 << ",";
        fout.precision(5);
        fout  << benchmark[idx - 1].px << ","
              << benchmark[idx - 1].py << ","
              << benchmark[idx - 1].pz << ","
              << benchmark[idx - 1].qw << ","
              << benchmark[idx - 1].qx << ","
              << benchmark[idx - 1].qy << ","
              << benchmark[idx - 1].qz << ","
              << benchmark[idx - 1].vx << ","
              << benchmark[idx - 1].vy << ","
              << benchmark[idx - 1].vz << "," 
              << benchmark[idx - 1].wx << ","
              << benchmark[idx - 1].wy << ","
              << benchmark[idx - 1].wz << "," 
              << benchmark[idx - 1].ax << ","
              << benchmark[idx - 1].ay << ","
              << benchmark[idx - 1].az << "," 
              << endl;
        fout.close();
    }

    {
      Vector3d tmp_T = baseRgt.inverse() * (Vector3d{odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z} - baseTgt);
      Quaterniond tmp_R = baseRgt.inverse() * Quaterniond{odom_msg->pose.pose.orientation.w,
                                                odom_msg->pose.pose.orientation.x,
                                                odom_msg->pose.pose.orientation.y,
                                                odom_msg->pose.pose.orientation.z};
      Vector3d tmp_V = baseRgt.inverse() * Vector3d{odom_msg->twist.twist.linear.x,
                                          odom_msg->twist.twist.linear.y,
                                          odom_msg->twist.twist.linear.z};

        ofstream fout(estimate_output_path, ios::app);
        fout.setf(ios::fixed, ios::floatfield);
        fout.precision(0);
        fout << odom_msg->header.stamp.toSec() * 1e9 << ",";
        fout.precision(5);
        fout  << tmp_T.x() << ","
              << tmp_T.y() << ","
              << tmp_T.z() << ","
              << tmp_R.w() << ","
              << tmp_R.x() << ","
              << tmp_R.y() << ","
              << tmp_R.z() << ","
              << tmp_V.x() << ","
              << tmp_V.y() << ","
              << tmp_V.z() << "," 
              << odom_msg->pose.covariance[0] << "," 
              << odom_msg->pose.covariance[1] << "," 
              << odom_msg->pose.covariance[2] << "," 
              << odom_msg->pose.covariance[3] << "," 
              << odom_msg->pose.covariance[4] << "," 
              << odom_msg->pose.covariance[5] << "," 
              << endl;
        fout.close();
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "benchmark_publisher");
    ros::NodeHandle n("~");

    string data_dir = readParam<string>(n, "data_dir");
    string data_name = readParam<string>(n, "data_name");
    string csv_file = data_dir + data_name + "/data.csv";
    benchmark_output_path = readParam<string>(n, "output_path") + "benchmark.csv";
    estimate_output_path = readParam<string>(n, "output_path") + "estimate_align_benchmark.csv";
    ofstream fout_benchmark(benchmark_output_path, ios::out);
    fout_benchmark.close();
    ofstream fout_estimate(estimate_output_path, ios::out);
    fout_estimate.close();


    FILE *f = fopen(csv_file.c_str(), "r");
    char tmp[10000];
    fgets(tmp, 10000, f);
    while (!feof(f))
        benchmark.emplace_back(f);
    fclose(f);
    benchmark.pop_back();
    ROS_INFO("Data loaded: %d", (int)benchmark.size());

    pub_odom = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_path = n.advertise<nav_msgs::Path>("path", 1000);

    ros::Subscriber sub_odom = n.subscribe("estimated_odometry", 1000, odom_callback);
    
    ros::Rate r(20);
    ros::spin();
}
