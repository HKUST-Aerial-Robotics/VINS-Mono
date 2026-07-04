#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <iostream>
#include <atomic>
#include <mutex>
#include <queue>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "pose_graph.h"
#include "utility/CameraPoseVisualization.h"
#include "parameters.h"
#define SKIP_FIRST_CNT 10
using namespace std;

queue<sensor_msgs::msg::Image::ConstSharedPtr> image_buf;
queue<sensor_msgs::msg::PointCloud::ConstSharedPtr> point_buf;
queue<nav_msgs::msg::Odometry::ConstSharedPtr> pose_buf;
queue<Eigen::Vector3d> odometry_buf;
std::mutex m_buf;
std::mutex m_process;
int frame_index  = 0;
int sequence = 1;
PoseGraph posegraph;
int skip_first_cnt = 0;
int SKIP_CNT;
int skip_cnt = 0;
bool load_flag = 0;
bool start_flag = 0;
double SKIP_DIS = 0;

int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
int ROW;
int COL;
int DEBUG_IMAGE;
int VISUALIZE_IMU_FORWARD;
int LOOP_CLOSURE;
int FAST_RELOCALIZATION;

camodocal::CameraPtr m_camera;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match_img;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_match_points;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_camera_pose_visual;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_key_odometrys;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_vio_path;
nav_msgs::msg::Path no_loop_path;

static double stampToSec(const builtin_interfaces::msg::Time &stamp)
{
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
std::string VINS_RESULT_PATH;
CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
Eigen::Vector3d last_t(-100, -100, -100);
double last_image_time = -1;
static std::string IMAGE_INPUT_FORMAT = "raw";
static std::atomic<uint64_t> pose_graph_image_count{0};
static std::atomic<uint64_t> pose_graph_point_count{0};
static std::atomic<uint64_t> pose_graph_pose_count{0};
static std::atomic<uint64_t> pose_graph_keyframe_count{0};

void new_sequence()
{
    printf("new sequence\n");
    sequence++;
    printf("sequence cnt %d \n", sequence);
    if (sequence > 5)
    {
        RCLCPP_ERROR(rclcpp::get_logger("pose_graph"), "only support 5 sequences");
        std::abort();
    }
    posegraph.posegraph_visualization->reset();
    posegraph.publish();
    m_buf.lock();
    while(!image_buf.empty())
        image_buf.pop();
    while(!point_buf.empty())
        point_buf.pop();
    while(!pose_buf.empty())
        pose_buf.pop();
    while(!odometry_buf.empty())
        odometry_buf.pop();
    m_buf.unlock();
}

void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg)
{
    //ROS_INFO("image_callback!");
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    image_buf.push(image_msg);
    const size_t queued_images = image_buf.size();
    m_buf.unlock();
    const auto count = ++pose_graph_image_count;
    if (count == 1 || count % 100 == 0)
        RCLCPP_INFO(rclcpp::get_logger("pose_graph"),
                    "[INPUT] image #%lu stamp=%.6f encoding=%s queue=%zu",
                    count, stampToSec(image_msg->header.stamp), image_msg->encoding.c_str(), queued_images);
    //printf(" image time %f \n", image_msg->header.stamp.toSec());

    // detect unstable camera stream
    if (last_image_time == -1)
        last_image_time = stampToSec(image_msg->header.stamp);
    else if (stampToSec(image_msg->header.stamp) - last_image_time > 1.0 || stampToSec(image_msg->header.stamp) < last_image_time)
    {
        RCLCPP_WARN(rclcpp::get_logger("pose_graph"), "image discontinue! detect a new sequence!");
        new_sequence();
    }
    last_image_time = stampToSec(image_msg->header.stamp);
}

void point_callback(const sensor_msgs::msg::PointCloud::ConstSharedPtr &point_msg)
{
    //ROS_INFO("point_callback!");
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    point_buf.push(point_msg);
    const size_t queued_points = point_buf.size();
    m_buf.unlock();
    const auto count = ++pose_graph_point_count;
    if (count == 1 || count % 20 == 0)
        RCLCPP_INFO(rclcpp::get_logger("pose_graph"),
                    "[INPUT] keyframe points #%lu stamp=%.6f points=%zu queue=%zu",
                    count, stampToSec(point_msg->header.stamp), point_msg->points.size(), queued_points);
    /*
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
    {
        printf("%d, 3D point: %f, %f, %f 2D point %f, %f \n",i , point_msg->points[i].x, 
                                                     point_msg->points[i].y,
                                                     point_msg->points[i].z,
                                                     point_msg->channels[i].values[0],
                                                     point_msg->channels[i].values[1]);
    }
    */
}

void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg)
{
    //ROS_INFO("pose_callback!");
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    pose_buf.push(pose_msg);
    const size_t queued_poses = pose_buf.size();
    m_buf.unlock();
    const auto count = ++pose_graph_pose_count;
    if (count == 1 || count % 20 == 0)
        RCLCPP_INFO(rclcpp::get_logger("pose_graph"),
                    "[INPUT] keyframe pose #%lu stamp=%.6f queue=%zu",
                    count, stampToSec(pose_msg->header.stamp), queued_poses);
    /*
    printf("pose t: %f, %f, %f   q: %f, %f, %f %f \n", pose_msg->pose.pose.position.x,
                                                       pose_msg->pose.pose.position.y,
                                                       pose_msg->pose.pose.position.z,
                                                       pose_msg->pose.pose.orientation.w,
                                                       pose_msg->pose.pose.orientation.x,
                                                       pose_msg->pose.pose.orientation.y,
                                                       pose_msg->pose.pose.orientation.z);
    */
}

void imu_forward_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &forward_msg)
{
    if (VISUALIZE_IMU_FORWARD)
    {
        Vector3d vio_t(forward_msg->pose.pose.position.x, forward_msg->pose.pose.position.y, forward_msg->pose.pose.position.z);
        Quaterniond vio_q;
        vio_q.w() = forward_msg->pose.pose.orientation.w;
        vio_q.x() = forward_msg->pose.pose.orientation.x;
        vio_q.y() = forward_msg->pose.pose.orientation.y;
        vio_q.z() = forward_msg->pose.pose.orientation.z;

        vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
        vio_q = posegraph.w_r_vio *  vio_q;

        vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
        vio_q = posegraph.r_drift * vio_q;

        Vector3d vio_t_cam;
        Quaterniond vio_q_cam;
        vio_t_cam = vio_t + vio_q * tic;
        vio_q_cam = vio_q * qic;        

        cameraposevisual.reset();
        cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
        cameraposevisual.publish_by(pub_camera_pose_visual, forward_msg->header);
    }
}
void relo_relative_pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg)
{
    Vector3d relative_t = Vector3d(pose_msg->pose.pose.position.x,
                                   pose_msg->pose.pose.position.y,
                                   pose_msg->pose.pose.position.z);
    Quaterniond relative_q;
    relative_q.w() = pose_msg->pose.pose.orientation.w;
    relative_q.x() = pose_msg->pose.pose.orientation.x;
    relative_q.y() = pose_msg->pose.pose.orientation.y;
    relative_q.z() = pose_msg->pose.pose.orientation.z;
    double relative_yaw = pose_msg->twist.twist.linear.x;
    int index = pose_msg->twist.twist.linear.y;
    //printf("receive index %d \n", index );
    Eigen::Matrix<double, 8, 1 > loop_info;
    loop_info << relative_t.x(), relative_t.y(), relative_t.z(),
                 relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                 relative_yaw;
    posegraph.updateKeyFrameLoop(index, loop_info);

}

void vio_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg)
{
    //ROS_INFO("vio_callback!");
    Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;

    vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
    vio_q = posegraph.w_r_vio *  vio_q;

    vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
    vio_q = posegraph.r_drift * vio_q;

    Vector3d vio_t_cam;
    Quaterniond vio_q_cam;
    vio_t_cam = vio_t + vio_q * tic;
    vio_q_cam = vio_q * qic;        

    if (!VISUALIZE_IMU_FORWARD)
    {
        cameraposevisual.reset();
        cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
        cameraposevisual.publish_by(pub_camera_pose_visual, pose_msg->header);
    }

    odometry_buf.push(vio_t_cam);
    if (odometry_buf.size() > 10)
    {
        odometry_buf.pop();
    }

    visualization_msgs::msg::Marker key_odometrys;
    key_odometrys.header = pose_msg->header;
    key_odometrys.header.frame_id = "world";
    key_odometrys.ns = "key_odometrys";
    key_odometrys.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    key_odometrys.action = visualization_msgs::msg::Marker::ADD;
    key_odometrys.pose.orientation.w = 1.0;
    key_odometrys.lifetime = builtin_interfaces::msg::Duration();

    //static int key_odometrys_id = 0;
    key_odometrys.id = 0; //key_odometrys_id++;
    key_odometrys.scale.x = 0.1;
    key_odometrys.scale.y = 0.1;
    key_odometrys.scale.z = 0.1;
    key_odometrys.color.r = 1.0;
    key_odometrys.color.a = 1.0;

    for (unsigned int i = 0; i < odometry_buf.size(); i++)
    {
        geometry_msgs::msg::Point pose_marker;
        Vector3d vio_t;
        vio_t = odometry_buf.front();
        odometry_buf.pop();
        pose_marker.x = vio_t.x();
        pose_marker.y = vio_t.y();
        pose_marker.z = vio_t.z();
        key_odometrys.points.push_back(pose_marker);
        odometry_buf.push(vio_t);
    }
    pub_key_odometrys->publish(key_odometrys);

    if (!LOOP_CLOSURE)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = pose_msg->header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = vio_t.x();
        pose_stamped.pose.position.y = vio_t.y();
        pose_stamped.pose.position.z = vio_t.z();
        no_loop_path.header = pose_msg->header;
        no_loop_path.header.frame_id = "world";
        no_loop_path.poses.push_back(pose_stamped);
        pub_vio_path->publish(no_loop_path);
    }
}

void extrinsic_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg)
{
    m_process.lock();
    tic = Vector3d(pose_msg->pose.pose.position.x,
                   pose_msg->pose.pose.position.y,
                   pose_msg->pose.pose.position.z);
    qic = Quaterniond(pose_msg->pose.pose.orientation.w,
                      pose_msg->pose.pose.orientation.x,
                      pose_msg->pose.pose.orientation.y,
                      pose_msg->pose.pose.orientation.z).toRotationMatrix();
    m_process.unlock();
}

void process()
{
    if (!LOOP_CLOSURE)
        return;
    while (true)
    {
        sensor_msgs::msg::Image::ConstSharedPtr image_msg;
        sensor_msgs::msg::PointCloud::ConstSharedPtr point_msg;
        nav_msgs::msg::Odometry::ConstSharedPtr pose_msg;

        // find out the messages with same time stamp
        m_buf.lock();
        if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            if (stampToSec(image_buf.front()->header.stamp) > stampToSec(pose_buf.front()->header.stamp))
            {
                pose_buf.pop();
                printf("throw pose at beginning\n");
            }
            else if (stampToSec(image_buf.front()->header.stamp) > stampToSec(point_buf.front()->header.stamp))
            {
                point_buf.pop();
                printf("throw point at beginning\n");
            }
            else if (stampToSec(image_buf.back()->header.stamp) >= stampToSec(pose_buf.front()->header.stamp)
                && stampToSec(point_buf.back()->header.stamp) >= stampToSec(pose_buf.front()->header.stamp))
            {
                pose_msg = pose_buf.front();
                pose_buf.pop();
                while (!pose_buf.empty())
                    pose_buf.pop();
                while (stampToSec(image_buf.front()->header.stamp) < stampToSec(pose_msg->header.stamp))
                {
                    image_buf.pop();
                    if (image_buf.empty())
                        break;
                }
                if (image_buf.empty())
                {
                    RCLCPP_WARN(rclcpp::get_logger("pose_graph"),
                                "[SYNC] image queue exhausted while matching keyframe pose");
                    m_buf.unlock();
                    continue;
                }
                image_msg = image_buf.front();
                image_buf.pop();

                while (stampToSec(point_buf.front()->header.stamp) < stampToSec(pose_msg->header.stamp))
                {
                    point_buf.pop();
                    if (point_buf.empty())
                        break;
                }
                if (point_buf.empty())
                {
                    RCLCPP_WARN(rclcpp::get_logger("pose_graph"),
                                "[SYNC] point queue exhausted while matching keyframe pose");
                    m_buf.unlock();
                    continue;
                }
                point_msg = point_buf.front();
                point_buf.pop();
            }
        }
        m_buf.unlock();

        if (pose_msg != NULL)
        {
            //printf(" pose time %f \n", pose_msg->header.stamp.toSec());
            //printf(" point time %f \n", point_msg->header.stamp.toSec());
            //printf(" image time %f \n", image_msg->header.stamp.toSec());
            // skip fisrt few
            if (skip_first_cnt < SKIP_FIRST_CNT)
            {
                skip_first_cnt++;
                continue;
            }

            if (skip_cnt < SKIP_CNT)
            {
                skip_cnt++;
                continue;
            }
            else
            {
                skip_cnt = 0;
            }

            cv_bridge::CvImageConstPtr ptr;
            if (image_msg->encoding == "8UC1")
            {
                sensor_msgs::msg::Image img;
                img.header = image_msg->header;
                img.height = image_msg->height;
                img.width = image_msg->width;
                img.is_bigendian = image_msg->is_bigendian;
                img.step = image_msg->step;
                img.data = image_msg->data;
                img.encoding = "mono8";
                ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            }
            else
                ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
            
            cv::Mat image = ptr->image;
            // build keyframe
            Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                                  pose_msg->pose.pose.position.y,
                                  pose_msg->pose.pose.position.z);
            Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                                     pose_msg->pose.pose.orientation.x,
                                     pose_msg->pose.pose.orientation.y,
                                     pose_msg->pose.pose.orientation.z).toRotationMatrix();
            if((T - last_t).norm() > SKIP_DIS)
            {
                vector<cv::Point3f> point_3d; 
                vector<cv::Point2f> point_2d_uv; 
                vector<cv::Point2f> point_2d_normal;
                vector<double> point_id;

                bool channels_valid = point_msg->channels.size() >= 5;
                if (channels_valid)
                {
                    for (size_t channel = 0; channel < 5; ++channel)
                        channels_valid = channels_valid &&
                            point_msg->channels[channel].values.size() >= point_msg->points.size();
                }
                if (!channels_valid)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("pose_graph"),
                                 "[INPUT] invalid keyframe point layout: points=%zu channels=%zu; keyframe dropped",
                                 point_msg->points.size(), point_msg->channels.size());
                    continue;
                }

                for (unsigned int i = 0; i < point_msg->points.size(); i++)
                {
                    cv::Point3f p_3d;
                    p_3d.x = point_msg->points[i].x;
                    p_3d.y = point_msg->points[i].y;
                    p_3d.z = point_msg->points[i].z;
                    point_3d.push_back(p_3d);

                    cv::Point2f p_2d_uv, p_2d_normal;
                    double p_id;
                    p_2d_normal.x = point_msg->channels[0].values[i];
                    p_2d_normal.y = point_msg->channels[1].values[i];
                    p_2d_uv.x = point_msg->channels[2].values[i];
                    p_2d_uv.y = point_msg->channels[3].values[i];
                    p_id = point_msg->channels[4].values[i];
                    point_2d_normal.push_back(p_2d_normal);
                    point_2d_uv.push_back(p_2d_uv);
                    point_id.push_back(p_id);

                    //printf("u %f, v %f \n", p_2d_uv.x, p_2d_uv.y);
                }

                KeyFrame* keyframe = new KeyFrame(stampToSec(pose_msg->header.stamp), frame_index, T, R, image,
                                   point_3d, point_2d_uv, point_2d_normal, point_id, sequence);   
                m_process.lock();
                start_flag = 1;
                posegraph.addKeyFrame(keyframe, 1);
                m_process.unlock();
                frame_index++;
                const auto count = ++pose_graph_keyframe_count;
                if (count == 1 || count % 10 == 0)
                    RCLCPP_INFO(rclcpp::get_logger("pose_graph"),
                                "[OUTPUT] accepted keyframe #%lu stamp=%.6f features=%zu",
                                count, stampToSec(pose_msg->header.stamp), point_3d.size());
                last_t = T;
            }
        }

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

void command()
{
    if (!LOOP_CLOSURE)
        return;
    while(1)
    {
        char c = getchar();
        if (c == 's')
        {
            m_process.lock();
            posegraph.savePoseGraph();
            m_process.unlock();
            printf("save pose graph finish\nyou can set 'load_previous_pose_graph' to 1 in the config file to reuse it next time\n");
            // printf("program shutting down...\n");
            // ros::shutdown();
        }
        if (c == 'n')
            new_sequence();

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pose_graph");
    posegraph.registerPub(node);

    // read param
    VISUALIZATION_SHIFT_X = node->declare_parameter<int>("visualization_shift_x", 0);
    VISUALIZATION_SHIFT_Y = node->declare_parameter<int>("visualization_shift_y", 0);
    SKIP_CNT = node->declare_parameter<int>("skip_cnt", 0);
    SKIP_DIS = node->declare_parameter<double>("skip_dis", 0.0);
    std::string config_file;
    config_file = node->declare_parameter<std::string>("config_file", "");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    double camera_visual_size = fsSettings["visualize_camera_size"];
    cameraposevisual.setScale(camera_visual_size);
    cameraposevisual.setLineWidth(camera_visual_size / 10.0);


    LOOP_CLOSURE = fsSettings["loop_closure"];
    std::string IMAGE_TOPIC;
    int LOAD_PREVIOUS_POSE_GRAPH;
    if (LOOP_CLOSURE)
    {
        ROW = fsSettings["image_height"];
        COL = fsSettings["image_width"];
        std::string pkg_path = ament_index_cpp::get_package_share_directory("pose_graph");
        string vocabulary_file = pkg_path + "/support_files/brief_k10L6.bin";
        cout << "vocabulary_file" << vocabulary_file << endl;
        posegraph.loadVocabulary(vocabulary_file);

        BRIEF_PATTERN_FILE = pkg_path + "/support_files/brief_pattern.yml";
        cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << endl;
        m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file.c_str());

        fsSettings["image_topic"] >> IMAGE_TOPIC;
        if (!fsSettings["image_input_format"].empty())
            fsSettings["image_input_format"] >> IMAGE_INPUT_FORMAT;
        fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
        fsSettings["output_path"] >> VINS_RESULT_PATH;
        fsSettings["save_image"] >> DEBUG_IMAGE;

        // create folder if not exists
        FileSystemHelper::createDirectoryIfNotExists(POSE_GRAPH_SAVE_PATH.c_str());
        FileSystemHelper::createDirectoryIfNotExists(VINS_RESULT_PATH.c_str());

        VISUALIZE_IMU_FORWARD = fsSettings["visualize_imu_forward"];
        LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"];
        FAST_RELOCALIZATION = fsSettings["fast_relocalization"];
        VINS_RESULT_PATH = VINS_RESULT_PATH + "/vins_result_loop.csv";
        std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
        fout.close();
        fsSettings.release();

        if (LOAD_PREVIOUS_POSE_GRAPH)
        {
            printf("load pose graph\n");
            m_process.lock();
            posegraph.loadPoseGraph();
            m_process.unlock();
            printf("load pose graph finish\n");
            load_flag = 1;
        }
        else
        {
            printf("no previous pose graph\n");
            load_flag = 1;
        }
    }

    fsSettings.release();
    RCLCPP_INFO(node->get_logger(), "[START] pose_graph image_input_format=%s image_topic=%s", IMAGE_INPUT_FORMAT.c_str(), IMAGE_TOPIC.c_str());

    auto sub_imu_forward = node->create_subscription<nav_msgs::msg::Odometry>("/vins_estimator/imu_propagate", 2000, imu_forward_callback);
    auto sub_vio = node->create_subscription<nav_msgs::msg::Odometry>("/vins_estimator/odometry", 2000, vio_callback);
    auto sub_image = image_transport::create_subscription(
        node.get(), IMAGE_TOPIC, image_callback, IMAGE_INPUT_FORMAT);
    RCLCPP_INFO(
        node->get_logger(),
        "[START] pose_graph image_transport subscription created for base_topic=%s transport=%s",
        IMAGE_TOPIC.c_str(), IMAGE_INPUT_FORMAT.c_str());
    auto sub_pose = node->create_subscription<nav_msgs::msg::Odometry>("/vins_estimator/keyframe_pose", 2000, pose_callback);
    auto sub_extrinsic = node->create_subscription<nav_msgs::msg::Odometry>("/vins_estimator/extrinsic", 2000, extrinsic_callback);
    auto sub_point = node->create_subscription<sensor_msgs::msg::PointCloud>("/vins_estimator/keyframe_point", 2000, point_callback);
    auto sub_relo_relative_pose = node->create_subscription<nav_msgs::msg::Odometry>("/vins_estimator/relo_relative_pose", 2000, relo_relative_pose_callback);

    pub_match_img = node->create_publisher<sensor_msgs::msg::Image>("/pose_graph/match_image", 1000);
    pub_camera_pose_visual = node->create_publisher<visualization_msgs::msg::MarkerArray>("/pose_graph/camera_pose_visual", 1000);
    pub_key_odometrys = node->create_publisher<visualization_msgs::msg::Marker>("/pose_graph/key_odometrys", 1000);
    pub_vio_path = node->create_publisher<nav_msgs::msg::Path>("/pose_graph/no_loop_path", 1000);
    pub_match_points = node->create_publisher<sensor_msgs::msg::PointCloud>("/pose_graph/match_points", 100);

    std::thread measurement_process;
    std::thread keyboard_command_process;

    measurement_process = std::thread(process);
    keyboard_command_process = std::thread(command);


    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
