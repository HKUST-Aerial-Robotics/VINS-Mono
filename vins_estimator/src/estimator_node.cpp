#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/header.hpp"

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"


Estimator estimator;

std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buf;
queue<sensor_msgs::msg::PointCloud::ConstSharedPtr> feature_buf;
queue<sensor_msgs::msg::PointCloud::ConstSharedPtr> relo_buf;
int sum_of_wait = 0;

std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_estimator;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = 0;
static bool first_feature_logged = false;
static int feature_msg_count = 0;
static int imu_msg_count = 0;

// IMU filtering variables
Eigen::Vector3d prev_acc(0, 0, 0);
Eigen::Vector3d filtered_acc(0, 0, 0);
bool acc_initialized = false;
const double MAX_ACC_CHANGE = 50.0;
const double ACC_FILTER_ALPHA = 0.8;

// Altitude-based scale correction from MAVLink velocity
double mavlink_altitude = 0.0;
double mavlink_vz = 0.0;
bool mavlink_odom_received = false;
std::mutex m_odom;

// Lidar odometry initialization & scale correction
struct LidarOdomSample
{
    double time;
    Eigen::Vector3d pos;
    Eigen::Matrix3d rot;
};
std::queue<LidarOdomSample> lidar_odom_buf;
std::mutex m_lidar;
bool lidar_odom_received = false;
double lidar_first_recv_time = -1.0;

static inline double stamp_to_sec(const builtin_interfaces::msg::Time &stamp)
{
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

void predict(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
{
    double t = stamp_to_sec(imu_msg->header.stamp);
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};
    
    // Accelerometer filtering: outlier rejection + exponential smoothing
    if (!acc_initialized) {
        prev_acc = linear_acceleration;
        filtered_acc = linear_acceleration;
        acc_initialized = true;
    } else {
        // Step 1: Outlier detection - reject sudden spikes
        Eigen::Vector3d acc_diff = linear_acceleration - prev_acc;
        if (acc_diff.norm() > MAX_ACC_CHANGE) {
            RCLCPP_WARN(rclcpp::get_logger("vins_estimator"), "IMU accelerometer spike detected: %.2f m/s² change, using filtered value", acc_diff.norm());
            linear_acceleration = filtered_acc;
        }
        
        // Step 2: Exponential moving average filter for smoothing
        filtered_acc = ACC_FILTER_ALPHA * filtered_acc + (1.0 - ACC_FILTER_ALPHA) * linear_acceleration;
        linear_acceleration = filtered_acc;
        
        prev_acc = linear_acceleration;
    }

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::msg::Imu::ConstSharedPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::msg::Imu::ConstSharedPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());

}

std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>, sensor_msgs::msg::PointCloud::ConstSharedPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>, sensor_msgs::msg::PointCloud::ConstSharedPtr>> measurements;

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;

        if (!(stamp_to_sec(imu_buf.back()->header.stamp) > stamp_to_sec(feature_buf.front()->header.stamp) + estimator.td))
        {
            sum_of_wait++;
            return measurements;
        }

        if (!(stamp_to_sec(imu_buf.front()->header.stamp) < stamp_to_sec(feature_buf.front()->header.stamp) + estimator.td))
        {
            RCLCPP_WARN(rclcpp::get_logger("vins_estimator"), "throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        sensor_msgs::msg::PointCloud::ConstSharedPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> IMUs;
        while (stamp_to_sec(imu_buf.front()->header.stamp) < stamp_to_sec(img_msg->header.stamp) + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            RCLCPP_WARN(rclcpp::get_logger("vins_estimator"), "no imu between two image");
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
{
    imu_msg_count++;
    double t = stamp_to_sec(imu_msg->header.stamp);
    if (t <= last_imu_t)
    {
        RCLCPP_WARN(rclcpp::get_logger("vins_estimator"), "imu message in disorder!");
        return;
    }
    last_imu_t = t;
    if (imu_msg_count == 1 || imu_msg_count % 500 == 0)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("vins_estimator"),
            "[INPUT] imu #%d stamp=%.6f queue_imu=%zu queue_feat=%zu",
            imu_msg_count, t, imu_buf.size(), feature_buf.size());
    }

    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::msg::Header header = imu_msg->header;
        header.frame_id = "world";
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }
}


void feature_callback(const sensor_msgs::msg::PointCloud::ConstSharedPtr &feature_msg)
{
    feature_msg_count++;
    if (!first_feature_logged)
    {
        first_feature_logged = true;
        RCLCPP_INFO(
            rclcpp::get_logger("vins_estimator"),
            "[INPUT] first feature packet received: stamp=%.6f points=%zu",
            stamp_to_sec(feature_msg->header.stamp), feature_msg->points.size());
    }
    else if (feature_msg_count % 20 == 0)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("vins_estimator"),
            "[INPUT] feature packet #%d stamp=%.6f points=%zu",
            feature_msg_count, stamp_to_sec(feature_msg->header.stamp), feature_msg->points.size());
    }
    if (!init_feature)
    {
        init_feature = 1;
        RCLCPP_INFO(rclcpp::get_logger("vins_estimator"), "[STATE] estimator primed on first feature packet");
        return;
    }
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

void restart_callback(const std_msgs::msg::Bool::ConstSharedPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        RCLCPP_WARN(rclcpp::get_logger("vins_estimator"), "restart the estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}

void relocalization_callback(const sensor_msgs::msg::PointCloud::ConstSharedPtr &points_msg)
{
    m_buf.lock();
    relo_buf.push(points_msg);
    m_buf.unlock();
}

void mavlink_velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr &vel_msg)
{
    m_odom.lock();
    mavlink_vz = vel_msg->twist.linear.z;
    
    static double last_vel_time = -1;
    double current_vel_time = stamp_to_sec(vel_msg->header.stamp);
    
    if (last_vel_time > 0) {
        double dt = current_vel_time - last_vel_time;
        if (dt > 0 && dt < 0.5) {
            mavlink_altitude += mavlink_vz * dt;
        }
    }
    last_vel_time = current_vel_time;
    
    mavlink_odom_received = true;
    m_odom.unlock();
}

void lidar_odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
{
    LidarOdomSample sample;
    sample.time = stamp_to_sec(odom_msg->header.stamp);
    sample.pos = Eigen::Vector3d(
        odom_msg->pose.pose.position.x,
        odom_msg->pose.pose.position.y,
        odom_msg->pose.pose.position.z);
    Eigen::Quaterniond q(
        odom_msg->pose.pose.orientation.w,
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z);
    sample.rot = q.toRotationMatrix();

    m_lidar.lock();
    lidar_odom_buf.push(sample);
    // Keep buffer bounded — store at most 200 samples (~20 s at 10 Hz)
    while (lidar_odom_buf.size() > 200)
        lidar_odom_buf.pop();
    if (lidar_first_recv_time < 0)
        lidar_first_recv_time = sample.time;
    lidar_odom_received = true;
    m_lidar.unlock();
}

// Find the lidar odometry sample closest to the given timestamp.
// Returns true if a sample within LIDAR_SYNC_MAX_DT was found and removes older samples.
bool getLidarOdomAtTime(double query_time, LidarOdomSample &out)
{
    std::lock_guard<std::mutex> lk(m_lidar);
    if (lidar_odom_buf.empty())
        return false;

    // Drop samples that are older than the query time minus the sync window
    while (!lidar_odom_buf.empty() && lidar_odom_buf.front().time < query_time - LIDAR_SYNC_MAX_DT)
        lidar_odom_buf.pop();

    if (lidar_odom_buf.empty())
        return false;

    // Find the closest sample in the remaining buffer
    LidarOdomSample best = lidar_odom_buf.front();
    double best_dt = std::abs(best.time - query_time);
    std::queue<LidarOdomSample> tmp = lidar_odom_buf;
    while (!tmp.empty())
    {
        double dt = std::abs(tmp.front().time - query_time);
        if (dt < best_dt)
        {
            best = tmp.front();
            best_dt = dt;
        }
        tmp.pop();
    }

    if (best_dt > LIDAR_SYNC_MAX_DT)
        return false;

    out = best;
    return true;
}

// thread: visual-inertial odometry
void process()
{
    while (true)
    {
        std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>, sensor_msgs::msg::PointCloud::ConstSharedPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;
                 });
        lk.unlock();
        m_estimator.lock();
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first)
            {
                double t = stamp_to_sec(imu_msg->header.stamp);
                double img_t = stamp_to_sec(img_msg->header.stamp) + estimator.td;
                if (t <= img_t)
                { 
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    assert(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    
                    // Apply accelerometer filtering
                    Eigen::Vector3d raw_acc(dx, dy, dz);
                    if (!acc_initialized) {
                        prev_acc = raw_acc;
                        filtered_acc = raw_acc;
                        acc_initialized = true;
                    } else {
                        Eigen::Vector3d acc_diff = raw_acc - prev_acc;
                        if (acc_diff.norm() > MAX_ACC_CHANGE) {
                            RCLCPP_WARN(rclcpp::get_logger("vins_estimator"), "IMU spike in process(): %.2f m/s², using filtered", acc_diff.norm());
                            raw_acc = filtered_acc;
                        }
                        filtered_acc = ACC_FILTER_ALPHA * filtered_acc + (1.0 - ACC_FILTER_ALPHA) * raw_acc;
                        raw_acc = filtered_acc;
                        prev_acc = raw_acc;
                    }
                    dx = raw_acc.x();
                    dy = raw_acc.y();
                    dz = raw_acc.z();
                    
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));

                }
                else
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    
                    if (dt_1 < 0)
                    {
                        RCLCPP_WARN(rclcpp::get_logger("vins_estimator"), "Negative dt_1 detected: %.6f (img_t=%.6f, current_time=%.6f). Skipping frame.",
                                 dt_1, img_t, current_time);
                        current_time = img_t;
                        continue;
                    }
                    
                    current_time = img_t;
                    assert(dt_2 >= 0);
                    assert(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                }
            }
            // set relocalization frame
            sensor_msgs::msg::PointCloud::ConstSharedPtr relo_msg = NULL;
            while (!relo_buf.empty())
            {
                relo_msg = relo_buf.front();
                relo_buf.pop();
            }
            if (relo_msg != NULL)
            {
                vector<Vector3d> match_points;
                double frame_stamp = stamp_to_sec(relo_msg->header.stamp);
                for (unsigned int i = 0; i < relo_msg->points.size(); i++)
                {
                    Vector3d u_v_id;
                    u_v_id.x() = relo_msg->points[i].x;
                    u_v_id.y() = relo_msg->points[i].y;
                    u_v_id.z() = relo_msg->points[i].z;
                    match_points.push_back(u_v_id);
                }
                Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg->channels[0].values[7];
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
            }

            RCLCPP_DEBUG(rclcpp::get_logger("vins_estimator"), "processing vision data with stamp %f \n", stamp_to_sec(img_msg->header.stamp));

            TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];
                double velocity_y = img_msg->channels[4].values[i];
                assert(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            }
            estimator.processImage(image, img_msg->header);
            
            // Apply altitude-based scale correction using MAVLink odometry
            m_odom.lock();
            if (mavlink_odom_received && estimator.solver_flag == Estimator::NON_LINEAR)
            {
                estimator.correctScaleWithAltitude(mavlink_altitude, mavlink_vz);
            }
            m_odom.unlock();

            // Lidar odometry: initial pose/scale alignment + ongoing scale correction
            if (ENABLE_LIDAR_INIT && lidar_odom_received && estimator.solver_flag == Estimator::NON_LINEAR)
            {
                double vins_time = stamp_to_sec(img_msg->header.stamp);
                LidarOdomSample lidar_sample;
                if (getLidarOdomAtTime(vins_time, lidar_sample))
                {
                    if (!estimator.getLidarInitStatus())
                    {
                        // One-time initial pose & coordinate alignment
                        estimator.initializeWithLidarOdom(
                            lidar_sample.pos, lidar_sample.rot,
                            lidar_sample.time, vins_time);
                    }
                    else
                    {
                        // Ongoing gentle scale correction against lidar reference
                        estimator.correctScaleWithLidarOdom(
                            lidar_sample.pos, lidar_sample.time, vins_time);
                    }
                }
            }

            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t);
            std_msgs::msg::Header header = img_msg->header;
            header.frame_id = WORLD_FRAME_ID;

            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);
            if (relo_msg != NULL)
                pubRelocalization(estimator);
        }
        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("vins_estimator");
    readParameters(node);
    estimator.setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
    RCLCPP_DEBUG(node->get_logger(), "EIGEN_DONT_PARALLELIZE");
#endif
    RCLCPP_WARN(node->get_logger(), "waiting for image and imu...");
    RCLCPP_INFO(node->get_logger(), "[START] vins_estimator is up and waiting for synchronized measurement pairs");

    registerPub(node);

    // QoS for sensor data (best effort replaces tcpNoDelay)
    rclcpp::QoS sensor_qos(2000);
    sensor_qos.best_effort();

    auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(
        IMU_TOPIC, sensor_qos, imu_callback);
    auto sub_image = node->create_subscription<sensor_msgs::msg::PointCloud>(
        "/feature_tracker/feature", 2000, feature_callback);
    auto sub_restart = node->create_subscription<std_msgs::msg::Bool>(
        "/feature_tracker/restart", 2000, restart_callback);
    auto sub_relo_points = node->create_subscription<sensor_msgs::msg::PointCloud>(
        "/pose_graph/match_points", 2000, relocalization_callback);
    auto sub_mavlink_vel = node->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/mavros/local_position/velocity_local", 100, mavlink_velocity_callback);
    auto sub_lidar_odom = node->create_subscription<nav_msgs::msg::Odometry>(
        LIDAR_ODOM_TOPIC, rclcpp::QoS(200).best_effort(), lidar_odom_callback);
    RCLCPP_INFO(node->get_logger(), "[START] subscribed IMU=%s | feature=/feature_tracker/feature | relo=/pose_graph/match_points | lidar=%s",
                IMU_TOPIC.c_str(), LIDAR_ODOM_TOPIC.c_str());

    std::thread measurement_process{process};
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
