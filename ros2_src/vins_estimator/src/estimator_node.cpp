#include "estimator_node.hpp"
using namespace std::placeholders;
Estimator estimator;

EstimatorNode::EstimatorNode() : Node("estimator_node")
{
    getParams();
    estimator.setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
    RCLCPP_DEBUG(this->get_logger(), "EIGEN_DONT_PARALLELIZE");
#endif
    RCLCPP_WARN(this->get_logger(), "waiting for image and imu...");
    initTopic();
    registerPub();

    measurement_process = std::thread(&EstimatorNode::process, this);
}

void EstimatorNode::predict(const imuMsg::SharedPtr imu_msg)
{
    double t = toSec(imu_msg->header);
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

void EstimatorNode::update()
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

    queue<imuMsg::SharedPtr> tmp_imu_buf = imu_buf;
    for (imuMsg::SharedPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());
}

std::vector<std::pair<std::vector<imuMsg::SharedPtr>, pointCloudMsg::SharedPtr>> EstimatorNode::getMeasurements()
{
    std::vector<std::pair<std::vector<imuMsg::SharedPtr>, pointCloudMsg::SharedPtr>> measurements;
    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())
        {
            return measurements;
        }
 
        if (toSec(imu_buf.back()->header) <= (toSec(feature_buf.front()->header) + (estimator.td / 1.0)))
        {
            RCLCPP_WARN(this->get_logger(),"Wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }

        if (toSec(imu_buf.front()->header) >= (toSec(feature_buf.front()->header) + (estimator.td / 1.0)))
        {
            RCLCPP_WARN(this->get_logger(), "throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        pointCloudMsg::SharedPtr img_msg = feature_buf.front();
        feature_buf.pop();
        std::vector<imuMsg::SharedPtr> IMUs;
        while (toSec(imu_buf.front()->header) < toSec(img_msg->header) + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            RCLCPP_WARN(this->get_logger(), "no imu between two image");

        measurements.emplace_back(IMUs, img_msg);
    }
        RCLCPP_INFO(rclcpp::get_logger(""), "1");
        return measurements;
}

void EstimatorNode::imu_callback(const imuMsg::SharedPtr imu_msg)
{
    // RCLCPP_INFO(this->get_logger(), "IMU Callback: %f", toSec(imu_msg->header));
    imu_timer_ = toSec(imu_msg->header);

    if (imu_timer_ <= last_imu_t)
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "imu message in disorder!");
        return;
    }
    last_imu_t = imu_timer_;

    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    last_imu_t = imu_timer_;
    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::msg::Header header = imu_msg->header;
        header.frame_id = "world";

        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }
}

void EstimatorNode::feature_callback(const pointCloudMsg::SharedPtr feature_msg)
{
    if (!init_feature)
    {        
        // skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

void EstimatorNode::restart_callback(const boolMsg::SharedPtr restart_msg)
{
    if (restart_msg->data == false) {  return; }
    
    RCLCPP_WARN(this->get_logger(), "restart the estimator!");
    m_buf.lock();
    while (!feature_buf.empty())
        feature_buf.pop();
    while (!imu_buf.empty())
        imu_buf.pop();
    m_buf.unlock();
    m_estimator.lock();
    estimator.clearState();
    estimator.setParameter();
    m_estimator.unlock();
    current_time = -1;
    last_imu_t = 0;
}

void EstimatorNode::relocalization_callback(const pointCloudMsg::SharedPtr points_msg)
{
    RCLCPP_INFO(this->get_logger(), "Relocalization callback!");
    m_buf.lock();
    relo_buf.push(points_msg);
    m_buf.unlock();
}

// thread: visual-inertial odometry
void EstimatorNode::process()
{
    while (true)
    {
        std::vector<std::pair<std::vector<imuMsg::SharedPtr>,
                                        pointCloudMsg::SharedPtr>>  measurements;

        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&] { 
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
                double t = toSec(imu_msg->header);
                double img_t = toSec(img_msg->header) + estimator.td;
                if (t <= img_t)
                {
                    if (current_time < 0)
                    {
                        current_time = t;
                    }

                    double dt = t - current_time;
                    assert(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    // printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
                }
                else
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    assert(dt_1 >= 0);
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
                    // printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }
            // set relocalization frame
            pointCloudMsg::SharedPtr relo_msg = NULL;
            while (!relo_buf.empty())
            {
                relo_msg = relo_buf.front();
                relo_buf.pop();
            }
            if (relo_msg != NULL)
            {
                vector<Vector3d> match_points;
                double frame_stamp = toSec(relo_msg->header);
                for (unsigned int i = 0; i < relo_msg->points.size(); i++)
                {
                    Vector3d u_v_id;
                    u_v_id.x() = relo_msg->points[i].x;
                    u_v_id.y() = relo_msg->points[i].y;
                    u_v_id.z() = relo_msg->points[i].z;
                    match_points.push_back(u_v_id);
                }
                Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], 
                                                                        relo_msg->channels[0].values[2]);
                Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], 
                                            relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg->channels[0].values[7];
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
            }

            RCLCPP_DEBUG(this->get_logger(), "processing vision data with stamp %f \n", toSec(img_msg->header));

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
                image[feature_id].emplace_back(camera_id, xyz_uv_velocity);
            }
            estimator.processImage(image, img_msg->header);

            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t);
            std_msgs::msg::Header header = img_msg->header;
            header.frame_id = "world";
            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);
            if (relo_msg != NULL)
                pubRelocalization(estimator);
            // printf("end: %d, at %d", toSec(img_msg->header), rclcpp::Node::now());
        }
        // RCLCPP_INFO(this->get_logger(), "UPDATE");
        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();

        m_state.unlock();
        m_buf.unlock();
    }
}

void EstimatorNode::initTopic()
{
    sub.imu = this->create_subscription<imuMsg>(IMU_TOPIC, 200,
                                                std::bind(&EstimatorNode::imu_callback, this, _1));
    sub.image = this->create_subscription<pointCloudMsg>("/feature_tracker/feature", 200,
                                                         std::bind(&EstimatorNode::feature_callback, this, _1));
    sub.restart = this->create_subscription<boolMsg>("/feature_tracker/restart", 200,
                                                     std::bind(&EstimatorNode::restart_callback, this, _1));
    sub.relo_points = this->create_subscription<pointCloudMsg>("/pose_graph/match_points", 200,
                                                               std::bind(&EstimatorNode::relocalization_callback, this, _1));

    pub_path = this->create_publisher<navPathMsg>("vins_estimator/path", 100);
    pub_key_poses = this->create_publisher<markerMsg>("vins_estimator/key_poses", 100);
    pub_odometry = this->create_publisher<navOdometryMsg>("vins_estimator/odometry", 100);
    pub_extrinsic = this->create_publisher<navOdometryMsg>("vins_estimator/extrinsic", 100);
    pub_point_cloud = this->create_publisher<pointCloudMsg>("vins_estimator/point_cloud", 100);
    pub_camera_pose = this->create_publisher<navOdometryMsg>("vins_estimator/camera_pose", 100);
    pub_margin_cloud = this->create_publisher<pointCloudMsg>("vins_estimator/history_cloud", 100);
    pub_relo_path = this->create_publisher<navPathMsg>("vins_estimator/relocalization_path", 100);
    pub_keyframe_pose = this->create_publisher<navOdometryMsg>("vins_estimator/keyframe_pose", 100);
    pub_keyframe_point = this->create_publisher<pointCloudMsg>("vins_estimator/keyframe_point", 100);
    pub_latest_odometry = this->create_publisher<navOdometryMsg>("vins_estimator/imu_propagate", 100);
    pub_camera_pose_visual = this->create_publisher<markerArrayMsg>("vins_estimator/camera_pose_visual", 100);
    pub_relo_relative_pose = this->create_publisher<navOdometryMsg>("vins_estimator/relo_relative_pose", 100);
}

void EstimatorNode::getParams()
{
    this->declare_parameter<std::string>("config_file", "/home/serkan/source_code/VINS-Mono/ros2_src/config/config/euroc/euroc_config.yaml");
    std::string config_file = this->get_parameter("config_file").as_string();
    readParameters(config_file);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EstimatorNode>());
    rclcpp::shutdown();
    return 0;
}