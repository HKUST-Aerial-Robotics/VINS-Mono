#include "visualization.h"

using namespace ros;
using namespace Eigen;
ros::Publisher pub_odometry, pub_latest_odometry;
ros::Publisher pub_path, pub_loop_path;
ros::Publisher pub_point_cloud, pub_margin_cloud;
ros::Publisher pub_key_poses;

ros::Publisher pub_camera_pose;
ros::Publisher pub_camera_pose_visual, pub_pose_graph;
nav_msgs::Path path, loop_path;
CameraPoseVisualization cameraposevisual(0, 0, 1, 1);
CameraPoseVisualization keyframebasevisual(0.0, 0.0, 1.0, 1.0);
static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);

void registerPub(ros::NodeHandle &n)
{
    pub_latest_odometry = n.advertise<nav_msgs::Odometry>("imu_propagate", 1000);
    pub_path = n.advertise<nav_msgs::Path>("path_no_loop", 1000);
    pub_loop_path = n.advertise<nav_msgs::Path>("path", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud", 1000);
    pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>("history_cloud", 1000);
    pub_key_poses = n.advertise<visualization_msgs::Marker>("key_poses", 1000);
    pub_camera_pose = n.advertise<geometry_msgs::PoseStamped>("camera_pose", 1000);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
    pub_pose_graph = n.advertise<visualization_msgs::MarkerArray>("pose_graph", 1000);

    cameraposevisual.setScale(0.2);
    cameraposevisual.setLineWidth(0.01);
    keyframebasevisual.setScale(0.1);
    keyframebasevisual.setLineWidth(0.01);
}

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, const std_msgs::Header &header)
{
    Eigen::Quaterniond quadrotor_Q = Q ;

    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = quadrotor_Q.x();
    odometry.pose.pose.orientation.y = quadrotor_Q.y();
    odometry.pose.pose.orientation.z = quadrotor_Q.z();
    odometry.pose.pose.orientation.w = quadrotor_Q.w();
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    pub_latest_odometry.publish(odometry);
}

void printStatistics(const Estimator &estimator, double t)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    ROS_INFO_STREAM("position: " << estimator.Ps[WINDOW_SIZE].transpose());
    ROS_DEBUG_STREAM("orientation: " << estimator.Vs[WINDOW_SIZE].transpose());
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        //ROS_DEBUG("calibration result for camera %d", i);
        ROS_DEBUG_STREAM("extirnsic tic: " << estimator.tic[i].transpose());
        ROS_DEBUG_STREAM("extrinsic ric: " << Utility::R2ypr(estimator.ric[i]).transpose());
        if (ESTIMATE_EXTRINSIC)
        {
            cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
            Eigen::Matrix3d eigen_R;
            Eigen::Vector3d eigen_T;
            eigen_R = estimator.ric[i];
            eigen_T = estimator.tic[i];
            cv::Mat cv_R, cv_T;
            cv::eigen2cv(eigen_R, cv_R);
            cv::eigen2cv(eigen_T, cv_T);
            fs << "extrinsicRotation" << cv_R << "extrinsicTranslation" << cv_T;
            fs.release();
        }
    }

    static double sum_of_time = 0;
    static int sum_of_calculation = 0;
    sum_of_time += t;
    sum_of_calculation++;
    ROS_DEBUG("vo solver costs: %f ms", t);
    ROS_DEBUG("average of time %f ms", sum_of_time / sum_of_calculation);

    sum_of_path += (estimator.Ps[WINDOW_SIZE] - last_path).norm();
    last_path = estimator.Ps[WINDOW_SIZE];
    ROS_DEBUG("sum of path %f", sum_of_path);
}

void pubOdometry(const Estimator &estimator, const std_msgs::Header &header, Eigen::Vector3d loop_correct_t,
                Eigen::Matrix3d loop_correct_r)
{
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.child_frame_id = "world";
        odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
        odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
        odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
        odometry.pose.pose.orientation.x = Quaterniond(estimator.Rs[WINDOW_SIZE]).x();
        odometry.pose.pose.orientation.y = Quaterniond(estimator.Rs[WINDOW_SIZE]).y();
        odometry.pose.pose.orientation.z = Quaterniond(estimator.Rs[WINDOW_SIZE]).z();
        odometry.pose.pose.orientation.w = Quaterniond(estimator.Rs[WINDOW_SIZE]).w();
        
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose = odometry.pose.pose;
        path.header = header;
        path.header.frame_id = "world";
        path.poses.push_back(pose_stamped);
        pub_path.publish(path);

        Vector3d correct_t;
        Vector3d correct_v;
        Quaterniond correct_q;
        correct_t = loop_correct_r * estimator.Ps[WINDOW_SIZE] + loop_correct_t;
        correct_q = loop_correct_r * estimator.Rs[WINDOW_SIZE];
        correct_v = loop_correct_r * estimator.Vs[WINDOW_SIZE];
        odometry.pose.pose.position.x = correct_t.x();
        odometry.pose.pose.position.y = correct_t.y();
        odometry.pose.pose.position.z = correct_t.z();
        odometry.pose.pose.orientation.x = correct_q.x();
        odometry.pose.pose.orientation.y = correct_q.y();
        odometry.pose.pose.orientation.z = correct_q.z();
        odometry.pose.pose.orientation.w = correct_q.w();
        odometry.twist.twist.linear.x = correct_v(0);
        odometry.twist.twist.linear.y = correct_v(1);
        odometry.twist.twist.linear.z = correct_v(2);
        pub_odometry.publish(odometry);

        pose_stamped.pose = odometry.pose.pose;
        loop_path.header = header;
        loop_path.header.frame_id = "world";
        loop_path.poses.push_back(pose_stamped);
        pub_loop_path.publish(loop_path);

        // write result to file
        ofstream foutC(VINS_RESULT_PATH, ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(0);
        foutC << header.stamp.toSec() * 1e9 << ",";
        foutC.precision(5);
        foutC << correct_t.x() << ","
              << correct_t.y() << ","
              << correct_t.z() << ","
              << correct_q.w() << ","
              << correct_q.x() << ","
              << correct_q.y() << ","
              << correct_q.z() << ","
              << correct_v(0) << ","
              << correct_v(1) << ","
              << correct_v(2) << "," << endl;
        foutC.close();
    }
}

void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header, Eigen::Vector3d loop_correct_t,
                Eigen::Matrix3d loop_correct_r)
{
    if (estimator.key_poses.size() == 0)
        return;
    visualization_msgs::Marker key_poses;
    key_poses.header = header;
    key_poses.header.frame_id = "world";
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::Marker::SPHERE_LIST;
    key_poses.action = visualization_msgs::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    key_poses.id = 0; //key_poses_id++;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        geometry_msgs::Point pose_marker;
        Vector3d correct_pose;
        correct_pose = loop_correct_r * estimator.key_poses[i] + loop_correct_t;
        pose_marker.x = correct_pose.x();
        pose_marker.y = correct_pose.y();
        pose_marker.z = correct_pose.z();
        key_poses.points.push_back(pose_marker);
    }
    pub_key_poses.publish(key_poses);
}

void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header, Eigen::Vector3d loop_correct_t,
                   Eigen::Matrix3d loop_correct_r)
{
    int idx2 = WINDOW_SIZE - 1;
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        int i = idx2;
        geometry_msgs::PoseStamped camera_pose;
        camera_pose.header = header;
        camera_pose.header.frame_id = std::to_string(estimator.Headers[i].stamp.toNSec());
        Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[0]);
        P = (loop_correct_r * estimator.Ps[i] + loop_correct_t) + (loop_correct_r * estimator.Rs[i]) * estimator.tic[0];
        R = Quaterniond((loop_correct_r * estimator.Rs[i]) * estimator.ric[0]);
        camera_pose.pose.position.x = P.x();
        camera_pose.pose.position.y = P.y();
        camera_pose.pose.position.z = P.z();
        camera_pose.pose.orientation.w = R.w();
        camera_pose.pose.orientation.x = R.x();
        camera_pose.pose.orientation.y = R.y();
        camera_pose.pose.orientation.z = R.z();

        pub_camera_pose.publish(camera_pose);

        cameraposevisual.reset();
        cameraposevisual.add_pose(P, R);
        camera_pose.header.frame_id = "world";
        cameraposevisual.publish_by(pub_camera_pose_visual, camera_pose.header);
    }
}


void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header, Eigen::Vector3d loop_correct_t,
                   Eigen::Matrix3d loop_correct_r)
{
    sensor_msgs::PointCloud point_cloud, loop_point_cloud;
    point_cloud.header = header;
    loop_point_cloud.header = header;

    for (auto &it_per_id : estimator.f_manager.feature)
    {
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
            continue;
        int imu_i = it_per_id.start_frame;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Vector3d w_pts_i = loop_correct_r * estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0])
                              + loop_correct_r * estimator.Ps[imu_i] + loop_correct_t;

        geometry_msgs::Point32 p;
        p.x = w_pts_i(0);
        p.y = w_pts_i(1);
        p.z = w_pts_i(2);
        point_cloud.points.push_back(p);
    }
    pub_point_cloud.publish(point_cloud);


    // pub margined potin
    sensor_msgs::PointCloud margin_cloud, loop_margin_cloud;
    margin_cloud.header = header;
    loop_margin_cloud.header = header;

    for (auto &it_per_id : estimator.f_manager.feature)
    { 
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        //if (it_per_id->start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id->solve_flag != 1)
        //        continue;

        if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2 
            && it_per_id.solve_flag == 1 )
        {
            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d w_pts_i = loop_correct_r * estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0])
                             + loop_correct_r * estimator.Ps[imu_i] + loop_correct_t;

            geometry_msgs::Point32 p;
            p.x = w_pts_i(0);
            p.y = w_pts_i(1);
            p.z = w_pts_i(2);
            margin_cloud.points.push_back(p);
        }
    }
    pub_margin_cloud.publish(margin_cloud);
}

void pubPoseGraph(CameraPoseVisualization* posegraph, const std_msgs::Header &header)
{
    posegraph->publish_by(pub_pose_graph, header);
    
}

void updateLoopPath(nav_msgs::Path _loop_path)
{
    loop_path = _loop_path;
}

void pubTF(const Estimator &estimator, const std_msgs::Header &header, Eigen::Vector3d loop_correct_t,
                   Eigen::Matrix3d loop_correct_r)
{
    if( estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    // body frame
    Vector3d correct_t;
    Quaterniond correct_q;
    correct_t = loop_correct_r * estimator.Ps[WINDOW_SIZE] + loop_correct_t;
    correct_q = loop_correct_r * estimator.Rs[WINDOW_SIZE];

    transform.setOrigin(tf::Vector3(correct_t(0),
                                    correct_t(1),
                                    correct_t(2)));
    q.setW(correct_q.w());
    q.setX(correct_q.x());
    q.setY(correct_q.y());
    q.setZ(correct_q.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "world", "body"));

    // camera frame
    transform.setOrigin(tf::Vector3(estimator.tic[0].x(),
                                    estimator.tic[0].y(),
                                    estimator.tic[0].z()));
    q.setW(Quaterniond(estimator.ric[0]).w());
    q.setX(Quaterniond(estimator.ric[0]).x());
    q.setY(Quaterniond(estimator.ric[0]).y());
    q.setZ(Quaterniond(estimator.ric[0]).z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "body", "camera"));
}