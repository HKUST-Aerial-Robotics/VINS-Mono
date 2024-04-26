#include "pose_graph_node.hpp"

using namespace std::placeholders;

camodocal::CameraPtr m_camera;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match_img;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_match_points;

int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
int ROW;
int COL;
std::string VINS_RESULT_PATH;
string vocabulary_file;
int DEBUG_IMAGE;
int FAST_RELOCALIZATION;

CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
Eigen::Vector3d last_t(-100, -100, -100);
PoseGraph posegraph;

PoseGraphNode::PoseGraphNode(): Node("pose_graph_node"){
    getParams();
    initTopic();

    measurement_process = std::thread(&PoseGraphNode::process, this);
    keyboard_command_process = std::thread(&PoseGraphNode::command, this);
}

void PoseGraphNode::newSequence(){
    RCLCPP_INFO(this->get_logger(), "new sequence\n");
    sequence++;
    RCLCPP_INFO(this->get_logger(), "sequence cnt %d \n", sequence);
    if (sequence > 5)
    {
        RCLCPP_WARN(this->get_logger(), "only support 5 sequences since it's boring to copy code for more sequences.");
        rclcpp::shutdown();
        exit(-1);
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

void PoseGraphNode::imageCallback(const imageMsg::SharedPtr image_msg){
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    image_buf.push(image_msg);
    m_buf.unlock();
    //printf(" image time %f \n", image_msg->header.stamp.toSec());

    // detect unstable camera stream
    if (last_image_time == -1)
        last_image_time = toSec(image_msg->header);
    else if (toSec(image_msg->header) - last_image_time > 1.0 || toSec(image_msg->header) < last_image_time)
    {
        RCLCPP_WARN(this->get_logger(), "image discontinue! detect a new sequence!");
        newSequence();
    }
    last_image_time = toSec(image_msg->header);
}

void PoseGraphNode::pointCallback(const pointCloudMsg::SharedPtr point_msg){
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    point_buf.push(point_msg);
    m_buf.unlock();
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

void PoseGraphNode::poseCallback(const odometryMsg::SharedPtr pose_msg){
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();
}

void PoseGraphNode::imuForwardCallback(const odometryMsg::SharedPtr forward_msg){
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

void PoseGraphNode::reloRelativePoseCallback(const odometryMsg::SharedPtr pose_msg){
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

void PoseGraphNode::vioCallback(const odometryMsg::SharedPtr pose_msg){
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
    key_odometrys.lifetime = rclcpp::Duration(0, 0);

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

void PoseGraphNode::extrinsicCallback(const odometryMsg::SharedPtr pose_msg){
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

void PoseGraphNode::process(){
    if (!LOOP_CLOSURE)
        return;
    while (true)
    {
        sensor_msgs::msg::Image::SharedPtr image_msg = NULL;
        sensor_msgs::msg::PointCloud::SharedPtr point_msg = NULL;
        nav_msgs::msg::Odometry::SharedPtr pose_msg = NULL;

        // find out the messages with same time stamp
        m_buf.lock();
        if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            if (toSec(image_buf.front()->header) > toSec(pose_buf.front()->header))
            {
                pose_buf.pop();
                printf("throw pose at beginning\n");
            }
            else if (toSec(image_buf.front()->header) > toSec(point_buf.front()->header))
            {
                point_buf.pop();
                printf("throw point at beginning\n");
            }
            else if (toSec(image_buf.back()->header) >= toSec(pose_buf.front()->header) 
                        && toSec(point_buf.back()->header) >= toSec(pose_buf.front()->header))
            {
                pose_msg = pose_buf.front();
                pose_buf.pop();
                while (!pose_buf.empty())
                    pose_buf.pop();
                while (toSec(image_buf.front()->header) < toSec(pose_msg->header))
                    image_buf.pop();
                image_msg = image_buf.front();
                image_buf.pop();

                while (toSec(point_buf.front()->header) < toSec(pose_msg->header))
                    point_buf.pop();
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

                for (unsigned int i = 0; i < point_msg->points.size(); i++)
                {
                    cv::Point3f p_3d;
                    p_3d.x = point_msg->points[i].x;
                    p_3d.y = point_msg->points[i].y;
                    p_3d.z = point_msg->points[i].z;
                    point_3d.push_back(p_3d);

                    cv::Point2f p_2d_uv, p_2d_normal;
                    double p_id;
                    p_2d_normal.x = point_msg->channels[i].values[0];
                    p_2d_normal.y = point_msg->channels[i].values[1];
                    p_2d_uv.x = point_msg->channels[i].values[2];
                    p_2d_uv.y = point_msg->channels[i].values[3];
                    p_id = point_msg->channels[i].values[4];
                    point_2d_normal.push_back(p_2d_normal);
                    point_2d_uv.push_back(p_2d_uv);
                    point_id.push_back(p_id);

                    //printf("u %f, v %f \n", p_2d_uv.x, p_2d_uv.y);
                }
                KeyFrame* keyframe = new KeyFrame(toSec(pose_msg->header), frame_index, T, R, image,
                                   point_3d, point_2d_uv, point_2d_normal, point_id, sequence);   
                m_process.lock();
                start_flag = 1;
                posegraph.addKeyFrame(keyframe, 1);
                m_process.unlock();
                frame_index++;
                last_t = T;
            }
        }

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

void PoseGraphNode::command(){
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
        }
        if (c == 'n')
            newSequence();

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

void PoseGraphNode::initTopic(){
    sub_imu_forward = this->create_subscription<odometryMsg>("/vins_estimator/imu_propagate", 1000, 
                                                std::bind(&PoseGraphNode::imuForwardCallback, this, _1));
    sub_vio = this->create_subscription<odometryMsg>("/vins_estimator/odometry", 1000, 
                                        std::bind(&PoseGraphNode::vioCallback, this, _1));
    sub_image = this->create_subscription<imageMsg>(IMAGE_TOPIC, 1000, std::bind(&PoseGraphNode::imageCallback, this, _1));
    sub_pose = this->create_subscription<odometryMsg>("/vins_estimator/keyframe_pose", 1000, 
                                        std::bind(&PoseGraphNode::poseCallback, this, _1));
    sub_extrinsic = this->create_subscription<odometryMsg>("/vins_estimator/extrinsic", 1000, 
                                                std::bind(&PoseGraphNode::extrinsicCallback, this , _1));
    sub_point = this->create_subscription<pointCloudMsg>("/vins_estimator/keyframe_point", 1000, 
                                                        std::bind(&PoseGraphNode::pointCallback, this, _1));
    sub_relo_relative_pose = this->create_subscription<odometryMsg>("/vins_estimator/relo_relative_pose", 1000, 
                                                                        std::bind(&PoseGraphNode::reloRelativePoseCallback, this, _1));

    pub_match_img = this->create_publisher<sensor_msgs::msg::Image>("/pose_graph/match_image", 500);
    pub_camera_pose_visual = this->create_publisher<visualization_msgs::msg::MarkerArray>("/pose_graph/camera_pose_visual", 500);
    pub_key_odometrys = this->create_publisher<visualization_msgs::msg::Marker>("/pose_graph/key_odometrys", 500);
    pub_vio_path = this->create_publisher<nav_msgs::msg::Path>("/pose_graph/no_loop_path", 500);
    pub_match_points = this->create_publisher<sensor_msgs::msg::PointCloud>("pose_graph/match_points", 500);

    posegraph.pub_pg_path = this->create_publisher<navPath>("/pose_graph/pose_graph_path", 500);
    posegraph.pub_base_path = this->create_publisher<navPath>("/pose_graph/base_path", 500);
    posegraph.pub_pose_graph = this->create_publisher<markerArrayMsg>("/pose_graph/pose_graph", 500);
    for (int i = 1; i < 10; i++)
        posegraph.pub_path[i] = this->create_publisher<navPath>("/pose_graph/path_" + to_string(i), 500);
}

void PoseGraphNode::getParams(){
    this->declare_parameter<std::string>("config_file", "/home/serkan/source_code/VINS-Mono/src/config/config/euroc/euroc_config.yaml");
    this->declare_parameter<int>("visualization_shift_x", 0);
    this->declare_parameter<int>("visualization_shift_y", 0);
    this->declare_parameter<int>("skip_cnt", 0);
    this->declare_parameter<double>("skip_dis", 0.0);
    
    VISUALIZATION_SHIFT_X = this->get_parameter("visualization_shift_x").as_int();
    VISUALIZATION_SHIFT_Y = this->get_parameter("visualization_shift_y").as_int();
    SKIP_CNT = this->get_parameter("skip_cnt").as_int();
    SKIP_DIS = this->get_parameter("skip_dis").as_double();
    config_file = this->get_parameter("config_file").as_string();

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    double camera_visual_size = fsSettings["visualize_camera_size"];
    cameraposevisual.setScale(camera_visual_size);
    cameraposevisual.setLineWidth(camera_visual_size / 10.0);

    LOOP_CLOSURE = fsSettings["loop_closure"];
    int LOAD_PREVIOUS_POSE_GRAPH;

    if (LOOP_CLOSURE){
        ROW = fsSettings["image_height"];
        COL = fsSettings["image_width"];
        std::string pkg_path = ament_index_cpp::get_package_share_directory("config");
        RCLCPP_INFO_STREAM(this->get_logger(), pkg_path);
        vocabulary_file = pkg_path + "/support_files/brief_k10L6.bin";
        posegraph.loadVocabulary(vocabulary_file);
        BRIEF_PATTERN_FILE = pkg_path + "/support_files/brief_pattern.yml";
        m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file.c_str());

        fsSettings["image_topic"] >> IMAGE_TOPIC;        
        fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
        fsSettings["output_path"] >> VINS_RESULT_PATH;
        fsSettings["save_image"] >> DEBUG_IMAGE;

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

    RCLCPP_INFO_STREAM(this->get_logger(), 
        COLOR_GRN 
        << "\n" << LINE
        << "\n- COL: " << COL
        << "\n- ROW: " << ROW
        << "\n- SKIP_CNT: " << SKIP_CNT
        << "\n- SKIP_DIS: " << SKIP_DIS
        << "\n- IMAGE_TOPIC: " << IMAGE_TOPIC
        << "\n- DEBUG_IMAGE: " << DEBUG_IMAGE
        << "\n- LOOP_CLOSURE: " << LOOP_CLOSURE
        << "\n- FAST_RELOCALIZATION: " << FAST_RELOCALIZATION
        << "\n- VISUALIZE_IMU_FORWARD: " << VISUALIZE_IMU_FORWARD
        << "\n- VISUALIZATION_SHIFT_X: " << VISUALIZATION_SHIFT_X
        << "\n- VISUALIZATION_SHIFT_Y: " << VISUALIZATION_SHIFT_Y 
        << "\n- LOAD_PREVIOUS_POSE_GRAPH: " << LOAD_PREVIOUS_POSE_GRAPH << std::endl
        << "\n- config_file: " << config_file
        << "\n- vocabulary_file: " << vocabulary_file
        << "\n- VINS_RESULT_PATH: " << VINS_RESULT_PATH
        << "\n- BRIEF_PATTERN_FILE: " << BRIEF_PATTERN_FILE
        << "\n- POSE_GRAPH_SAVE_PATH: " << POSE_GRAPH_SAVE_PATH
        << "\n" << LINE
        << COLOR_RST
    );

    fsSettings.release();
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<PoseGraphNode>());
    rclcpp::shutdown();
}