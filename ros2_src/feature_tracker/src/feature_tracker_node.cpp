#include "feature_tracker_node.hpp"

FeatureTrackerNode::FeatureTrackerNode(): Node("feature_tracker_node"){
    getParams();
    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++){
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
                RCLCPP_ERROR(this->get_logger(), "load mask fail");
            else
                RCLCPP_ERROR(this->get_logger(), "load mask success");
        }
    }
    
    initTopic();
}

void FeatureTrackerNode::initTopic(){
    pub_img     = this->create_publisher<pointCloudMsg>("feature", 2000);
    pub_match   = this->create_publisher<imageMsg>("feature_img", 2000);
    pub_restart = this->create_publisher<boolMsg>("restart", 2000);
    sub_img     = this->create_subscription<imageMsg>(IMAGE_TOPIC, 2000, std::bind(
                        &FeatureTrackerNode::imgCallback, this, std::placeholders::_1));
}

void FeatureTrackerNode::imgCallback(const imageMsg::SharedPtr img_msg){
    current_time = static_cast<double>(img_msg->header.stamp.sec) + static_cast<double>(img_msg->header.stamp.nanosec / 1.0e9);
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = current_time;
        last_image_time = current_time;
        return;
    }
    // detect unstable camera stream
    if (current_time - last_image_time > 1.0 || current_time < last_image_time) {
        RCLCPP_WARN(this->get_logger(), "image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        std_msgs::msg::Bool restart_flag;
        restart_flag.data = true;
        pub_restart->publish(restart_flag);
        return;
    }
    last_image_time = current_time;
    // frequency control
    // RCLCPP_INFO(this->get_logger(), COLOR_BLE"now: %lf prev: %lf",current_time, first_image_time);
    // RCLCPP_INFO(this->get_logger(), "now freq: %lf freq: %d",(1.0 * pub_count /(current_time - first_image_time)), FREQ);
    if (1.0 * pub_count /(current_time - first_image_time) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (current_time - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = current_time;
            pub_count = 0;
        }
    }
    else{
        PUB_THIS_FRAME = false;
    }
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::msg::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat show_img = ptr->image;
    TicToc t_r;
    for (int i = 0; i < NUM_OF_CAM; i++) {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "processing camera " << i );
        if (i != 1 || !STEREO_TRACK)
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), img_msg->header.stamp.sec);
        else
        {
            if (EQUALIZE) {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
        }

#if SHOW_UNDISTORTION
        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
    }

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }

   if (PUB_THIS_FRAME) {
        pub_count++;
        sensor_msgs::msg::PointCloud::SharedPtr feature_points(new sensor_msgs::msg::PointCloud);
        sensor_msgs::msg::ChannelFloat32 id_of_point;
        sensor_msgs::msg::ChannelFloat32 u_of_point;
        sensor_msgs::msg::ChannelFloat32 v_of_point;
        sensor_msgs::msg::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::msg::ChannelFloat32 velocity_y_of_point;

        feature_points->header = img_msg->header;
        feature_points->header.frame_id = "world";

        std::vector<std::set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::msg::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }
        }
        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);
        RCLCPP_DEBUG_STREAM(this->get_logger(), "publish " << feature_points->header.stamp.sec << " at " << (this->now().seconds()));
        // skip the first image; since no optical speed on frist image
        if (!init_pub){
            init_pub = 1;
        }
        else
            pub_img->publish(*feature_points);

        if (SHOW_TRACK)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                    //draw speed line
                    /*
                    Vector2d tmp_cur_un_pts (trackerData[i].cur_un_pts[j].x, trackerData[i].cur_un_pts[j].y);
                    Vector2d tmp_pts_velocity (trackerData[i].pts_velocity[j].x, trackerData[i].pts_velocity[j].y);
                    Vector3d tmp_prev_un_pts;
                    tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                    tmp_prev_un_pts.z() = 1;
                    Vector2d tmp_prev_uv;
                    trackerData[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                    cv::line(tmp_img, trackerData[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
                    */
                    //char name[10];
                    //sprintf(name, "%d", trackerData[i].ids[j]);
                    //cv::putText(tmp_img, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
                }
            }
            // cv::imshow("vis", stereo_img);
            // cv::waitKey(5);
            pub_match->publish(*ptr->toImageMsg());
        }
    }
    RCLCPP_DEBUG(this->get_logger(), "whole feature tracker processing costs: %f", t_r.toc());
}

void FeatureTrackerNode::getParams(){
    this->declare_parameter<std::string>("config_file", "/home/serkan/source_code/VINS-Mono/ros2_src/config/config/euroc/euroc_config.yaml");
    this->declare_parameter<std::string>("vins_folder", "/home/serkan/source_code/VINS-Mono/ros2_src/config");

    std::string config_file = this->get_parameter("config_file").as_string();
    std::string VINS_FOLDER_PATH = this->get_parameter("vins_folder").as_string();
    readParameters(config_file, VINS_FOLDER_PATH);
}   

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FeatureTrackerNode>());
    rclcpp::shutdown();

}