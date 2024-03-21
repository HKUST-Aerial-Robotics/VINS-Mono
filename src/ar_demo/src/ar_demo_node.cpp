#include "ar_demo_node.hpp"

ArDemo::ArDemo():Node("ar_demo"){
    object_pub = this->create_publisher<markerArrayMsg>("object", 10);
    pub_ARimage = this->create_publisher<sensor_msgs::msg::Image>("AR_image_topic", 10);
    // std::cout << now().nanoseconds() << std::endl;
}

ArDemo::~ArDemo(){

}

void ArDemo::axisGenerate(visualization_msgs::msg::Marker &line_list, Eigen::Vector3d &origin, int id){
    line_list.id = id;
    line_list.header.frame_id = "world";
    line_list.header.stamp = this->now();
    line_list.action = visualization_msgs::msg::Marker::ADD;
    line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
    line_list.scale.x = 0.1;
    line_list.color.a = 1.0;
    line_list.lifetime = rclcpp::Duration::from_seconds(0.3);
    line_list.pose.orientation.w = 1.0;
    line_list.color.b = 1.0;

    geometry_msgs::msg::Point p;
    p.x = origin.x();
    p.y = origin.y();
    p.z = origin.z();
    line_list.points.push_back(p);
    line_list.colors.push_back(line_color_r);
    p.x += 1.0;
    line_list.points.push_back(p);
    line_list.colors.push_back(line_color_r);
    p.x -= 1.0;
    line_list.points.push_back(p);
    line_list.colors.push_back(line_color_g);
    p.y += 1.0;
    line_list.points.push_back(p);
    line_list.colors.push_back(line_color_g);
    p.y -= 1.0;
    line_list.points.push_back(p);
    line_list.colors.push_back(line_color_b);
    p.z += 1.0;
    line_list.points.push_back(p);
    line_list.colors.push_back(line_color_b);
}

void ArDemo::cubeGenerate(visualization_msgs::msg::Marker &marker, Eigen::Vector3d &origin, int id){
    //uint32_t shape = visualization_msgs::Marker::CUBE;
    marker.header.frame_id = "world";
    marker.header.stamp = this->now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    //marker.type = shape;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;

    // marker.pose.position.x = origin.x();
    // marker.pose.position.y = origin.y();
    // marker.pose.position.z = origin.z();
    // marker.pose.orientation.x = 0.0;
    // marker.pose.orientation.y = 0.0;
    // marker.pose.orientation.z = 0.0;
    // marker.pose.orientation.w = 1.0;

    marker.scale.x = box_length;
    marker.scale.y = box_length;
    marker.scale.z = box_length;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = rclcpp::Duration::from_seconds(0.3);

    geometry_msgs::msg::Point p;
    p.x = origin.x();
    p.y = origin.y();
    p.z = origin.z();
    marker.points.push_back(p);
    marker.colors.push_back(line_color_r);
    cube_corner[id].clear();
    cube_corner[id].push_back(Eigen::Vector3d(origin.x() - box_length / 2, origin.y() - box_length / 2, origin.z() - box_length / 2));
    cube_corner[id].push_back(Eigen::Vector3d(origin.x() + box_length / 2, origin.y() - box_length / 2, origin.z() - box_length / 2));
    cube_corner[id].push_back(Eigen::Vector3d(origin.x() - box_length / 2, origin.y() + box_length / 2, origin.z() - box_length / 2));
    cube_corner[id].push_back(Eigen::Vector3d(origin.x() + box_length / 2, origin.y() + box_length / 2, origin.z() - box_length / 2));
    cube_corner[id].push_back(Eigen::Vector3d(origin.x() - box_length / 2, origin.y() - box_length / 2, origin.z() + box_length / 2));
    cube_corner[id].push_back(Eigen::Vector3d(origin.x() + box_length / 2, origin.y() - box_length / 2, origin.z() + box_length / 2));
    cube_corner[id].push_back(Eigen::Vector3d(origin.x() - box_length / 2, origin.y() + box_length / 2, origin.z() + box_length / 2));
    cube_corner[id].push_back(Eigen::Vector3d(origin.x() + box_length / 2, origin.y() + box_length / 2, origin.z() + box_length / 2));
}

void ArDemo::addObject(){
    visualization_msgs::msg::MarkerArray marker_array_msg;
    visualization_msgs::msg::Marker line_list;
    visualization_msgs::msg::Marker cube_list;

    for(int i = 0; i < axis_num; i++){
        axisGenerate(line_list, axis[i], i);
        marker_array_msg.markers.push_back(line_list);
    }

    for(int i = 0; i < cube_num; i++){
        cubeGenerate(cube_list, cube_center[i], i);
    }

    marker_array_msg.markers.push_back(cube_list);
    object_pub->publish(marker_array_msg);
}

void ArDemo::projectObject(Eigen::Vector3d camera_p, Eigen::Quaterniond camera_q){
    for(int i = 0; i < axis_num; i++){
        output_axis[i].clear();
        Eigen::Vector3d local_point;
        Eigen::Vector2d local_uv;
        local_point = camera_q.inverse() * (axis[i] - camera_p);
        m_camera->spaceToPlane(local_point, local_uv);

        if(local_point.z() > 0){
            output_axis[i].push_back(Eigen::Vector3d(local_uv.x(), local_uv.y(), 1));

            local_point = camera_q.inverse() * (axis[i] + Eigen::Vector3d(1, 0, 0) - camera_p);
            m_camera->spaceToPlane(local_point, local_uv);
            output_axis[i].push_back(Eigen::Vector3d(local_uv.x(), local_uv.y(), 1));

            local_point = camera_q.inverse() * (axis[i] + Eigen::Vector3d(0, 1, 0) - camera_p);
            m_camera->spaceToPlane(local_point, local_uv);
            output_axis[i].push_back(Eigen::Vector3d(local_uv.x(), local_uv.y(), 1));

            local_point = camera_q.inverse() * (axis[i] + Eigen::Vector3d(0, 0, 1) - camera_p);
            m_camera->spaceToPlane(local_point, local_uv);
            output_axis[i].push_back(Eigen::Vector3d(local_uv.x(), local_uv.y(), 1));
        }
    }

    for (int i = 0; i < cube_num; i++) {
        output_cube[i].clear();
        output_corner_dis[i].clear();
        Eigen::Vector3d local_point;
        Eigen::Vector2d local_uv;
        local_point = camera_q.inverse() * (cube_center[i] - camera_p);
        if (use_undistored_img){
            local_uv.x() = local_point(0) / local_point(2) * focal_length + column / 2;
            local_uv.y() = local_point(1) / local_point(2) * focal_length + row / 2;
        }
        else{
            m_camera->spaceToPlane(local_point, local_uv);
        }
        if (local_point.z() > box_length / 2) {
            cube_center_depth[i] = local_point.z();
            for (int j = 0; j < 8; j++) {
                local_point = camera_q.inverse() * (cube_corner[i][j] - camera_p);
                output_corner_dis[i].push_back(local_point.norm());
                if (use_undistored_img) {
                    //ROS_INFO("directly project!");
                    local_uv.x() = local_point(0) / local_point(2) * focal_length + column / 2;
                    local_uv.y() = local_point(1) / local_point(2) * focal_length + row / 2;
                }
                else {
                    //ROS_INFO("camera model project!");
                    m_camera->spaceToPlane(local_point, local_uv);
                    local_uv.x() = std::min(std::max(-5000.0, local_uv.x()),5000.0);
                    local_uv.y() = std::min(std::max(-5000.0, local_uv.y()),5000.0);
                }
                output_cube[i].push_back(Eigen::Vector3d(local_uv.x(), local_uv.y(), 1));
            }
        }
        else{
            cube_center_depth[i] = -1;
        }

    }   
}

void ArDemo::drawObject(cv::Mat &AR_image){
    for (int i = 0; i < axis_num; i++) {
        if(output_axis[i].empty())
            continue;
        cv::Point2d origin(output_axis[i][0].x(), output_axis[i][0].y());
        cv::Point2d axis_x(output_axis[i][1].x(), output_axis[i][1].y());
        cv::Point2d axis_y(output_axis[i][2].x(), output_axis[i][2].y());
        cv::Point2d axis_z(output_axis[i][3].x(), output_axis[i][3].y());
        cv::line(AR_image, origin, axis_x, cv::Scalar(0, 0, 255), 2, 8, 0);
        cv::line(AR_image, origin, axis_y, cv::Scalar(0, 255, 0), 2, 8, 0);
        cv::line(AR_image, origin, axis_z, cv::Scalar(255, 0, 0), 2, 8, 0);
    }

    //depth sort  big---->small
    int index[cube_num];
    for (int i = 0; i < cube_num; i++)  {
        index[i] = i;
        //cout << "i " << i << " init depth" << Cube_center_depth[i] << endl;
    }
    for (int i = 0; i < cube_num; i++)
        for (int j = 0; j < cube_num - i - 1; j++) {
            if (cube_center_depth[j] < cube_center_depth[j + 1])
            {
                double tmp = cube_center_depth[j];
                cube_center_depth[j] = cube_center_depth[j + 1];
                cube_center_depth[j + 1] = tmp;
                int tmp_index = index[j];
                index[j] = index[j + 1];
                index[j + 1] = tmp_index;
            }
        }

    for (int k = 0; k < cube_num; k++) {
        int i = index[k];
        //cout << "draw " << i << "depth " << cube_center_depth[i] << endl;
        if (output_cube[i].empty())
            continue;
        //draw color
        cv::Point* p = new cv::Point[8];
        p[0] = cv::Point(output_cube[i][0].x(), output_cube[i][0].y());
        p[1] = cv::Point(output_cube[i][1].x(), output_cube[i][1].y());
        p[2] = cv::Point(output_cube[i][2].x(), output_cube[i][2].y());
        p[3] = cv::Point(output_cube[i][3].x(), output_cube[i][3].y());
        p[4] = cv::Point(output_cube[i][4].x(), output_cube[i][4].y());
        p[5] = cv::Point(output_cube[i][5].x(), output_cube[i][5].y());
        p[6] = cv::Point(output_cube[i][6].x(), output_cube[i][6].y());
        p[7] = cv::Point(output_cube[i][7].x(), output_cube[i][7].y());
        
        int npts[1] = {4};
        float min_depth = 100000;
        int min_index = 5;
        for(int j= 0; j < (int)output_corner_dis[i].size(); j++)
        {
            if(output_corner_dis[i][j] < min_depth)
            {
                min_depth = output_corner_dis[i][j];
                min_index = j;
            }
        }
        
        cv::Point plain[1][4];
        const cv::Point* ppt[1] = {plain[0]};
        //first draw large depth plane
        int point_group[8][12] = {{0,1,5,4, 0,4,6,2, 0,1,3,2},
            {0,1,5,4, 1,5,7,3, 0,1,3,2},
            {2,3,7,6, 0,4,6,2, 0,1,3,2},
            {2,3,7,6, 1,5,7,3, 0,1,3,2},
            {0,1,5,4, 0,4,6,2, 4,5,7,6},
            {0,1,5,4, 1,5,7,3, 4,5,7,6},
            {2,3,7,6, 0,4,6,2, 4,5,7,6},
            {2,3,7,6, 1,5,7,3, 4,5,7,6}};
        
        plain[0][0] = p[point_group[min_index][4]];
        plain[0][1] = p[point_group[min_index][5]];
        plain[0][2] = p[point_group[min_index][6]];
        plain[0][3] = p[point_group[min_index][7]];
        cv::fillPoly(AR_image, ppt, npts, 1, cv::Scalar(0, 200, 0));
        
        plain[0][0] = p[point_group[min_index][0]];
        plain[0][1] = p[point_group[min_index][1]];
        plain[0][2] = p[point_group[min_index][2]];
        plain[0][3] = p[point_group[min_index][3]];
        cv::fillPoly(AR_image, ppt, npts, 1, cv::Scalar(200, 0, 0));
        
        if(output_corner_dis[i][point_group[min_index][2]] + output_corner_dis[i][point_group[min_index][3]] >
           output_corner_dis[i][point_group[min_index][5]] + output_corner_dis[i][point_group[min_index][6]])
        {
            plain[0][0] = p[point_group[min_index][4]];
            plain[0][1] = p[point_group[min_index][5]];
            plain[0][2] = p[point_group[min_index][6]];
            plain[0][3] = p[point_group[min_index][7]];
            cv::fillPoly(AR_image, ppt, npts, 1, cv::Scalar(0, 200, 0));

        }
        plain[0][0] = p[point_group[min_index][8]];
        plain[0][1] = p[point_group[min_index][9]];
        plain[0][2] = p[point_group[min_index][10]];
        plain[0][3] = p[point_group[min_index][11]];
        cv::fillPoly(AR_image, ppt, npts, 1, cv::Scalar(0, 0, 200));
        delete p;
    }
}

void ArDemo::callback(const imageConstPtr& img_msg, const odometryConstPtr pose_msg){
   if(img_cnt < 50)  {
        img_cnt ++;
        return;
    }
   //ROS_INFO("sync callback!");
   Eigen::Vector3d camera_p(pose_msg->pose.pose.position.x,
                     pose_msg->pose.pose.position.y,
                     pose_msg->pose.pose.position.z);
   Eigen::Quaterniond camera_q(pose_msg->pose.pose.orientation.w,
                        pose_msg->pose.pose.orientation.x,
                        pose_msg->pose.pose.orientation.y,
                        pose_msg->pose.pose.orientation.z);

   //test plane
   Eigen::Vector3d cam_z(0, 0, -1);
   Eigen::Vector3d w_cam_z = camera_q * cam_z;
   //cout << "angle " << acos(w_cam_z.dot(Eigen::Vector3d(0, 0, 1))) * 180.0 / M_PI << endl;
   if (acos(w_cam_z.dot(Eigen::Vector3d(0, 0, 1))) * 180.0 / M_PI < 90) {
        //ROS_WARN(" look down");
        look_ground = 1;
   }
   else
        look_ground = 0;

   projectObject(camera_p, camera_q);

   cv_bridge::CvImageConstPtr ptr;
   if (img_msg->encoding == "8UC1") {
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

   //cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(ptr, sensor_msgs::image_encodings::MONO8);
   cv::Mat AR_image;
   AR_image = ptr->image.clone();
   cv::cvtColor(AR_image, AR_image, cv::COLOR_GRAY2RGB);
   drawObject(AR_image);

   sensor_msgs::msg::Image::SharedPtr AR_msg = cv_bridge::CvImage(img_msg->header, "bgr8", AR_image).toImageMsg();
   pub_ARimage->publish(*AR_msg);
}

void ArDemo::pointCallback(const sensor_msgs::msg::PointCloud::SharedPtr &point_msg){
    if (!look_ground)
        return;
    int height_range[30];
    double height_sum[30];
    for (int i = 0; i < 30; i++) {
        height_range[i] = 0;
        height_sum[i] = 0;
    }
    for (unsigned int i = 0; i < point_msg->points.size(); i++) {
        //double x = point_msg->points[i].x;
        //double y = point_msg->points[i].y;
        double z = point_msg->points[i].z;
        int index = (z + 2.0) / 0.1;
        if (0 <= index && index < 30)
        {
            height_range[index]++;
            height_sum[index] += z;
        }
        //cout << "point " << " z " << z << endl;
    }
    int max_num = 0;
    int max_index = -1;
    for (int i = 1; i < 29; i++) {
        if (max_num < height_range[i])
        {
            max_num = height_range[i];
            max_index = i;
        }
    }
    if (max_index == -1)
        return;
    int tmp_num = height_range[max_index - 1] + height_range[max_index] + height_range[max_index + 1];
    double new_height = (height_sum[max_index - 1] + height_sum[max_index] + height_sum[max_index + 1]) / tmp_num;
    //ROS_WARN("detect ground plain, height %f", new_height);
    if (tmp_num < (int)point_msg->points.size() / 2) {
        //ROS_INFO("points not enough");
        return;
    }
    //update height
    for (int i = 0; i < cube_num; i++) {
        cube_center[i].z() = new_height + box_length / 2.0;
    }
    addObject();

}

void ArDemo::imgCallback(const imageConstPtr& img_msg){
    if(pose_init) {
        img_buf.push(img_msg);
    }
    else
        return;
}

void ArDemo::poseCallback(const odometryConstPtr &pose_msg){
    if(!pose_init)  {
        pose_init = true;
        return;
    }

    if (img_buf.empty()) {
        return;
    }

    while (img_buf.front()->header.stamp.nanosec < pose_msg->header.stamp.nanosec && !img_buf.empty()) {
        img_buf.pop();
    }

    if (!img_buf.empty()) {
        callback(img_buf.front(), pose_msg);
        img_buf.pop();
    }
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArDemo>());
    rclcpp::shutdown();
    return 0;
}