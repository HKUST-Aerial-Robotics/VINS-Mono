#include "ar_demo_node.hpp"

ArDemo::ArDemo():Node("ar_demo"){
    object_pub = this->create_publisher<markerArrayMsg>("object", 10);
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

    }
}


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArDemo>());
    rclcpp::shutdown();
    return 0;
}