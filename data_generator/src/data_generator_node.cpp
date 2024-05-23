#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include "data_generator.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;
using namespace Eigen;

#define ROW 480
#define COL 752

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_generator");
    ros::NodeHandle n("~");

    ros::Publisher pub_imu = n.advertise<sensor_msgs::Imu>("imu", 1000);
    ros::Publisher pub_feature = n.advertise<sensor_msgs::PointCloud>("/feature_tracker/feature", 1000);
    ros::Publisher pub_wifi = n.advertise<sensor_msgs::PointCloud>("wifi", 1000);
    ros::Publisher pub_flow = n.advertise<nav_msgs::Odometry>("flow", 1000);


    ros::Publisher pub_path = n.advertise<nav_msgs::Path>("path", 1000);
    ros::Publisher pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);

    ros::Publisher pub_pose = n.advertise<geometry_msgs::PoseStamped>("pose", 1000);
    ros::Publisher pub_cloud = n.advertise<sensor_msgs::PointCloud>("cloud", 1000);
    ros::Publisher pub_ap = n.advertise<sensor_msgs::PointCloud>("ap", 1000);
    ros::Publisher pub_line = n.advertise<visualization_msgs::Marker>("sar", 1000);

    image_transport::ImageTransport it(n);
    image_transport::Publisher pub_image;
    pub_image = it.advertise("tracked_image", 1000);

    ros::Duration(1).sleep();

    DataGenerator generator;
    ros::Rate loop_rate(generator.FREQ);

    //if (argc == 1)
    //    while (pub_imu.getNumSubscribers() == 0)
    //        loop_rate.sleep();

    sensor_msgs::PointCloud point_cloud;
    point_cloud.header.frame_id = "world";
    point_cloud.header.stamp = ros::Time::now();
    for (auto &it : generator.getCloud())
    {
        geometry_msgs::Point32 p;
        p.x = it(0);
        p.y = it(1);
        p.z = it(2);
        point_cloud.points.push_back(p);
    }
    pub_cloud.publish(point_cloud);

    point_cloud.points.clear();

    visualization_msgs::Marker line_ap[DataGenerator::NUMBER_OF_AP];
    for (int i = 0; i < DataGenerator::NUMBER_OF_AP; i++)
    {
        geometry_msgs::Point32 p;
        Vector3d p_ap = generator.getAP(i);
        p.x = p_ap(0);
        p.y = p_ap(1);
        p.z = p_ap(2);
        point_cloud.points.push_back(p);

        line_ap[i].id = i;
        line_ap[i].header.frame_id = "world";
        line_ap[i].ns = "line";
        line_ap[i].action = visualization_msgs::Marker::ADD;
        line_ap[i].pose.orientation.w = 1.0;
        line_ap[i].type = visualization_msgs::Marker::LINE_STRIP;
        line_ap[i].scale.x = 0.1;
        line_ap[i].color.r = 1.0;
        line_ap[i].color.a = 1.0;

        geometry_msgs::Point p2;
        p2.x = p.x;
        p2.y = p.y;
        p2.z = p.z;
        line_ap[i].points.push_back(p2);
        line_ap[i].points.push_back(p2);
    }
    pub_ap.publish(point_cloud);

    int publish_count = 0;

    nav_msgs::Path path;
    path.header.frame_id = "world";

    while (ros::ok())
    {
        double current_time = generator.getTime();
        ROS_INFO("time: %lf", current_time);

        Vector3d position = generator.getPosition();
        Vector3d velocity = generator.getVelocity();
        Matrix3d rotation = generator.getRotation();
        Quaterniond q(rotation);

        Vector3d linear_acceleration = generator.getLinearAcceleration();
        Vector3d angular_velocity = generator.getAngularVelocity();

        nav_msgs::Odometry odometry;
        odometry.header.frame_id = "world";
        odometry.header.stamp = ros::Time(current_time);
        odometry.pose.pose.position.x = position(0);
        odometry.pose.pose.position.y = position(1);
        odometry.pose.pose.position.z = position(2);
        odometry.pose.pose.orientation.x = q.x();
        odometry.pose.pose.orientation.y = q.y();
        odometry.pose.pose.orientation.z = q.z();
        odometry.pose.pose.orientation.w = q.w();
        odometry.twist.twist.linear.x = velocity(0);
        odometry.twist.twist.linear.y = velocity(1);
        odometry.twist.twist.linear.z = velocity(2);
        pub_odometry.publish(odometry);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "world";
        pose_stamped.header.stamp = ros::Time(current_time);
        pose_stamped.pose = odometry.pose.pose;
        path.poses.push_back(pose_stamped);
        pub_path.publish(path);
        pub_pose.publish(pose_stamped);

        for (int i = 0; i < DataGenerator::NUMBER_OF_AP; i++)
        {
            line_ap[i].header.stamp = ros::Time(current_time);
            line_ap[i].points.back().x = position(0);
            line_ap[i].points.back().y = position(1);
            line_ap[i].points.back().z = position(2);
            pub_line.publish(line_ap[i]);
        }

        sensor_msgs::Imu imu;
        imu.header.frame_id = "world";
        imu.header.stamp = ros::Time(current_time);
        imu.linear_acceleration.x = linear_acceleration(0);
        imu.linear_acceleration.y = linear_acceleration(1);
        imu.linear_acceleration.z = linear_acceleration(2);
        imu.angular_velocity.x = angular_velocity(0);
        imu.angular_velocity.y = angular_velocity(1);
        imu.angular_velocity.z = angular_velocity(2);
        imu.orientation.x = q.x();
        imu.orientation.y = q.y();
        imu.orientation.z = q.z();
        imu.orientation.w = q.w();

        pub_imu.publish(imu);
        //ROS_INFO("publish imu data with stamp %lf", imu.header.stamp.toSec());

        //publish wifi data
        if (publish_count % generator.IMU_PER_WIFI == 0)
        {
            sensor_msgs::PointCloud wifi;
            sensor_msgs::ChannelFloat32 id_ap;
            wifi.header.stamp = ros::Time(current_time);

            for (int i = 0; i < DataGenerator::NUMBER_OF_AP; i++)
            {
                Vector3d sar;
                sar(0) = line_ap[i].points[0].x - line_ap[i].points[1].x;
                sar(1) = line_ap[i].points[0].y - line_ap[i].points[1].y;
                sar(2) = line_ap[i].points[0].z - line_ap[i].points[1].z;
                sar.normalize();
                geometry_msgs::Point32 p;
                p.x = sar.dot(rotation.col(0));
                p.y = 0.0;
                p.z = 0.0;
                wifi.points.push_back(p);
                id_ap.values.push_back(i);
            }
            wifi.channels.push_back(id_ap);
            pub_wifi.publish(wifi);
        }

        //publish image data
        if (publish_count % generator.IMU_PER_IMG == 0)
        {
            ROS_INFO("feature count: %lu", generator.getImage().size());
            sensor_msgs::PointCloud feature;
            sensor_msgs::ChannelFloat32 ids;
            sensor_msgs::ChannelFloat32 pixel;
            sensor_msgs::ChannelFloat32 p_x, p_y, p_z;
            for (int i = 0; i < ROW; i++)
                for (int j = 0; j < COL; j++)
                    pixel.values.push_back(255);
            feature.header.stamp = ros::Time(current_time);
            feature.header.frame_id = "world";

            cv::Mat simu_img[DataGenerator::NUMBER_OF_CAMERA];
            for (int i = 0; i < DataGenerator::NUMBER_OF_CAMERA; i++)
                simu_img[i] = cv::Mat(600, 600, CV_8UC3, cv::Scalar(0, 0, 0));

            int tmp_idx = 0;
            for (auto &id_pts : generator.getImage())
            {
                int id = id_pts.first;
                geometry_msgs::Point32 p;
                p.x = id_pts.second(0);
                p.y = id_pts.second(1);
                p.z = id_pts.second(2);

                feature.points.push_back(p);
                ids.values.push_back(id);
                p_x.values.push_back(generator.output_gr_pts[tmp_idx].x());
                p_y.values.push_back(generator.output_gr_pts[tmp_idx].y());
                p_z.values.push_back(generator.output_gr_pts[tmp_idx].z());
                tmp_idx++;

                char label[10];
                sprintf(label, "%d", id / DataGenerator::NUMBER_OF_CAMERA);
                cv::putText(simu_img[id % DataGenerator::NUMBER_OF_CAMERA], label, cv::Point2d(p.x + 1, p.y + 1) * 0.5 * 600, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
            }

            for (int i = 0; i < 6; i++)
            {
                if(generator.output_Axis[i].empty())
                    continue;
                cv::Point2d origin((generator.output_Axis[i][0].x() + 1) * 300, (generator.output_Axis[i][0].y() + 1) * 300);
                cv::Point2d axis_x((generator.output_Axis[i][1].x() + 1) * 300, (generator.output_Axis[i][1].y() + 1) * 300);
                cv::Point2d axis_y((generator.output_Axis[i][2].x() + 1) * 300, (generator.output_Axis[i][2].y() + 1) * 300);
                cv::Point2d axis_z((generator.output_Axis[i][3].x() + 1) * 300, (generator.output_Axis[i][3].y() + 1) * 300);
                //cv::line(simu_img[0], origin, axis_x, cv::Scalar(0, 255, 0), 2, 8, 0);
                //cv::line(simu_img[0], origin, axis_y, cv::Scalar(0, 0, 255), 2, 8, 0);
                //cv::line(simu_img[0], origin, axis_z, cv::Scalar(255, 0, 0), 2, 8, 0);
            }
            feature.channels.push_back(ids);
            feature.channels.push_back(pixel);
            feature.channels.push_back(p_x);
            feature.channels.push_back(p_y);
            feature.channels.push_back(p_z);
            pub_feature.publish(feature);
            ROS_INFO("publish image data with stamp %lf", feature.header.stamp.toSec());
            for (int k = 0; k < DataGenerator::NUMBER_OF_CAMERA; k++)
            {
                char name[] = "camera 1";
                name[7] += k;
                cv::imshow(name, simu_img[k]);
                cv::Mat gray_image;
                cv::cvtColor(simu_img[k], gray_image, CV_BGR2GRAY);
                sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(feature.header, "mono8", gray_image).toImageMsg();
                pub_image.publish(img_msg);
            }
            cv::waitKey(1);
            if (generator.getTime() > 3 * DataGenerator::MAX_TIME)
                break;
        }

        //update work
        generator.update();
        publish_count++;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
