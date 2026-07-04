#include <memory>
#include <vector>
#include <queue>
#include <atomic>
#include <csignal>
#include <cstring>
#include <execinfo.h>
#include <fcntl.h>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/channel_float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include "feature_tracker.h"
#include "tracking_safety.h"

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::msg::Image::ConstSharedPtr> img_buf;

rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_img;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_restart;

FeatureTracker trackerData[NUM_OF_CAM];
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;
rclcpp::Clock::SharedPtr g_clock;
static bool first_image_logged = false;
static int image_count = 0;
static int published_feature_count = 0;
static int published_debug_image_count = 0;
static std::atomic<const char *> current_stage{"startup"};
static int crash_log_fd = -1;

static void writeCrashReport(int fd, const char *stage, void *const *frames, int frame_count)
{
    if (fd < 0)
        return;
    static constexpr char prefix[] = "\n[CRASH] feature_tracker stage=";
    static constexpr char suffix[] = "\n[CRASH] native stack:\n";
    (void)::write(fd, prefix, sizeof(prefix) - 1);
    (void)::write(fd, stage, std::strlen(stage));
    (void)::write(fd, suffix, sizeof(suffix) - 1);
    ::backtrace_symbols_fd(frames, frame_count, fd);
}

static void crashSignalHandler(int signal_number)
{
    const char *stage = current_stage.load(std::memory_order_relaxed);
    void *frames[64];
    const int frame_count = ::backtrace(frames, 64);
    writeCrashReport(STDERR_FILENO, stage, frames, frame_count);
    writeCrashReport(crash_log_fd, stage, frames, frame_count);
    std::_Exit(128 + signal_number);
}

void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
    current_stage.store("callback/input", std::memory_order_relaxed);
    image_count++;
    double stamp = rclcpp::Time(img_msg->header.stamp).seconds();
    if (!first_image_logged)
    {
        first_image_logged = true;
        RCLCPP_INFO(
            rclcpp::get_logger("feature_tracker"),
            "[INPUT] first image received: topic=%s stamp=%.6f encoding=%s size=%ux%u",
            IMAGE_TOPIC.c_str(), stamp, img_msg->encoding.c_str(), img_msg->width, img_msg->height);
    }
    else if (image_count % 50 == 0)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("feature_tracker"),
            "[INPUT] image #%d stamp=%.6f queue=%zu",
            image_count, stamp, img_buf.size());
    }

    if (!tracking_safety::imageSizeIsValid(
            img_msg->width, img_msg->height, COL, ROW, NUM_OF_CAM))
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("feature_tracker"),
            "[INPUT] image size %ux%u is smaller than configured %dx%d; frame dropped",
            img_msg->width, img_msg->height, COL, ROW * NUM_OF_CAM);
        return;
    }

    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = stamp;
        last_image_time = stamp;
        RCLCPP_INFO(rclcpp::get_logger("feature_tracker"), "[STATE] tracker primed on first frame");
        return;
    }
    // detect unstable camera stream
    if (stamp - last_image_time > 1.0 || stamp < last_image_time)
    {
        RCLCPP_WARN(rclcpp::get_logger("feature_tracker"), "image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        std_msgs::msg::Bool restart_flag;
        restart_flag.data = true;
        pub_restart->publish(restart_flag);
        return;
    }
    last_image_time = stamp;
    // frequency control
    if (round(1.0 * pub_count / (stamp - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (stamp - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = stamp;
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    current_stage.store("cv_bridge/to_mono8", std::memory_order_relaxed);
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
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        current_stage.store("tracking/readImage", std::memory_order_relaxed);
        RCLCPP_DEBUG(rclcpp::get_logger("feature_tracker"), "processing camera %d", i);
        if (i != 1 || !STEREO_TRACK)
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), stamp);
        else
        {
            if (EQUALIZE)
            {
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
        current_stage.store("tracking/updateID", std::memory_order_relaxed);
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }

   if (PUB_THIS_FRAME)
   {
        current_stage.store("output/build_feature_packet", std::memory_order_relaxed);
        pub_count++;
        sensor_msgs::msg::PointCloud::SharedPtr feature_points(new sensor_msgs::msg::PointCloud);
        sensor_msgs::msg::ChannelFloat32 id_of_point;
        sensor_msgs::msg::ChannelFloat32 u_of_point;
        sensor_msgs::msg::ChannelFloat32 v_of_point;
        sensor_msgs::msg::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::msg::ChannelFloat32 velocity_y_of_point;

        feature_points->header = img_msg->header;
        feature_points->header.frame_id = "world";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            const size_t point_count = tracking_safety::commonSize(
                {un_pts.size(), cur_pts.size(), ids.size(), pts_velocity.size(),
                 trackerData[i].track_cnt.size()});
            if (point_count != ids.size() || point_count != cur_pts.size() ||
                point_count != un_pts.size() || point_count != pts_velocity.size() ||
                point_count != trackerData[i].track_cnt.size())
            {
                RCLCPP_WARN_THROTTLE(
                    rclcpp::get_logger("feature_tracker"), *g_clock, 2000,
                    "[TRACKER] inconsistent feature vectors: ids=%zu cur=%zu un=%zu vel=%zu track=%zu; using %zu",
                    ids.size(), cur_pts.size(), un_pts.size(), pts_velocity.size(),
                    trackerData[i].track_cnt.size(), point_count);
            }
            for (size_t j = 0; j < point_count; j++)
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
        RCLCPP_DEBUG(rclcpp::get_logger("feature_tracker"), "publish %f, at %f", rclcpp::Time(feature_points->header.stamp).seconds(), g_clock->now().seconds());
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
            RCLCPP_INFO(rclcpp::get_logger("feature_tracker"), "[STATE] first feature packet is ready for publication");
        }
        else
        {
            pub_img->publish(*feature_points);
            current_stage.store("output/feature_published", std::memory_order_relaxed);
            published_feature_count++;
            if (published_feature_count == 1 || published_feature_count % 20 == 0)
            {
                RCLCPP_INFO(
                    rclcpp::get_logger("feature_tracker"),
                    "[OUTPUT] published feature packet #%d with %zu points",
                    published_feature_count,
                    feature_points->points.size());
            }
        }

        if (SHOW_TRACK)
        {
            current_stage.store("output/draw_feature_image", std::memory_order_relaxed);
            cv::Mat stereo_img;
            cv::cvtColor(show_img, stereo_img, cv::COLOR_GRAY2BGR);

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                const size_t draw_count = tracking_safety::commonSize(
                    {trackerData[i].cur_pts.size(), trackerData[i].track_cnt.size(),
                     trackerData[i].pts_velocity.size()});

                for (size_t j = 0; j < draw_count; j++)
                {
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                    
                    // Calculate velocity magnitude as proxy for depth (closer = faster motion)
                    double vel_x = trackerData[i].pts_velocity[j].x;
                    double vel_y = trackerData[i].pts_velocity[j].y;
                    double vel_mag = std::sqrt(vel_x * vel_x + vel_y * vel_y);
                    
                    // Map velocity to circle radius: high velocity (close) = large radius
                    // Velocity range ~0-50 pixels/sec, map to radius 1-4
                    int radius = 1 + static_cast<int>(std::min(3.0, vel_mag / 10.0));
                    
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], radius, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
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
            auto debug_msg = cv_bridge::CvImage(
                img_msg->header, sensor_msgs::image_encodings::BGR8, stereo_img).toImageMsg();
            pub_match->publish(*debug_msg);
            current_stage.store("output/feature_image_published", std::memory_order_relaxed);
            published_debug_image_count++;
            if (published_debug_image_count == 1 || published_debug_image_count % 20 == 0)
            {
                RCLCPP_INFO(
                    rclcpp::get_logger("feature_tracker"),
                    "[OUTPUT] published feature image #%d with %zu drawn points on /feature_tracker/feature_img",
                    published_debug_image_count, trackerData[0].cur_pts.size());
            }
        }
        
        // Print tracking statistics with velocity info
        if (ENABLE_VELOCITY_CHECK && (published_debug_image_count == 1 || published_debug_image_count % 20 == 0))
        {
            RCLCPP_INFO(rclcpp::get_logger("feature_tracker"), "[TRACKER] Features: %lu/%d | Velocity: %.2f m/s | Time: %.1fms",
                     trackerData[0].cur_pts.size(), 
                     trackerData[0].adaptive_max_cnt,
                     trackerData[0].current_velocity,
                     t_r.toc());
        }
    }
    current_stage.store("callback/complete", std::memory_order_relaxed);
    // RCLCPP_INFO(rclcpp::get_logger("feature_tracker"), "whole feature tracker processing costs: %f", t_r.toc());
}

int main(int argc, char **argv)
{
    crash_log_fd = ::open(
        "/tmp/feature_tracker_crash.log",
        O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC,
        0644);
    std::signal(SIGSEGV, crashSignalHandler);
    std::signal(SIGABRT, crashSignalHandler);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("feature_tracker");
    g_clock = node->get_clock();
    readParameters(node);
    RCLCPP_INFO(
        node->get_logger(),
        "[START] feature_tracker subscribing to image_topic=%s | freq=%d | max_cnt=%d | min_dist=%d | fisheye=%d | show_track=%d",
        IMAGE_TOPIC.c_str(), FREQ, MAX_CNT, MIN_DIST, FISHEYE, SHOW_TRACK);
    RCLCPP_INFO(node->get_logger(), "[START] image_input_format=%s", IMAGE_INPUT_FORMAT.c_str());

    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                RCLCPP_ERROR(node->get_logger(), "failed to load feature mask: %s", FISHEYE_MASK.c_str());
                std::abort();
            }
            else if (trackerData[i].fisheye_mask.cols != COL ||
                     trackerData[i].fisheye_mask.rows != ROW)
            {
                RCLCPP_ERROR(
                    node->get_logger(),
                    "feature mask size %dx%d does not match configured image %dx%d",
                    trackerData[i].fisheye_mask.cols, trackerData[i].fisheye_mask.rows, COL, ROW);
                std::abort();
            }
            else
            {
                cv::threshold(
                    trackerData[i].fisheye_mask, trackerData[i].fisheye_mask,
                    127, 255, cv::THRESH_BINARY);
                const int enabled_pixels = cv::countNonZero(trackerData[i].fisheye_mask);
                const double enabled_percent = 100.0 * enabled_pixels /
                    static_cast<double>(trackerData[i].fisheye_mask.total());
                RCLCPP_INFO(
                    node->get_logger(),
                    "feature mask loaded: %s size=%dx%d enabled=%.1f%%",
                    FISHEYE_MASK.c_str(), COL, ROW, enabled_percent);
            }
        }
    }

    auto sub_img = image_transport::create_subscription(
        node.get(), IMAGE_TOPIC, img_callback, IMAGE_INPUT_FORMAT);
    RCLCPP_INFO(
        node->get_logger(),
        "[START] image_transport subscription created for base_topic=%s transport=%s",
        IMAGE_TOPIC.c_str(), IMAGE_INPUT_FORMAT.c_str());

    pub_img = node->create_publisher<sensor_msgs::msg::PointCloud>("/feature_tracker/feature", 1000);
    pub_match = node->create_publisher<sensor_msgs::msg::Image>("/feature_tracker/feature_img", 1000);
    pub_restart = node->create_publisher<std_msgs::msg::Bool>("/feature_tracker/restart", 1000);
    RCLCPP_INFO(
        node->get_logger(),
        "[START] outputs: feature=/feature_tracker/feature | feature image=/feature_tracker/feature_img (%s)",
        SHOW_TRACK ? "enabled" : "disabled by show_track");
    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


// new points velocity is 0, pub or not?
// track cnt > 1 pub?
