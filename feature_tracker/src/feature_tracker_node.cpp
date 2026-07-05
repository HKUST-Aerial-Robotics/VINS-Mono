#include <memory>
#include <vector>
#include <queue>
#include <atomic>
#include <csignal>
#include <cstring>
#include <execinfo.h>
#include <fcntl.h>
#include <unistd.h>
#include <unordered_set>
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
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

static rcl_interfaces::msg::ParameterDescriptor integerDescriptor(
    const std::string &description, int64_t from, int64_t to)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = from;
    range.to_value = to;
    range.step = 1;
    descriptor.integer_range.push_back(range);
    return descriptor;
}

static rcl_interfaces::msg::ParameterDescriptor floatingDescriptor(
    const std::string &description, double from, double to, double step = 0.0)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = from;
    range.to_value = to;
    range.step = step;
    descriptor.floating_point_range.push_back(range);
    return descriptor;
}

static rcl_interfaces::msg::ParameterDescriptor plainDescriptor(const std::string &description)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    return descriptor;
}

static std::string resolveMaskPath(const std::string &path)
{
    if (path.empty() || path.front() == '/') return path;
    return VINS_FOLDER_PATH + path;
}

static bool loadFeatureMask(const std::string &path, cv::Mat &loaded, std::string &reason)
{
    const std::string resolved = resolveMaskPath(path);
    loaded = cv::imread(resolved, cv::IMREAD_GRAYSCALE);
    if (loaded.empty())
    {
        reason = "cannot read fisheye_mask: " + resolved;
        return false;
    }
    if (loaded.cols != COL || loaded.rows != ROW)
    {
        reason = "fisheye_mask size " + std::to_string(loaded.cols) + "x" +
            std::to_string(loaded.rows) + " does not match image " +
            std::to_string(COL) + "x" + std::to_string(ROW);
        return false;
    }
    cv::threshold(loaded, loaded, 127, 255, cv::THRESH_BINARY);
    return true;
}

static void declareRuntimeParameters(const rclcpp::Node::SharedPtr &node)
{
    MAX_CNT = node->declare_parameter<int>("max_cnt", MAX_CNT,
        integerDescriptor("Maximum tracked feature count", 8, 5000));
    MIN_DIST = node->declare_parameter<int>("min_dist", MIN_DIST,
        integerDescriptor("Minimum feature spacing in pixels", 1, 500));
    FREQ = node->declare_parameter<int>("freq", FREQ,
        integerDescriptor("Feature publication and replenishment rate in Hz", 1, 240));
    F_THRESHOLD = node->declare_parameter<double>("F_threshold", F_THRESHOLD,
        floatingDescriptor("Fundamental-matrix RANSAC threshold in pixels", 0.1, 10.0, 0.1));
    GFTT_QUALITY_LEVEL = node->declare_parameter<double>("gftt_quality_level", GFTT_QUALITY_LEVEL,
        floatingDescriptor("Shi-Tomasi relative quality threshold", 0.0001, 1.0));
    FAST_THRESHOLD = node->declare_parameter<int>("fast_threshold", FAST_THRESHOLD,
        integerDescriptor("AGAST response threshold", 1, 255));
    ENABLE_FEATURE_AUTO_TUNING = node->declare_parameter<bool>(
        "enable_feature_auto_tuning", ENABLE_FEATURE_AUTO_TUNING != 0,
        plainDescriptor("Periodically tune the active detector threshold"));
    FEATURE_AUTO_TUNE_INTERVAL = node->declare_parameter<double>(
        "feature_auto_tune_interval", FEATURE_AUTO_TUNE_INTERVAL,
        floatingDescriptor("Auto-tuning interval in seconds", 0.5, 3600.0, 0.5));
    FEATURE_AUTO_TUNE_LOG = node->declare_parameter<bool>(
        "feature_auto_tune_log", FEATURE_AUTO_TUNE_LOG != 0,
        plainDescriptor("Log periodic auto-tuning diagnostics"));
    FAST_THRESHOLD_MIN = node->declare_parameter<int>("fast_threshold_min", FAST_THRESHOLD_MIN,
        integerDescriptor("Minimum AGAST auto-tuning threshold", 1, 255));
    FAST_THRESHOLD_MAX = node->declare_parameter<int>("fast_threshold_max", FAST_THRESHOLD_MAX,
        integerDescriptor("Maximum AGAST auto-tuning threshold", 1, 255));
    GFTT_QUALITY_MIN = node->declare_parameter<double>("gftt_quality_min", GFTT_QUALITY_MIN,
        floatingDescriptor("Minimum Shi-Tomasi auto-tuning threshold", 0.0001, 1.0));
    GFTT_QUALITY_MAX = node->declare_parameter<double>("gftt_quality_max", GFTT_QUALITY_MAX,
        floatingDescriptor("Maximum Shi-Tomasi auto-tuning threshold", 0.0001, 1.0));
    USE_BIDIRECTIONAL_FLOW = node->declare_parameter<bool>(
        "use_bidirectional_flow", USE_BIDIRECTIONAL_FLOW != 0,
        plainDescriptor("Enable forward-backward optical-flow validation"));
    USE_ADVANCED_FLOW = node->declare_parameter<bool>(
        "use_advanced_flow", USE_ADVANCED_FLOW != 0,
        plainDescriptor("Use AGAST and advanced LK instead of Shi-Tomasi/basic LK"));
    SHOW_TRACK = node->declare_parameter<bool>("show_track", SHOW_TRACK != 0,
        plainDescriptor("Publish the feature visualization image"));
    EQUALIZE = node->declare_parameter<bool>("equalize", EQUALIZE != 0,
        plainDescriptor("Apply CLAHE before tracking"));
    FISHEYE = node->declare_parameter<bool>("fisheye", FISHEYE != 0,
        plainDescriptor("Apply the feature validity mask"));
    FISHEYE_MASK = resolveMaskPath(node->declare_parameter<std::string>(
        "fisheye_mask", FISHEYE_MASK, plainDescriptor("Feature validity mask path")));
}

static rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
registerRuntimeParameterCallback(const rclcpp::Node::SharedPtr &node)
{
    return node->add_on_set_parameters_callback(
        [node](const std::vector<rclcpp::Parameter> &parameters) {
            static const std::unordered_set<std::string> runtime_names = {
                "max_cnt", "min_dist", "freq", "F_threshold", "gftt_quality_level",
                "fast_threshold", "enable_feature_auto_tuning", "feature_auto_tune_interval",
                "feature_auto_tune_log", "fast_threshold_min", "fast_threshold_max",
                "gftt_quality_min", "gftt_quality_max", "use_bidirectional_flow",
                "use_advanced_flow", "show_track", "equalize", "fisheye", "fisheye_mask"};
            int max_cnt = MAX_CNT, min_dist = MIN_DIST, freq = FREQ;
            int fast_threshold = FAST_THRESHOLD, fast_min = FAST_THRESHOLD_MIN, fast_max = FAST_THRESHOLD_MAX;
            double f_threshold = F_THRESHOLD, gftt = GFTT_QUALITY_LEVEL;
            double tune_interval = FEATURE_AUTO_TUNE_INTERVAL;
            double gftt_min = GFTT_QUALITY_MIN, gftt_max = GFTT_QUALITY_MAX;
            int auto_tune = ENABLE_FEATURE_AUTO_TUNING, tune_log = FEATURE_AUTO_TUNE_LOG;
            int bidirectional = USE_BIDIRECTIONAL_FLOW, advanced = USE_ADVANCED_FLOW;
            int show_track = SHOW_TRACK, equalize = EQUALIZE, fisheye = FISHEYE;
            std::string mask_path = FISHEYE_MASK;
            bool freq_changed = false;
            bool mask_changed = false;
            bool has_runtime_parameter = false;

            for (const auto &parameter : parameters)
            {
                const auto &name = parameter.get_name();
                if (runtime_names.count(name) == 0) continue;
                has_runtime_parameter = true;
                if (name == "max_cnt") max_cnt = parameter.as_int();
                else if (name == "min_dist") min_dist = parameter.as_int();
                else if (name == "freq") { freq = parameter.as_int(); freq_changed = true; }
                else if (name == "F_threshold") f_threshold = parameter.as_double();
                else if (name == "gftt_quality_level") gftt = parameter.as_double();
                else if (name == "fast_threshold") fast_threshold = parameter.as_int();
                else if (name == "enable_feature_auto_tuning") auto_tune = parameter.as_bool();
                else if (name == "feature_auto_tune_interval") tune_interval = parameter.as_double();
                else if (name == "feature_auto_tune_log") tune_log = parameter.as_bool();
                else if (name == "fast_threshold_min") fast_min = parameter.as_int();
                else if (name == "fast_threshold_max") fast_max = parameter.as_int();
                else if (name == "gftt_quality_min") gftt_min = parameter.as_double();
                else if (name == "gftt_quality_max") gftt_max = parameter.as_double();
                else if (name == "use_bidirectional_flow") bidirectional = parameter.as_bool();
                else if (name == "use_advanced_flow") advanced = parameter.as_bool();
                else if (name == "show_track") show_track = parameter.as_bool();
                else if (name == "equalize") equalize = parameter.as_bool();
                else if (name == "fisheye") fisheye = parameter.as_bool();
                else if (name == "fisheye_mask") { mask_path = parameter.as_string(); mask_changed = true; }
            }

            rcl_interfaces::msg::SetParametersResult result;
            if (!has_runtime_parameter)
            {
                result.successful = true;
                return result;
            }
            result.successful = false;
            if (fast_min > fast_max) { result.reason = "fast_threshold_min must be <= fast_threshold_max"; return result; }
            if (gftt_min > gftt_max) { result.reason = "gftt_quality_min must be <= gftt_quality_max"; return result; }
            if (min_dist >= std::max(ROW, COL)) { result.reason = "min_dist must be smaller than the image size"; return result; }

            cv::Mat loaded_mask;
            if (mask_changed || (fisheye && trackerData[0].fisheye_mask.empty()))
            {
                if (!loadFeatureMask(mask_path, loaded_mask, result.reason)) return result;
            }

            MAX_CNT = max_cnt; MIN_DIST = min_dist; FREQ = freq; F_THRESHOLD = f_threshold;
            FAST_THRESHOLD_MIN = fast_min; FAST_THRESHOLD_MAX = fast_max;
            GFTT_QUALITY_MIN = gftt_min; GFTT_QUALITY_MAX = gftt_max;
            FAST_THRESHOLD = std::clamp(fast_threshold, fast_min, fast_max);
            GFTT_QUALITY_LEVEL = std::clamp(gftt, gftt_min, gftt_max);
            ENABLE_FEATURE_AUTO_TUNING = auto_tune; FEATURE_AUTO_TUNE_INTERVAL = tune_interval;
            FEATURE_AUTO_TUNE_LOG = tune_log; USE_BIDIRECTIONAL_FLOW = bidirectional;
            USE_ADVANCED_FLOW = advanced; SHOW_TRACK = show_track; EQUALIZE = equalize;
            FISHEYE = fisheye;
            if (mask_changed)
            {
                FISHEYE_MASK = resolveMaskPath(mask_path);
                for (auto &tracker : trackerData) tracker.fisheye_mask = loaded_mask.clone();
            }
            else if (!loaded_mask.empty())
            {
                for (auto &tracker : trackerData) tracker.fisheye_mask = loaded_mask.clone();
            }
            if (freq_changed)
            {
                first_image_time = last_image_time;
                pub_count = 0;
            }
            result.successful = true;
            result.reason = "applied";
            RCLCPP_INFO(node->get_logger(),
                "[RUNTIME_CONFIG] max_cnt=%d min_dist=%d freq=%d detector=%s fast=%d gftt=%.5f auto_tune=%d",
                MAX_CNT, MIN_DIST, FREQ, USE_ADVANCED_FLOW ? "AGAST" : "Shi-Tomasi",
                FAST_THRESHOLD, GFTT_QUALITY_LEVEL, ENABLE_FEATURE_AUTO_TUNING);
            return result;
        });
}

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
    declareRuntimeParameters(node);
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

    // Keep the handle alive for the lifetime of the node. rqt and
    // `ros2 param set` now update the tracker without restarting it.
    auto runtime_parameter_callback = registerRuntimeParameterCallback(node);

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
    sub_img.shutdown();
    runtime_parameter_callback.reset();
    pub_restart.reset();
    pub_match.reset();
    pub_img.reset();
    g_clock.reset();
    rclcpp::shutdown();
    return 0;
}


// new points velocity is 0, pub or not?
// track cnt > 1 pub?
