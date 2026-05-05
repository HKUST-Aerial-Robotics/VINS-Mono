#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <deque>
#include <mutex>
#include <cmath>

/*
 * VisionPoseBridge — forwards VINS-Mono odometry to ArduPilot via
 * /mavros/vision_pose/pose (and /mavros/vision_speed/twist) with
 * dual-mode sanity checking and recovery, mirrors the sanitized pose
 * on /vins_bridge/pose for downstream consumers.
 *
 * States:
 *   PRE_INIT  →  Publishes (0,0,0) + identity orientation at ~rate Hz.
 *                Allows arm in PosHold.
 *                Transition: VINS odometry arrives (passes sanity) → ACTIVE
 *
 *   ACTIVE    →  Forwards VINS position + orientation.
 *                Transitions:
 *                  - VINS data stale > odom_timeout → DROPOUT
 *                  - Sanity check fails → DROPOUT
 *
 *   DROPOUT   →  Stops publishing. EKF dead-reckons.
 *                Transitions:
 *                  - VINS resumes (passes sanity) → ACTIVE
 *                  - Drone is disarmed → PRE_INIT
 *
 * Sanity check: combined velocity + acceleration test.
 *   Velocity  > max_speed  → hard divergence, immediate DROPOUT.
 *   Accel     > max_accel  → transient spike; increment counter.
 *   Violation counter uses a time window:
 *     - 2 consecutive → DROPOUT
 *     - 5 within 1 s  → DROPOUT
 *     - otherwise decay after 2 s of clean samples.
 *
 * Published topics:
 *   /mavros/vision_pose/pose      — position + orientation for EKF3 pose
 *   /mavros/vision_speed/twist    — linear velocity for EKF3
 *   /vins_bridge/pose             — mirror (telemetry / debug)
 *
 * Parameters (ROS):
 *   ~odom_timeout  seconds without data before DROPOUT (default 1.0)
 *   ~rate          publish rate Hz (default 10.0)
 *   ~max_accel     per-sample acceleration warn cap  [m/s²] (default 40.0)
 *   ~max_speed     hard divergence speed cap         [m/s] (default 30.0)
 *   ~pose_frame    frame_id for the outgoing PoseStamped (default "odom")
 *   ~publish_speed publish /mavros/vision_speed/twist (default true)
 */

class VisionPoseBridge
{
    enum class State { PRE_INIT, ACTIVE, DROPOUT };

public:
    VisionPoseBridge()
        : state_(State::PRE_INIT)
        , is_armed_(false)
        , last_odom_time_(0)
        , have_prev_pos_(false)
        , have_prev_vel_(false)
    {
        ros::NodeHandle pnh("~");
        pnh.param("max_accel",    max_accel_,    40.0);
        pnh.param("max_speed",    max_speed_,    30.0);
        pnh.param("odom_timeout", odom_timeout_, 1.0);
        pnh.param<std::string>("pose_frame", pose_frame_, "odom");
        pnh.param("publish_speed", publish_speed_, true);
        double rate;
        pnh.param("rate", rate, 10.0);

        pub_mavros_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "/mavros/vision_pose/pose", 10);
        pub_mirror_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "/vins_bridge/pose", 10);
        if (publish_speed_)
            pub_speed_ = nh_.advertise<geometry_msgs::TwistStamped>(
                "/mavros/vision_speed/twist", 10);
        sub_odom_ = nh_.subscribe(
            "/vins_estimator/odometry", 10,
            &VisionPoseBridge::odomCallback, this);
        sub_state_ = nh_.subscribe(
            "/mavros/state", 5,
            &VisionPoseBridge::stateCallback, this);

        timer_ = nh_.createTimer(
            ros::Duration(1.0 / rate),
            &VisionPoseBridge::timerCallback, this);

        ROS_INFO("vision_pose_bridge: rate=%.0fHz frame=%s speed=%s",
                 rate, pose_frame_.c_str(), publish_speed_ ? "on" : "off");
        ROS_INFO("  max_accel=%.1f m/s² max_speed=%.1f m/s timeout=%.1fs",
                 max_accel_, max_speed_, odom_timeout_);
    }

private:
    // ── Combined speed + accel sanity (3.3) ─────────────────────────
    //
    // Returns:
    //   >= 0  : pass (value is the implied speed — for diagnostics)
    //   -1    : velocity hard cap exceeded, immediate DROPOUT
    //   -2    : accel spike; may still pass depending on violation window
    bool checkSanity(const geometry_msgs::Point &pos, double stamp)
    {
        if (!have_prev_pos_)
        {
            prev_pos_ = pos;
            prev_stamp_ = stamp;
            prev_dt_ = 0;
            have_prev_pos_ = true;
            have_prev_vel_ = false;
            viol_times_.clear();
            return true;
        }

        double dt = stamp - prev_stamp_;
        // 5.3: if dt is too large (e.g. bridge returned from DROPOUT via
        // long gap), we can't trust the finite-difference velocity. Treat
        // as "re-entry" — reset prev and skip this sample's sanity.
        if (dt < 0.001 || dt > 0.5)
        {
            prev_pos_ = pos;
            prev_stamp_ = stamp;
            have_prev_vel_ = false;
            viol_times_.clear();
            return true;
        }

        double vx = (pos.x - prev_pos_.x) / dt;
        double vy = (pos.y - prev_pos_.y) / dt;
        double vz = (pos.z - prev_pos_.z) / dt;
        double speed = std::sqrt(vx * vx + vy * vy + vz * vz);

        prev_pos_ = pos;
        prev_stamp_ = stamp;

        // Hard cap: vehicle can't physically move this fast (3.3). If it
        // looks like it is, VIO jumped — no amount of spike-filtering
        // recovers it. Force DROPOUT.
        if (speed > max_speed_)
        {
            ROS_WARN("vision_pose_bridge: speed=%.1f m/s > cap %.1f — DROPOUT",
                     speed, max_speed_);
            return false;
        }

        if (!have_prev_vel_)
        {
            prev_vx_ = vx; prev_vy_ = vy; prev_vz_ = vz;
            prev_dt_ = dt;
            have_prev_vel_ = true;
            viol_times_.clear();
            return true;
        }

        double dt_accel = (prev_dt_ + dt) * 0.5;
        if (dt_accel < 0.001) dt_accel = dt;

        double ax = (vx - prev_vx_) / dt_accel;
        double ay = (vy - prev_vy_) / dt_accel;
        double az = (vz - prev_vz_) / dt_accel;
        double accel = std::sqrt(ax*ax + ay*ay + az*az);

        prev_vx_ = vx; prev_vy_ = vy; prev_vz_ = vz;
        prev_dt_ = dt;

        // Accel warn — vehicle can't reach this, so it's a position spike.
        // Filter rather than DROP to avoid kicking out on a single noisy
        // sample. Escalation logic (5.2):
        //   - cleanup: violations older than 2s are dropped
        //   - trigger: 2 consecutive or 5 within 1s → DROPOUT
        while (!viol_times_.empty() && stamp - viol_times_.front() > 2.0)
            viol_times_.pop_front();

        if (accel > max_accel_)
        {
            viol_times_.push_back(stamp);
            // consecutive detection: previous sample was also a violation
            bool consecutive = viol_times_.size() >= 2 &&
                               (stamp - viol_times_[viol_times_.size() - 2]) < 0.2;
            // storm: 5+ within 1 s
            int within_1s = 0;
            for (auto it = viol_times_.rbegin(); it != viol_times_.rend(); ++it)
            {
                if (stamp - *it > 1.0) break;
                within_1s++;
            }
            ROS_WARN("vision_pose_bridge: accel=%.1f m/s² (cap %.1f), viol=%zu "
                     "(consec=%d within1s=%d)",
                     accel, max_accel_, viol_times_.size(),
                     (int)consecutive, within_1s);

            if (consecutive || within_1s >= 5)
                return false;
            // single violation — keep going but DON'T forward this sample
            return true;
        }
        return true;
    }

    void resetSanity()
    {
        have_prev_pos_ = false;
        have_prev_vel_ = false;
        prev_dt_ = 0;
        viol_times_.clear();
    }

    // ── Callbacks ─────────────────────────────────────────────────
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // Snapshot inputs under lock; logging done outside the lock so the
        // publisher thread doesn't stall on it (5.8). transition_to_*
        // flags carry the state change into the post-lock log section.
        bool transition_to_active = false;
        bool transition_to_dropout = false;
        State prev_state = State::PRE_INIT;

        // 5.7: use the message's own timestamp, not ros::Time::now().
        // The VINS pipeline takes tens of ms from image to pose; using
        // ::now() adds that delay as attitude/position lag to the EKF.
        double msg_t = msg->header.stamp.toSec();

        {
            std::lock_guard<std::mutex> lock(mtx_);

            if (!checkSanity(msg->pose.pose.position, msg_t))
            {
                if (state_ == State::ACTIVE)
                {
                    prev_state = state_;
                    state_ = State::DROPOUT;
                    transition_to_dropout = true;
                    resetSanity();
                }
                return;
            }

            last_odom_time_ = msg_t;
            last_pose_.header = msg->header;
            last_pose_.header.frame_id = pose_frame_;
            last_pose_.pose.position = msg->pose.pose.position;
            last_pose_.pose.orientation = msg->pose.pose.orientation;
            last_twist_.header = msg->header;
            last_twist_.header.frame_id = pose_frame_;
            last_twist_.twist.linear = msg->twist.twist.linear;
            last_twist_.twist.angular = msg->twist.twist.angular;
            have_last_twist_ = true;

            if (state_ != State::ACTIVE)
            {
                prev_state = state_;
                state_ = State::ACTIVE;
                transition_to_active = true;
            }
        }

        if (transition_to_dropout)
            ROS_WARN("vision_pose_bridge: sanity fail -> DROPOUT");
        if (transition_to_active)
            ROS_INFO("vision_pose_bridge: -> ACTIVE%s",
                     prev_state == State::PRE_INIT ? " (initialized)" : " (recovered)");
    }

    void stateCallback(const mavros_msgs::State::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        is_armed_ = msg->armed;
    }

    // ── Timer (publishes at fixed rate) ────────────────────────────
    void timerCallback(const ros::TimerEvent &)
    {
        geometry_msgs::PoseStamped  pose_out;
        geometry_msgs::TwistStamped speed_out;
        bool publish_speed_now = false;
        bool do_publish_pose = true;

        {
            std::lock_guard<std::mutex> lock(mtx_);
            double now = ros::Time::now().toSec();

            if (state_ == State::ACTIVE)
            {
                if (now - last_odom_time_ > odom_timeout_)
                {
                    state_ = State::DROPOUT;
                    resetSanity();
                    ROS_WARN("vision_pose_bridge: stale -> DROPOUT");
                    do_publish_pose = false;
                }
            }
            else if (state_ == State::DROPOUT)
            {
                if (!is_armed_)
                {
                    state_ = State::PRE_INIT;
                    resetSanity();
                    // 5.9: reset last_odom_time_ so a quick post-rearm
                    // ACTIVE transition does not inherit the old stamp
                    // (which would instantly satisfy freshness but cause
                    // a bogus finite-diff velocity on the first sample
                    // — handled by dt > 0.5 in checkSanity, but cleaner
                    // to zero it here).
                    last_odom_time_ = 0;
                    ROS_INFO("vision_pose_bridge: disarmed -> PRE_INIT");
                }
                else
                    do_publish_pose = false;
            }

            if (!do_publish_pose)
                return;

            pose_out.header.stamp = ros::Time::now();
            pose_out.header.frame_id = pose_frame_;
            if (state_ == State::ACTIVE)
            {
                pose_out.pose = last_pose_.pose;
                speed_out = last_twist_;
                publish_speed_now = publish_speed_ && have_last_twist_;
            }
            else // PRE_INIT
            {
                pose_out.pose.position.x = 0;
                pose_out.pose.position.y = 0;
                pose_out.pose.position.z = 0;
                pose_out.pose.orientation.x = 0;
                pose_out.pose.orientation.y = 0;
                pose_out.pose.orientation.z = 0;
                pose_out.pose.orientation.w = 1;
            }
        }

        // Publish outside the lock so slow TCP flush on mavros bridge
        // doesn't stall the callback thread (5.8).
        pub_mavros_.publish(pose_out);
        pub_mirror_.publish(pose_out);
        if (publish_speed_now)
            pub_speed_.publish(speed_out);
    }

    ros::NodeHandle nh_;
    ros::Publisher  pub_mavros_;
    ros::Publisher  pub_mirror_;
    ros::Publisher  pub_speed_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_state_;
    ros::Timer      timer_;
    std::mutex      mtx_;

    State  state_;
    bool   is_armed_;
    double last_odom_time_;

    double max_accel_;
    double max_speed_;
    double odom_timeout_;
    std::string pose_frame_;
    bool   publish_speed_;

    geometry_msgs::Point prev_pos_;
    double prev_stamp_;
    bool   have_prev_pos_;
    double prev_vx_, prev_vy_, prev_vz_;
    double prev_dt_;
    bool   have_prev_vel_;
    // Timestamps of recent accel violations for windowed escalation.
    std::deque<double> viol_times_;

    geometry_msgs::PoseStamped  last_pose_;
    geometry_msgs::TwistStamped last_twist_;
    bool have_last_twist_ = false;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_pose_bridge");
    VisionPoseBridge bridge;
    ros::spin();
    return 0;
}
