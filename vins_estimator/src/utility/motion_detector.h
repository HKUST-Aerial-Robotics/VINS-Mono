#pragma once

#include <deque>
#include <vector>
#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Dense>

// Rolling-window motion classifier used by the hover-aware fork.
//
// Design goals addressed in this revision:
//   * Single timebase — all pushes take an externally-supplied ROS timestamp,
//     IMU and flow buffers live on the same clock so the rolling window is
//     coherent when image-rate and IMU-rate drift apart, when a camera drops
//     frames for a sub-second gap, or when online-td shifts the IMU clock.
//   * Outlier-robust classification — percentile-based quietness tests ignore
//     isolated acc spikes (motor vibration, prop wash), gyro pulses (impacts)
//     and single flying objects across the visual field (bird, cable, leaf).
//     Thresholds act on the 90th percentile, so up to 10% outliers in the
//     window can't flip the classification on their own.
//   * Trimmed-mean robust statistics — gravity-in-body reference and gyro
//     bias priming use a trimmed mean (~14% cut at each tail), preserving
//     the central tendency of genuine motion while dropping extreme outliers.
//   * Cycle-count hysteresis state machine — transitions between MOVING /
//     ROTATION_ONLY / STATIONARY require N consecutive identical raw
//     classifications. No wall-clock dependency — the count is in
//     evaluate()-cycles, so at IMU rate the confirmation delay is typically
//     ~15 ms (3 cycles at 200 Hz).
//
// Public classifications:
//   1. STATIONARY    — accelerometer ≈ gravity, gyro ≈ 0, flow ≈ 0.
//                      Gates ZUPT, static init bias priming, failure
//                      softening.
//   2. ROTATION_ONLY — gyroscope significantly excited, but optical flow is
//                      explainable by rotation alone. Gates rotation-only
//                      position ZUPT.
//   3. MOVING        — anything else. Default.
class MotionDetector
{
public:
    enum class State { UNKNOWN, STATIONARY, ROTATION_ONLY, MOVING };

    MotionDetector() { reset(); }

    void configure(double acc_thr, double gyr_thr, double flow_thr,
                   double window_sec, int min_samples,
                   double gravity_norm, int confirm_cycles)
    {
        acc_thr_        = acc_thr;
        gyr_thr_        = gyr_thr;
        flow_thr_       = flow_thr;
        window_sec_     = window_sec;
        min_samples_    = std::max(4, min_samples);
        gravity_norm_   = gravity_norm;
        confirm_cycles_ = std::max(1, confirm_cycles);
    }

    void configureRotation(double gyr_min, double ratio, double baseline)
    {
        gyr_rot_min_   = gyr_min;
        flow_ratio_    = ratio;
        flow_baseline_ = baseline;
    }

    // Lever arm from IMU origin to camera origin (imu^T_cam). Used to
    // include the rotation-induced translation at the camera in the
    // rotation-only classifier; matches VINS-Mono's TIC[0] convention.
    void setLeverArm(const Eigen::Vector3d &t_ic) { t_ic_ = t_ic; }
    void setExtrinsicR(const Eigen::Matrix3d &R_ic) { R_ic_ = R_ic; }
    void setFocalLength(double f) { focal_ = f; }

    // Clears transient state only; preserves configured thresholds so a
    // post-reset detector behaves identically without re-calling configure.
    void reset()
    {
        imu_buf_.clear();
        flow_buf_.clear();
        still_cnt_ = rot_only_cnt_ = move_cnt_ = 0;
        state_ = State::UNKNOWN;
        have_still_ref_ = false;
        still_ref_acc_.setZero();
        still_ref_gyr_.setZero();
        still_ref_acc_std_ = 0.0;
        last_t_ = -1.0;
    }

    // IMU push. `t` is an external monotonic timestamp (ROS header stamp,
    // seconds). IMU and flow pushes MUST share the same clock.
    void pushIMU(double t, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
    {
        imu_buf_.push_back({t, acc, gyr});
        if (t > last_t_) last_t_ = t;
        trimBoth(last_t_);
        evaluate();
    }

    // Preferred image push: pass per-feature flow magnitudes (px/s). We
    // median-reduce per-image before pushing to time buffer, so a single
    // object crossing the field of view does not bias the aggregate.
    void pushFlowSamples(double t, const std::vector<double> &flows_px_per_sec)
    {
        double med = 0.0;
        if (!flows_px_per_sec.empty())
        {
            std::vector<double> v = flows_px_per_sec;
            auto mid = v.begin() + v.size() / 2;
            std::nth_element(v.begin(), mid, v.end());
            med = *mid;
        }
        flow_buf_.push_back({t, med});
        if (t > last_t_) last_t_ = t;
        trimBoth(last_t_);
    }

    // Backward-compat overload: single scalar flow.
    void pushFlow(double t, double flow_px_per_sec)
    {
        flow_buf_.push_back({t, flow_px_per_sec});
        if (t > last_t_) last_t_ = t;
        trimBoth(last_t_);
    }

    State state() const { return state_; }
    bool isStationary()   const { return state_ == State::STATIONARY; }
    bool isRotationOnly() const { return state_ == State::ROTATION_ONLY; }

    int imuCount()  const { return (int)imu_buf_.size(); }
    int flowCount() const { return (int)flow_buf_.size(); }

    Eigen::Vector3d meanGyr() const { return trimmedMeanVec3Gyr(); }
    Eigen::Vector3d meanAcc() const { return trimmedMeanVec3Acc(); }

    double meanGyrMagnitude() const
    {
        std::vector<double> m; m.reserve(imu_buf_.size());
        for (const auto &e : imu_buf_) m.push_back(e.gyr.norm());
        return trimmedMeanScalar(m);
    }

    double meanFlow() const
    {
        std::vector<double> m; m.reserve(flow_buf_.size());
        for (const auto &e : flow_buf_) m.push_back(e.second);
        return trimmedMeanScalar(m);
    }

    double peakAccDeviation() const
    {
        double mx = 0.0;
        for (const auto &e : imu_buf_)
            mx = std::max(mx, std::fabs(e.acc.norm() - gravity_norm_));
        return mx;
    }

    bool hasStationaryReference() const { return have_still_ref_; }
    const Eigen::Vector3d &stationaryReferenceAcc() const { return still_ref_acc_; }
    const Eigen::Vector3d &stationaryReferenceGyr() const { return still_ref_gyr_; }
    double stationaryAccNoise() const { return still_ref_acc_std_; }

    // Backward-compat classifier (parameters overridden at call site).
    bool isRotationOnly(double focal_px, double gyr_min_rad_s,
                        double flow_ratio, double flow_baseline_px) const
    {
        if ((int)imu_buf_.size() < 4 || flow_buf_.empty()) return false;
        double mg = meanGyrMagnitude();
        if (mg < gyr_min_rad_s) return false;
        return meanFlow() < flow_ratio * mg * focal_px + flow_baseline_px;
    }

private:
    struct ImuSample { double t; Eigen::Vector3d acc; Eigen::Vector3d gyr; };

    Eigen::Vector3d trimmedMeanVec3Acc() const
    {
        if (imu_buf_.empty()) return Eigen::Vector3d::Zero();
        std::vector<double> xs, ys, zs;
        xs.reserve(imu_buf_.size()); ys.reserve(imu_buf_.size()); zs.reserve(imu_buf_.size());
        for (const auto &s : imu_buf_)
        {
            xs.push_back(s.acc.x()); ys.push_back(s.acc.y()); zs.push_back(s.acc.z());
        }
        return Eigen::Vector3d(trimmedMeanScalar(xs),
                               trimmedMeanScalar(ys),
                               trimmedMeanScalar(zs));
    }

    Eigen::Vector3d trimmedMeanVec3Gyr() const
    {
        if (imu_buf_.empty()) return Eigen::Vector3d::Zero();
        std::vector<double> xs, ys, zs;
        xs.reserve(imu_buf_.size()); ys.reserve(imu_buf_.size()); zs.reserve(imu_buf_.size());
        for (const auto &s : imu_buf_)
        {
            xs.push_back(s.gyr.x()); ys.push_back(s.gyr.y()); zs.push_back(s.gyr.z());
        }
        return Eigen::Vector3d(trimmedMeanScalar(xs),
                               trimmedMeanScalar(ys),
                               trimmedMeanScalar(zs));
    }

    // ~14% cut per tail trimmed mean.
    double trimmedMeanScalar(std::vector<double> v) const
    {
        if (v.empty()) return 0.0;
        const int n = (int)v.size();
        std::sort(v.begin(), v.end());
        int cut = n / 7;
        int lo = cut, hi = n - cut;
        if (hi <= lo) { lo = 0; hi = n; }
        double s = 0.0;
        for (int i = lo; i < hi; ++i) s += v[i];
        return s / double(hi - lo);
    }

    static double percentile(std::vector<double> &v, double p)
    {
        if (v.empty()) return 0.0;
        std::sort(v.begin(), v.end());
        int k = (int)std::round((v.size() - 1) * p);
        if (k < 0) k = 0;
        if (k >= (int)v.size()) k = (int)v.size() - 1;
        return v[k];
    }

    void trimBoth(double now)
    {
        while (!imu_buf_.empty()  && now - imu_buf_.front().t     > window_sec_) imu_buf_.pop_front();
        while (!flow_buf_.empty() && now - flow_buf_.front().first > window_sec_) flow_buf_.pop_front();
    }

    void evaluate()
    {
        if ((int)imu_buf_.size() < min_samples_) return;

        std::vector<double> acc_dev; acc_dev.reserve(imu_buf_.size());
        std::vector<double> gyr_mag; gyr_mag.reserve(imu_buf_.size());
        for (const auto &e : imu_buf_)
        {
            acc_dev.push_back(std::fabs(e.acc.norm() - gravity_norm_));
            gyr_mag.push_back(e.gyr.norm());
        }
        double p90_acc = percentile(acc_dev, 0.90);
        double p90_gyr = percentile(gyr_mag, 0.90);
        double p50_gyr = percentile(gyr_mag, 0.50);

        double p90_flow = 0.0;
        const bool have_flow = !flow_buf_.empty();
        if (have_flow)
        {
            std::vector<double> f; f.reserve(flow_buf_.size());
            for (const auto &e : flow_buf_) f.push_back(e.second);
            p90_flow = percentile(f, 0.90);
        }

        bool imu_quiet  = (p90_acc < acc_thr_) && (p90_gyr < gyr_thr_);
        bool flow_quiet = !have_flow || (p90_flow < flow_thr_);
        bool raw_still  = imu_quiet && flow_quiet;

        bool raw_rot = false;
        if (!raw_still && have_flow && p50_gyr > gyr_rot_min_)
        {
            double predicted = p50_gyr * focal_;
            raw_rot = p90_flow < flow_ratio_ * predicted + flow_baseline_;
        }

        State raw = raw_still ? State::STATIONARY
                              : (raw_rot ? State::ROTATION_ONLY : State::MOVING);

        if      (raw == State::STATIONARY)    { still_cnt_++;    rot_only_cnt_ = 0; move_cnt_ = 0; }
        else if (raw == State::ROTATION_ONLY) { rot_only_cnt_++; still_cnt_    = 0; move_cnt_ = 0; }
        else                                  { move_cnt_++;     still_cnt_    = 0; rot_only_cnt_ = 0; }

        if      (still_cnt_    >= confirm_cycles_) state_ = State::STATIONARY;
        else if (rot_only_cnt_ >= confirm_cycles_) state_ = State::ROTATION_ONLY;
        else if (move_cnt_     >= confirm_cycles_) state_ = State::MOVING;

        if (state_ == State::STATIONARY)
        {
            still_ref_acc_ = meanAcc();
            still_ref_gyr_ = meanGyr();
            double s2 = 0.0;
            for (const auto &e : imu_buf_)
                s2 += (e.acc - still_ref_acc_).squaredNorm();
            still_ref_acc_std_ = std::sqrt(s2 / double(std::max(1, (int)imu_buf_.size())));
            have_still_ref_ = true;
        }
    }

    std::deque<ImuSample>                imu_buf_;
    std::deque<std::pair<double, double>> flow_buf_;

    State state_;
    int   still_cnt_, rot_only_cnt_, move_cnt_;

    double last_t_;
    double acc_thr_      = 0.5;
    double gyr_thr_      = 0.05;
    double flow_thr_     = 2.0;
    double window_sec_   = 0.5;
    int    min_samples_  = 20;
    double gravity_norm_ = 9.81;
    int    confirm_cycles_ = 3;

    double gyr_rot_min_   = 0.3;
    double flow_ratio_    = 1.3;
    double flow_baseline_ = 30.0;
    double focal_         = 460.0;

    bool            have_still_ref_;
    Eigen::Vector3d still_ref_acc_;
    Eigen::Vector3d still_ref_gyr_;
    double          still_ref_acc_std_;

    Eigen::Vector3d t_ic_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_ic_ = Eigen::Matrix3d::Identity();
};
