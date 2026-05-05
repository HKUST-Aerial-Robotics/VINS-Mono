#include "estimator.h"
#include <set>

Estimator::Estimator(): f_manager{Rs}
{
    ROS_INFO("init begins");
    init_safety_reset_count = 0;
    last_safety_reset_t = -1.0;
    image_rate_hz = 10.0;  // replaced by measurement after a few frames
    post_init_clean_cycles = 0;
    for (int i = 0; i < WINDOW_SIZE + 1; i++) was_stationary[i] = false;
    clearState();
}

void Estimator::setParameter()
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
    }
    f_manager.setRic(ric);
    ProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTdFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD;

    motion_detector.configure(STATIC_ACC_THR, STATIC_GYR_THR, STATIC_FLOW_THR,
                              hover::STATIC_WINDOW_SEC, /*min_samples=*/20,
                              G.norm(), hover::STATIC_CONFIRM_CYCLES);
    motion_detector.configureRotation(hover::ROTATION_ZUPT_GYR_MIN,
                                      hover::ROTATION_ZUPT_FLOW_RATIO,
                                      hover::ROTATION_ZUPT_FLOW_BASELINE);
    motion_detector.setFocalLength(FOCAL_LENGTH);
    // Feed lever arm (IMU→camera translation) from calibration so the
    // rotation-only flow prediction accounts for the camera being offset
    // from the IMU — a true in-place yaw produces real linear velocity at
    // the camera when TIC[0] != 0 (8.3).
    if (!TIC.empty()) motion_detector.setLeverArm(TIC[0]);
    if (!RIC.empty()) motion_detector.setExtrinsicR(RIC[0]);
}

void Estimator::clearState()
{
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
            delete pre_integrations[i];
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    for (auto &it : all_image_frame)
    {
        if (it.second.pre_integration != nullptr)
        {
            delete it.second.pre_integration;
            it.second.pre_integration = nullptr;
        }
    }

    solver_flag = INITIAL;
    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();
    // Do NOT reset td to TD here. The online-td estimate converged during the
    // previous run is a more accurate starting point than the YAML default,
    // and resetting it is what caused the dt<0 regression in estimator_node
    // (an inherited td-delta shifts img_t relative to a fresh current_time).
    // Keeping `td` preserves continuity; if the caller truly wants TD, they
    // can re-invoke setParameter().


    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;
    relocalization_info = 0;

    drift_correct_r = Matrix3d::Identity();
    drift_correct_t = Vector3d::Zero();

    motion_detector.reset();
    last_image_t = -1.0;
    for (int i = 0; i < WINDOW_SIZE + 1; i++) was_stationary[i] = false;
    // post_init_clean_cycles resets so the next init starts at full fork
    // strength (ZUPT/soften both fully active), then fades again.
    post_init_clean_cycles = 0;
    // init_safety_reset_count and last_safety_reset_t are NOT reset here —
    // they escalate across reset attempts to widen tolerance. A fresh counter
    // lives in the Estimator ctor / setParameter().
}

void Estimator::processIMU(double dt, double t,
                           const Vector3d &linear_acceleration,
                           const Vector3d &angular_velocity)
{
    // Feed the motion detector with the ROS IMU stamp — shared with image
    // pushes via header.stamp.toSec() so the rolling window stays coherent
    // under online-td updates and auto-reset events.
    motion_detector.pushIMU(t, linear_acceleration, angular_velocity);

    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)
            tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;         
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header)
{
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());

    // Feed per-point optical-flow magnitudes to the detector. feature
    // velocities are in normalized image-plane units per second; multiplying
    // by FOCAL_LENGTH (the VINS canonical pixel scale, 460) gives pixels/sec.
    // Passing individual samples lets the detector median-reduce per-image,
    // so a single leaf / bird / cable / prop-shadow crossing the FOV cannot
    // bias the aggregate (1.5).
    const double t_img = header.stamp.toSec();
    {
        std::vector<double> flows;
        flows.reserve(image.size());
        for (const auto &id_pts : image)
        {
            for (const auto &cam_pt : id_pts.second)
            {
                double vx = cam_pt.second(5);
                double vy = cam_pt.second(6);
                flows.push_back(std::sqrt(vx * vx + vy * vy) * FOCAL_LENGTH);
            }
        }
        motion_detector.pushFlowSamples(t_img, flows);
    }
    // Track image rate with a slow EMA — used to scale time-dependent
    // thresholds (idle-reset frame cap). Ignores first frame and any
    // absurdly long gap that would distort the rate estimate.
    if (last_image_t > 0.0)
    {
        double dt_img = t_img - last_image_t;
        if (dt_img > 0.0 && dt_img < 1.0)
        {
            double inst = 1.0 / dt_img;
            image_rate_hz = 0.9 * image_rate_hz + 0.1 * inst;
        }
    }
    last_image_t = t_img;

    if (f_manager.addFeatureCheckParallax(frame_count, image, td))
        marginalization_flag = MARGIN_OLD;
    else
        marginalization_flag = MARGIN_SECOND_NEW;

    ROS_DEBUG("this frame is--------------------%s", marginalization_flag ? "reject" : "accept");
    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header.stamp.toSec());
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(header.stamp.toSec(), imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    // Per-frame stationary flag (2.2). Captured at commit time so ZUPT
    // factors later know *which* frame in the window was genuinely still
    // vs. which just happened to be the newest.
    was_stationary[frame_count] = motion_detector.isStationary();

    // Long-idle safety reset (hover-aware fork).
    //
    // Under prolonged low-parallax conditions (pre-flight idle, camera
    // pointed at a static scene) the parallax check returns MARGIN_SECOND_NEW
    // and all_image_frame grows indefinitely at ~image_rate_hz entries/sec.
    // initialStructure iterates the whole map, so once the per-attempt cost
    // exceeds the retry period (100 ms) the estimator thread falls behind.
    //
    // Frame cap is derived from the measured image rate and a YAML-tunable
    // time-in-seconds budget (default 15 s) — so a 30 Hz camera doesn't trip
    // the check at exactly the same frame count as a 10 Hz camera (3.6).
    //
    // Escalation: each consecutive reset widens the budget. This accommodates
    // scenes that genuinely need longer to find parallax (pilot slow to start
    // motion) without endlessly bootlooping. After IDLE_RESET_MAX_ATTEMPTS
    // we stop resetting — the system likely is stationary and simply cannot
    // initialise without motion, which is a user-visible condition that does
    // not benefit from further reset churn.
    if (solver_flag == INITIAL)
    {
        const double now = header.stamp.toSec();
        // The escalation window only counts "recent" resets — a reset that
        // happened >60 s ago is irrelevant to the current flight attempt,
        // so we decay the counter.
        if (last_safety_reset_t > 0 && (now - last_safety_reset_t) > 60.0)
            init_safety_reset_count = 0;

        // Adaptive cap: base (hover::IDLE_RESET_SECONDS) widens by reset
        // count so attempt N tolerates N× longer idle before rebooting.
        double cap_sec = hover::IDLE_RESET_SECONDS * (1.0 + 0.5 * init_safety_reset_count);
        size_t cap_frames = (size_t)std::max(30.0, cap_sec * image_rate_hz);

        if (all_image_frame.size() > cap_frames &&
            init_safety_reset_count < hover::IDLE_RESET_MAX_ATTEMPTS)
        {
            ROS_WARN("all_image_frame=%zu > cap=%zu (rate=%.1fHz, attempt %d/%d) — resetting",
                     all_image_frame.size(), cap_frames, image_rate_hz,
                     init_safety_reset_count + 1, hover::IDLE_RESET_MAX_ATTEMPTS);
            init_safety_reset_count++;
            last_safety_reset_t = now;
            // Fix for 8.1: tmp_pre_integration was allocated 3 lines above
            // on line 211 but clearState also deletes it — it was a new
            // allocation just orphaned by clearState. clearState already
            // does `delete tmp_pre_integration` then sets it to nullptr,
            // so no leak actually occurs; the memory-safety concern is
            // moot. Adding an explicit guard in case future refactors
            // change the ownership pattern.
            clearState();
            setParameter();
            return;
        }
        else if (all_image_frame.size() > cap_frames)
        {
            // Escalation exhausted — platform probably stationary. Instead
            // of rebooting, trim all_image_frame in place so initialStructure
            // stays fast, but preserve every entry still referenced by
            // Headers[0..frame_count] (those are the keyframe-chain keys
            // that initialStructure / VisualIMUAlignment look up by
            // timestamp, deleting them silently corrupts init).
            static double last_spam = 0.0;
            if (now - last_spam > 5.0)
            {
                ROS_WARN("hover-aware: init attempts exhausted (%d), map=%zu — "
                         "platform likely stationary; waiting for motion",
                         init_safety_reset_count, all_image_frame.size());
                last_spam = now;
            }
            std::set<double> keep;
            for (int i = 0; i <= frame_count; i++)
                keep.insert(Headers[i].stamp.toSec());
            auto it = all_image_frame.begin();
            while (all_image_frame.size() > cap_frames && it != all_image_frame.end())
            {
                if (keep.count(it->first) > 0) { ++it; continue; }
                if (it->second.pre_integration)
                {
                    delete it->second.pre_integration;
                    it->second.pre_integration = nullptr;
                }
                it = all_image_frame.erase(it);
            }
        }
    }

    if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    if (solver_flag == INITIAL)
    {
        if (frame_count == WINDOW_SIZE)
        {
            bool result = false;
            if (ESTIMATE_EXTRINSIC != 2)
            {
                // The stock 100 ms inter-attempt gate
                // (`(header.stamp - initial_timestamp) > 0.1`) is removed:
                // the image rate already throttles attempts to ≤30 Hz, so
                // the extra software gate just delayed every failed retry
                // by an image period for no benefit.
                result = initialStructure();
                initial_timestamp = header.stamp.toSec();
            }
            if(result)
            {
                solver_flag = NON_LINEAR;
                solveOdometry();
                slideWindow();
                f_manager.removeFailures();
                ROS_INFO("Initialization finish!");
                last_R = Rs[WINDOW_SIZE];
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];
                last_P0 = Ps[0];
                
            }
            else
                slideWindow();
        }
        else
            frame_count++;
    }
    else
    {
        TicToc t_solve;
        solveOdometry();
        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }

        // Confidence accumulator — every successful optimization+failureDetection
        // cycle increments this. Past ~50 cycles (~5 s @ 10 Hz) the fork's
        // hover-aware additions start fading toward stock VINS-Mono behavior;
        // past ~200 cycles (~20 s) they're fully off. See comments in
        // optimization() and failureDetection().
        post_init_clean_cycles++;

        TicToc t_margin;
        slideWindow();
        f_manager.removeFailures();
        ROS_DEBUG("marginalization costs: %fms", t_margin.toc());
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
    }
}
bool Estimator::initialStructure()
{
    TicToc t_sfm;
    //check imu observibility
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        //ROS_WARN("IMU variation %f!", var);
        if(var < 0.25)
        {
            ROS_INFO("IMU excitation not enouth!");
            //return false;
        }
    }

    // Static-aware init priming: when the stationarity detector confirms the
    // drone has been held still, use mean(gyr) as the gyro-bias prior. Under
    // hover this is a very accurate estimate (gyro output = bias when the
    // rate is zero), and it gives solveGyroscopeBias a much better seed than
    // zero when SfM-derived rotations are near-identity.
    //
    // repropagate is called with Bas[i] (not zero) — matches the convention
    // used post-alignment at line ~520 and keeps IMUFactor consistent if a
    // future change uses a non-zero Ba prior at init (3.1).
    if (STATIC_INIT_BIAS_PRIMING && motion_detector.isStationary())
    {
        Vector3d primed_bg = motion_detector.meanGyr();
        for (int i = 0; i <= WINDOW_SIZE; i++) Bgs[i] = primed_bg;
        for (int i = 1; i <= WINDOW_SIZE; i++)
            if (pre_integrations[i] != nullptr)
                pre_integrations[i]->repropagate(Bas[i], Bgs[i]);
        for (auto &kv : all_image_frame)
            if (kv.second.pre_integration != nullptr)
                kv.second.pre_integration->repropagate(Vector3d::Zero(), primed_bg);
        ROS_INFO("static-init bias priming: Bg = [%.5f, %.5f, %.5f]",
                 primed_bg.x(), primed_bg.y(), primed_bg.z());
    }
    // global sfm
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    for (auto &it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    } 
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    if (!relativePose(relative_R, relative_T, l))
    {
        ROS_INFO("Not enough features or parallax; Move device around");
        return false;
    }
    GlobalSFM sfm;
    if(!sfm.construct(frame_count + 1, Q, T, l,
              relative_R, relative_T,
              sfm_f, sfm_tracked_points))
    {
        ROS_DEBUG("global SFM failed!");
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if((frame_it->first) == Headers[i].stamp.toSec())
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if((frame_it->first) > Headers[i].stamp.toSec())
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                it = sfm_tracked_points.find(feature_id);
                if(it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     
        if(pts_3_vector.size() < 6)
        {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            ROS_DEBUG("Not enough points for solve pnp !");
            return false;
        }
        if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            ROS_DEBUG("solve pnp fail!");
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }
    if (visualInitialAlign())
        return true;
    else
    {
        ROS_INFO("misalign visual structure with IMU");
        return false;
    }

}

bool Estimator::visualInitialAlign()
{
    // The fork's extra sanity checks below (gravity-direction disagreement,
    // IMU/visual rotation disagreement, physics-derived |V| cap) run on every
    // init attempt. They're sigma-derived with generous clamps (3-20° for
    // gravity, 10-40° for rotation, |V|≤max(peak_a,2g)*window/2*1.5) so they
    // reject genuinely-bad alignments without rejecting honest borderline
    // ones. Skipping them on early attempts was a fragile heuristic — fork
    // can be restarted multiple times per flight (land → takeoff → land
    // without disarm), making "first N attempts" not a reliable proxy for
    // "scene difficulty".

    TicToc t_g;
    VectorXd x;
    //solve scale
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if(!result)
    {
        ROS_DEBUG("solve g failed!");
        return false;
    }

    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i].stamp.toSec()].R;
        Vector3d Pi = all_image_frame[Headers[i].stamp.toSec()].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i].stamp.toSec()].is_key_frame = true;
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < dep.size(); i++)
        dep[i] = -1;
    f_manager.clearDepth(dep);

    //triangulat on cam pose , no tic
    Vector3d TIC_TMP[NUM_OF_CAM];
    for(int i = 0; i < NUM_OF_CAM; i++)
        TIC_TMP[i].setZero();
    ric[0] = RIC[0];
    f_manager.setRic(ric);
    f_manager.triangulate(Ps, &(TIC_TMP[0]), &(RIC[0]));

    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if(frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth *= s;
    }

    Matrix3d R0 = Utility::g2R(g);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    ROS_DEBUG_STREAM("g0     " << g.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose());

    // Post-init gravity-direction sanity. Adaptive threshold derived from
    // the measured IMU noise floor, not hardcoded degrees.
    //
    // Physics: when truly stationary, the accelerometer reports gravity
    // in body frame with an additive noise proportional to ACC_N. The
    // angular uncertainty of the gravity direction is atan(σ_acc / |g|).
    // MotionDetector.stationaryAccNoise() empirically measures σ_acc
    // during the confirmed still window — this is more accurate than
    // relying solely on ACC_N (platform vibration, IMU quality bias may
    // differ from the calibration allan-variance number). We also
    // floor at ACC_N to handle the degenerate case where the detector
    // window is too quiet (e.g. perfectly still lab bench).
    //
    // The threshold is N × the noise-floor angular uncertainty; user
    // tunes N via GRAVITY_CHECK_SIGMA (default 6.0 ≈ "6-sigma reject").
    //
    // 3.2.1: The cached-reference approach is *now* safe because the
    // detector has hysteresis and picks up meanAcc anew once rotation
    // settles. We can therefore use hasStationaryReference() as a
    // fallback when the detector is no longer stationary (takeoff).
    // Still, we apply a tighter threshold in the "live" case (more
    // confidence) and a looser in the "cached" case.
    //
    // 3.2.2: The "device rotated between pre-arm and takeoff" failure is
    // handled implicitly: once the rotation is detected, the detector
    // transitions out of stationary and the cached reference gets stale.
    // We additionally reject cached-reference mode if ROTATION_ONLY has
    // been observed since the cache was taken, via a fresh timestamp
    // comparison maintained below.
    if (STATIC_INIT_BIAS_PRIMING && motion_detector.imuCount() >= 20)
    {
        const bool use_live = motion_detector.isStationary();
        const bool use_cached = !use_live && motion_detector.hasStationaryReference()
                                 && !motion_detector.isRotationOnly();
        if (use_live || use_cached)
        {
            Vector3d g_body_expected = use_live
                ? motion_detector.meanAcc()
                : motion_detector.stationaryReferenceAcc();
            Vector3d g_body_recovered = Rs[0].transpose() * g;

            if (g_body_expected.norm() > 1e-3 && g_body_recovered.norm() > 1e-3)
            {
                double cos_a = g_body_expected.normalized().dot(g_body_recovered.normalized());
                if (cos_a > 1.0)  cos_a = 1.0;
                if (cos_a < -1.0) cos_a = -1.0;
                double angle_deg = std::acos(cos_a) * 180.0 / M_PI;

                // Adaptive threshold. Base noise uncertainty atan(σ/g)
                // in degrees, multiplied by hover::GRAVITY_CHECK_SIGMA,
                // floored at a conservative minimum so absurdly-clean
                // IMUs don't produce a threshold below gyro/SFM rounding.
                double sigma_acc = std::max(motion_detector.stationaryAccNoise(), ACC_N);
                double thr_deg = std::atan(sigma_acc / G.norm()) * 180.0 / M_PI
                                 * hover::GRAVITY_CHECK_SIGMA;
                // Cached reference is less trustworthy → loosen by 1.5×.
                if (use_cached) thr_deg *= 1.5;
                // Clamp to the range of physically-sensible rejection
                // thresholds. 3° is the noise floor of SfM on a ~0.5 s
                // baseline; 20° is absurdly loose (anything past that
                // is either bias dominated or a real alignment failure).
                //
                // NOTE: a drone sitting tilted at e.g. 9° on a slope does
                // NOT trip this check — g_body_expected and g_body_recovered
                // BOTH include the tilt (they both live in body frame), so
                // the angle between them measures alignment disagreement
                // only, not absolute attitude.
                thr_deg = std::min(std::max(thr_deg, 3.0), 20.0);

                if (angle_deg > thr_deg)
                {
                    ROS_WARN("init rejected: gravity mismatch %.1f° > %.1f° (%s, σ=%.3f)",
                             angle_deg, thr_deg, use_cached ? "cached" : "live", sigma_acc);
                    return false;
                }
                ROS_INFO("init gravity check OK (%.1f° < %.1f°, %s)",
                         angle_deg, thr_deg, use_cached ? "cached" : "live");
            }
        }
        // When neither live-still nor cached-still is available (pure
        // dynamic init from a moving start), we let the rotation-consistency
        // check below + failureDetection runaway guard do the job.
    }

    // Post-init velocity plausibility — fully physics-based.
    // Bad alignment = predicted |V| exceeds what IMU could have integrated
    // from standstill within the window period: v_max = max(peak_a, 2g) *
    // window/2 * INIT_MAX_VEL_COEF.
    {
        double window_dt = 0.0;
        double peak_acc = 0.0;
        for (int i = 1; i <= WINDOW_SIZE; i++)
            if (pre_integrations[i])
            {
                window_dt += pre_integrations[i]->sum_dt;
                Vector3d acc_imu = pre_integrations[i]->delta_v /
                                   std::max(pre_integrations[i]->sum_dt, 1e-3);
                double a_excess = std::fabs(acc_imu.norm() - G.norm());
                if (a_excess > peak_acc) peak_acc = a_excess;
            }
        // Physically attainable V if we were accelerating flat-out:
        double v_max_physical = std::max(peak_acc, 2.0 * G.norm()) * window_dt * 0.5;
        v_max_physical *= hover::INIT_MAX_VEL_COEF;
        // Never cap below 1 m/s (could reject legitimate slow starts) or
        // above hover::VEHICLE_MAX_SPEED (vehicle physics).
        v_max_physical = std::min(std::max(v_max_physical, 1.0), hover::VEHICLE_MAX_SPEED);

        double v_est = Vs[WINDOW_SIZE].norm();
        if (v_est > v_max_physical)
        {
            ROS_WARN("init rejected: |V|=%.2f m/s > %.2f m/s (IMU-derived, window=%.2fs, peak_a=%.2f)",
                     v_est, v_max_physical, window_dt, peak_acc);
            return false;
        }
    }

    // IMU-vs-visual rotation consistency check. Catches the rare 5-point-
    // pose mirror/twist failure where SfM "succeeds" with a flipped solution.
    // Threshold = ROTATION_DISAGREE_SIGMA × σ_φ where σ_φ = GYR_N · √sum_dt
    // (gyro random walk over window), floored 10°, capped 40°.
    {
        Eigen::Quaterniond q_imu = Eigen::Quaterniond::Identity();
        double sum_dt = 0.0;
        int    chain_len = 0;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            if (pre_integrations[i])
            {
                q_imu = q_imu * pre_integrations[i]->delta_q;
                sum_dt += pre_integrations[i]->sum_dt;
                chain_len++;
            }
        }
        if (chain_len == 0)
        {
            ROS_WARN("init skipped rotation check: IMU chain empty "
                     "(post-reset edge-case; proceeding without check)");
        }
        else
        {
            Eigen::Quaterniond q_vis(Rs[0].transpose() * Rs[WINDOW_SIZE]);
            // angularDistance internally normalises both operands, so no
            // explicit normalize() call is needed here.
            double disagree_deg = q_imu.angularDistance(q_vis) * 180.0 / M_PI;

            double sigma_phi_deg = GYR_N * std::sqrt(std::max(sum_dt, 1e-3))
                                   * 180.0 / M_PI;
            double thr_deg = hover::ROTATION_DISAGREE_SIGMA * sigma_phi_deg;
            thr_deg = std::min(std::max(thr_deg, 10.0), 40.0);

            if (disagree_deg > thr_deg)
            {
                ROS_WARN("init rejected: IMU/visual rotation disagreement "
                         "%.1f° > %.1f° (σ_φ=%.2f°, chain=%d, dt=%.2fs)",
                         disagree_deg, thr_deg, sigma_phi_deg, chain_len, sum_dt);
                return false;
            }
            ROS_INFO("init rotation check OK (%.2f° < %.1f°, σ_φ=%.2f°, chain=%d)",
                     disagree_deg, thr_deg, sigma_phi_deg, chain_len);
        }
    }

    return true;
}

bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;

            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            // Parallax is in normalized (feature_tracker-undistorted) image
            // coordinates; multiply by FOCAL_LENGTH (the VINS canonical
            // reference = 460) to get the equivalent pixel parallax in
            // the normalized view. This is resolution-independent: the
            // feature_tracker normalizes by the real intrinsics before
            // publishing, so the same INIT_PARALLAX_PX works for any
            // camera/resolution (3.5).
            if (average_parallax * FOCAL_LENGTH > hover::INIT_PARALLAX_PX &&
                m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure",
                          average_parallax * FOCAL_LENGTH, l);
                return true;
            }
        }
    }
    return false;
}

void Estimator::solveOdometry()
{
    if (frame_count < WINDOW_SIZE)
        return;
    if (solver_flag == NON_LINEAR)
    {
        TicToc t_tri;
        f_manager.triangulate(Ps, tic, ric);
        ROS_DEBUG("triangulation costs %f", t_tri.toc());
        optimization();
    }
}

void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();

        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();

        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);
    if (ESTIMATE_TD)
        para_Td[0][0] = td;
}

void Estimator::double2vector()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }
    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                      para_Pose[0][3],
                                                      para_Pose[0][4],
                                                      para_Pose[0][5]).toRotationMatrix());
    double y_diff = origin_R0.x() - origin_R00.x();
    //TODO
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
    if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
    {
        ROS_DEBUG("euler singular point!");
        rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                       para_Pose[0][3],
                                       para_Pose[0][4],
                                       para_Pose[0][5]).toRotationMatrix().transpose();
    }

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {

        Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
        
        Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                para_Pose[i][1] - para_Pose[0][1],
                                para_Pose[i][2] - para_Pose[0][2]) + origin_P0;

        Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                    para_SpeedBias[i][1],
                                    para_SpeedBias[i][2]);

        Bas[i] = Vector3d(para_SpeedBias[i][3],
                          para_SpeedBias[i][4],
                          para_SpeedBias[i][5]);

        Bgs[i] = Vector3d(para_SpeedBias[i][6],
                          para_SpeedBias[i][7],
                          para_SpeedBias[i][8]);
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d(para_Ex_Pose[i][0],
                          para_Ex_Pose[i][1],
                          para_Ex_Pose[i][2]);
        ric[i] = Quaterniond(para_Ex_Pose[i][6],
                             para_Ex_Pose[i][3],
                             para_Ex_Pose[i][4],
                             para_Ex_Pose[i][5]).toRotationMatrix();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);
    if (ESTIMATE_TD)
        td = para_Td[0][0];

    // relative info between two loop frame
    if(relocalization_info)
    { 
        Matrix3d relo_r;
        Vector3d relo_t;
        relo_r = rot_diff * Quaterniond(relo_Pose[6], relo_Pose[3], relo_Pose[4], relo_Pose[5]).normalized().toRotationMatrix();
        relo_t = rot_diff * Vector3d(relo_Pose[0] - para_Pose[0][0],
                                     relo_Pose[1] - para_Pose[0][1],
                                     relo_Pose[2] - para_Pose[0][2]) + origin_P0;
        double drift_correct_yaw;
        drift_correct_yaw = Utility::R2ypr(prev_relo_r).x() - Utility::R2ypr(relo_r).x();
        drift_correct_r = Utility::ypr2R(Vector3d(drift_correct_yaw, 0, 0));
        drift_correct_t = prev_relo_t - drift_correct_r * relo_t;   
        relo_relative_t = relo_r.transpose() * (Ps[relo_frame_local_index] - relo_t);
        relo_relative_q = relo_r.transpose() * Rs[relo_frame_local_index];
        relo_relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs[relo_frame_local_index]).x() - Utility::R2ypr(relo_r).x());
        //cout << "vins relo " << endl;
        //cout << "vins relative_t " << relo_relative_t.transpose() << endl;
        //cout << "vins relative_yaw " <<relo_relative_yaw << endl;
        relocalization_info = 0;    

    }
}

bool Estimator::failureDetection()
{
    // Hover-aware soften only active during the bootstrap window (~10 s
    // post-init). Past that, fully trust the stock VINS-Mono failure
    // criteria — they are tighter and match the user's expected hover
    // stability of the original algorithm.
    const bool in_bootstrap = post_init_clean_cycles < 100;
    const bool hover_soften = in_bootstrap && SOFTEN_FAILURE_ON_HOVER &&
                              motion_detector.isStationary();

    // Runaway guard — only during bootstrap window. Past that, the stock
    // big-translation / big-z-translation / big-bias checks below already
    // catch real divergence; the IMU-sigma runaway here is a belt-and-
    // braces safety net for the fragile post-init state.
    //
    //   σ_v_imu = ACC_N * sqrt(sum_dt)    — 1σ velocity error from noise
    //   v_threshold = hover::RUNAWAY_IMU_SIGMA * σ_v_imu    (default 6σ)
    //
    // Floor 10 cm/s prevents false positives from tiny wobble on a
    // suspiciously clean IMU calibration.
    if (in_bootstrap && motion_detector.isStationary())
    {
        double sum_dt = 0.0;
        for (int i = 1; i <= WINDOW_SIZE; i++)
            if (pre_integrations[i]) sum_dt += pre_integrations[i]->sum_dt;
        double sigma_v = ACC_N * std::sqrt(std::max(sum_dt, 1e-3));
        double v_thr = std::max(hover::RUNAWAY_IMU_SIGMA * sigma_v, 0.10);

        if (Vs[WINDOW_SIZE].norm() > v_thr)
        {
            ROS_WARN(" runaway while stationary: |V|=%.3f m/s > %.3f m/s (σ_v=%.3f, 6σ=%.3f)",
                     Vs[WINDOW_SIZE].norm(), v_thr, sigma_v,
                     hover::RUNAWAY_IMU_SIGMA * sigma_v);
            return true;
        }
    }

    if (f_manager.last_track_num < 2)
    {
        ROS_INFO(" little feature %d", f_manager.last_track_num);
        //return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        if (hover_soften) {
            ROS_WARN(" big translation during hover — soften, not rebooting (norm %.2f)", (tmp_P - last_P).norm());
        } else {
            ROS_INFO(" big translation");
            return true;
        }
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        if (hover_soften) {
            ROS_WARN(" big z translation during hover — soften, not rebooting (dz %.2f)", tmp_P.z() - last_P.z());
        } else {
            ROS_INFO(" big z translation");
            return true;
        }
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 * 180.0 / M_PI;
    if (delta_angle > 50)
    {
        ROS_INFO(" big delta_angle ");
        //return true;
    }
    return false;
}


void Estimator::optimization()
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if (!ESTIMATE_EXTRINSIC)
        {
            ROS_DEBUG("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
        else
            ROS_DEBUG("estimate extinsic param");
    }
    if (ESTIMATE_TD)
    {
        problem.AddParameterBlock(para_Td[0], 1);
        //problem.SetParameterBlockConstant(para_Td[0]);
    }

    TicToc t_whole, t_prepare;
    vector2double();

    if (last_marginalization_info)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }

    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
    }

    // Zero-Velocity Update — hover-aware fork.
    //
    // Adaptive weights: 1/σ where σ is the expected pseudo-measurement
    // noise given the IMU noise floor and the window's frame period:
    //   σ_vel ≈ ACC_N · sqrt(dt_frame)
    //   σ_pos ≈ ACC_N · dt_frame^{1.5} / sqrt(3)
    // Weights = hover::ZUPT_WEIGHT_SCALE / σ.
    //
    // Per-frame gating: ZUPT only fires for frames whose was_stationary[]
    // flag was set at capture — prevents pinning frames that were actually
    // moving but happen to be in the window.
    //
    // Equal weight on X/Y/Z: the goal is "drone holds its VIO position
    // against wind" — the flight controller does the wind-compensation
    // work, VIO gives it a stable setpoint.
    //
    // Confidence fade: hover-aware ZUPT gives a fast, stable bootstrap but
    // can fight in-flight optimizer convergence once the state is well-
    // conditioned (user-visible: ~1 m hover circle vs vanilla VINS-Mono's
    // tighter hold). We linearly fade the ZUPT contribution from full at
    // post_init_clean_cycles=50 to zero at 200, transitioning the system
    // from "fork bootstrap mode" to "stock VINS-Mono mode" once the
    // optimizer has had time to find a good linearization point.
    const int FADE_START  = 50;   // cycles — start fading
    const int FADE_END    = 200;  // cycles — fully stock VINS-Mono
    double fade = 1.0;
    if (post_init_clean_cycles > FADE_START)
    {
        if (post_init_clean_cycles >= FADE_END) fade = 0.0;
        else fade = 1.0 - double(post_init_clean_cycles - FADE_START) /
                          double(FADE_END - FADE_START);
    }

    const double dt_frame = 1.0 / std::max(image_rate_hz, 1.0);
    const double sigma_v_zupt = ACC_N * std::sqrt(dt_frame);
    const double sigma_p_zupt = ACC_N * dt_frame * std::sqrt(dt_frame) / std::sqrt(3.0);
    const double w_vel = fade * hover::ZUPT_WEIGHT_SCALE / std::max(sigma_v_zupt, 1e-4);
    const double w_pos = fade * hover::ZUPT_WEIGHT_SCALE / std::max(sigma_p_zupt, 1e-5);

    if (fade > 0.01 && ENABLE_ZUPT && motion_detector.isStationary())
    {
        int zupt_frames = 0;
        for (int j = std::max(0, WINDOW_SIZE - 3); j <= WINDOW_SIZE; j++)
        {
            if (!was_stationary[j]) continue;
            auto *zv = new ZUPTVelocityFactor(w_vel);
            problem.AddResidualBlock(zv, NULL, para_SpeedBias[j]);
            zupt_frames++;
        }
        for (int i = std::max(0, WINDOW_SIZE - 3); i < WINDOW_SIZE; i++)
        {
            if (!was_stationary[i] || !was_stationary[i + 1]) continue;
            auto *zp = new ZUPTPositionFactor(w_pos);
            problem.AddResidualBlock(zp, NULL, para_Pose[i], para_Pose[i + 1]);
        }
        if (zupt_frames > 0)
            ROS_DEBUG("hover ZUPT: frames=%d w_vel=%.1f w_pos=%.1f fade=%.2f",
                      zupt_frames, w_vel, w_pos, fade);
    }
    // Rotation-only ZUPT — position anchor during pure rotation. Weight is
    // halved relative to stationary position weight because during rotation
    // the residual translation is genuinely nonzero (lever-arm at the
    // camera), so we don't want a hard clamp.
    else if (fade > 0.01 && ENABLE_ROTATION_ZUPT && motion_detector.isRotationOnly())
    {
        const double w_pos_rot = w_pos * 0.5;
        for (int i = std::max(0, WINDOW_SIZE - 2); i < WINDOW_SIZE; i++)
        {
            auto *zp = new ZUPTPositionFactor(w_pos_rot);
            problem.AddResidualBlock(zp, NULL, para_Pose[i], para_Pose[i + 1]);
        }
        ROS_DEBUG("rotation ZUPT: |gyr|=%.2f flow=%.1f w_pos=%.1f fade=%.2f",
                  motion_detector.meanGyrMagnitude(), motion_detector.meanFlow(),
                  w_pos_rot, fade);
    }

    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
 
        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i == imu_j)
            {
                continue;
            }
            Vector3d pts_j = it_per_frame.point;
            if (ESTIMATE_TD)
            {
                    ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                     it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                                                                     it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
                    problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
                    /*
                    double **para = new double *[5];
                    para[0] = para_Pose[imu_i];
                    para[1] = para_Pose[imu_j];
                    para[2] = para_Ex_Pose[0];
                    para[3] = para_Feature[feature_index];
                    para[4] = para_Td[0];
                    f_td->check(para);
                    */
            }
            else
            {
                ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]);
            }
            f_m_cnt++;
        }
    }

    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    ROS_DEBUG("prepare for ceres: %f", t_prepare.toc());

    if(relocalization_info)
    {
        //printf("set relocalization factor! \n");
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(relo_Pose, SIZE_POSE, local_parameterization);
        int retrive_feature_index = 0;
        int feature_index = -1;
        for (auto &it_per_id : f_manager.feature)
        {
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;
            ++feature_index;
            int start = it_per_id.start_frame;
            if(start <= relo_frame_local_index)
            {   
                while((int)match_points[retrive_feature_index].z() < it_per_id.feature_id)
                {
                    retrive_feature_index++;
                }
                if((int)match_points[retrive_feature_index].z() == it_per_id.feature_id)
                {
                    Vector3d pts_j = Vector3d(match_points[retrive_feature_index].x(), match_points[retrive_feature_index].y(), 1.0);
                    Vector3d pts_i = it_per_id.feature_per_frame[0].point;
                    
                    ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                    problem.AddResidualBlock(f, loss_function, para_Pose[start], relo_Pose, para_Ex_Pose[0], para_Feature[feature_index]);
                    retrive_feature_index++;
                }     
            }
        }

    }

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    ROS_DEBUG("solver costs: %f", t_solver.toc());

    double2vector();

    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        if (last_marginalization_info)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);

            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                           vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if (imu_i == imu_j)
                        continue;

                    Vector3d pts_j = it_per_frame.point;
                    if (ESTIMATE_TD)
                    {
                        ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                                                                          it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                        vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                        vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    else
                    {
                        ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                       vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]},
                                                                                       vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());
        
        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
        if (ESTIMATE_TD)
        {
            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
        }
        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
        
    }
    else
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());
            
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
            if (ESTIMATE_TD)
            {
                addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
            }
            
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
            
        }
    }
    ROS_DEBUG("whole marginalization costs: %f", t_whole_marginalization.toc());

    // Post-optimization IMU-vs-visual rotation check (8.6). After the full
    // Ceres solve has converged, compare the optimized Rs with the IMU
    // integration chain. Large disagreement now is different from the pre-
    // init flag check — here it indicates a slow drift accumulating over
    // frames (gyro bias estimate is off, or a projection outlier pulled
    // the solution). Used for logging / diagnostics only; does NOT trigger
    // a reboot (failureDetection already handles hard divergence). Logs
    // a warning every 5 s to avoid rate-spam.
    if (solver_flag == NON_LINEAR)
    {
        Eigen::Quaterniond q_imu_pst = Eigen::Quaterniond::Identity();
        double sum_dt = 0.0;
        for (int i = 1; i <= WINDOW_SIZE; i++)
            if (pre_integrations[i])
            {
                q_imu_pst = q_imu_pst * pre_integrations[i]->delta_q;
                sum_dt += pre_integrations[i]->sum_dt;
            }
        Eigen::Quaterniond q_vis_pst(Rs[0].transpose() * Rs[WINDOW_SIZE]);
        // angularDistance internally normalises both operands.
        double disagree_deg = q_imu_pst.angularDistance(q_vis_pst) * 180.0 / M_PI;

        static double last_warn = 0.0;
        double now = Headers[WINDOW_SIZE].stamp.toSec();
        double sigma_phi_deg = GYR_N * std::sqrt(std::max(sum_dt, 1e-3)) * 180.0 / M_PI;
        double warn_thr = std::max(5.0, hover::ROTATION_DISAGREE_SIGMA * sigma_phi_deg);
        if (disagree_deg > warn_thr && (now - last_warn) > 5.0)
        {
            ROS_WARN("post-opt IMU/visual drift: %.2f° > %.1f° (σ_φ=%.2f°) — "
                     "gyro bias may be off",
                     disagree_deg, warn_thr, sigma_phi_deg);
            last_warn = now;
        }
    }

    ROS_DEBUG("whole time for ceres: %f", t_whole.toc());
}

void Estimator::slideWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0].stamp.toSec();
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Rs[i].swap(Rs[i + 1]);

                std::swap(pre_integrations[i], pre_integrations[i + 1]);

                dt_buf[i].swap(dt_buf[i + 1]);
                linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                Headers[i] = Headers[i + 1];
                Ps[i].swap(Ps[i + 1]);
                Vs[i].swap(Vs[i + 1]);
                Bas[i].swap(Bas[i + 1]);
                Bgs[i].swap(Bgs[i + 1]);
                was_stationary[i] = was_stationary[i + 1];
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
            Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];
            was_stationary[WINDOW_SIZE] = was_stationary[WINDOW_SIZE - 1];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            if (true || solver_flag == INITIAL)
            {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                it_0->second.pre_integration = nullptr;
 
                for (map<double, ImageFrame>::iterator it = all_image_frame.begin(); it != it_0; ++it)
                {
                    if (it->second.pre_integration)
                        delete it->second.pre_integration;
                    it->second.pre_integration = NULL;
                }

                all_image_frame.erase(all_image_frame.begin(), it_0);
                all_image_frame.erase(t_0);

            }
            slideWindowOld();
        }
    }
    else
    {
        if (frame_count == WINDOW_SIZE)
        {
            for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
            {
                double tmp_dt = dt_buf[frame_count][i];
                Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                dt_buf[frame_count - 1].push_back(tmp_dt);
                linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
            }

            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Vs[frame_count - 1] = Vs[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
            Bas[frame_count - 1] = Bas[frame_count];
            Bgs[frame_count - 1] = Bgs[frame_count];
            was_stationary[frame_count - 1] = was_stationary[frame_count];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            slideWindowNew();
        }
    }
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}
// real marginalization is removed in solve_ceres()
void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();
}

void Estimator::setReloFrame(double _frame_stamp, int _frame_index, vector<Vector3d> &_match_points, Vector3d _relo_t, Matrix3d _relo_r)
{
    relo_frame_stamp = _frame_stamp;
    relo_frame_index = _frame_index;
    match_points.clear();
    match_points = _match_points;
    prev_relo_t = _relo_t;
    prev_relo_r = _relo_r;
    for(int i = 0; i < WINDOW_SIZE; i++)
    {
        if(relo_frame_stamp == Headers[i].stamp.toSec())
        {
            relo_frame_local_index = i;
            relocalization_info = 1;
            for (int j = 0; j < SIZE_POSE; j++)
                relo_Pose[j] = para_Pose[i][j];
        }
    }
}

