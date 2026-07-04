#include "feature_tracker.h"
#include "rclcpp/rclcpp.hpp"
#include "tracking_safety.h"

int FeatureTracker::n_id = 0;

bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    const int safe_size = std::min<int>(v.size(), status.size());
    for (int i = 0; i < safe_size; i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    const int safe_size = std::min<int>(v.size(), status.size());
    for (int i = 0; i < safe_size; i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}


FeatureTracker::FeatureTracker()
{
    clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    current_velocity = 0.0;
    adaptive_max_cnt = MAX_CNT;
}

void FeatureTracker::setMask()
{
    if (FISHEYE && !fisheye_mask.empty())
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    const size_t aligned_count = tracking_safety::commonSize(
        {forw_pts.size(), track_cnt.size(), ids.size()});
    if (aligned_count != forw_pts.size() || aligned_count != track_cnt.size() ||
        aligned_count != ids.size())
    {
        RCLCPP_WARN(rclcpp::get_logger("feature_tracker"),
                    "[MASK] vector mismatch: points=%zu tracks=%zu ids=%zu; using %zu",
                    forw_pts.size(), track_cnt.size(), ids.size(), aligned_count);
    }
    for (size_t i = 0; i < aligned_count; i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        const int x = cvRound(it.second.first.x);
        const int y = cvRound(it.second.first.y);
        if (!std::isfinite(it.second.first.x) || !std::isfinite(it.second.first.y) ||
            !tracking_safety::pixelIsInside(x, y, mask.cols, mask.rows))
        {
            RCLCPP_WARN(rclcpp::get_logger("feature_tracker"),
                        "[MASK] rejected invalid feature coordinate x=%.3f y=%.3f",
                        it.second.first.x, it.second.first.y);
            continue;
        }
        if (mask.at<uchar>(y, x) == 255)
        {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}

void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)
{
    cv::Mat img;
    TicToc t_r;
    cur_time = _cur_time;
    
    // Always initialize with default feature count
    adaptive_max_cnt = MAX_CNT;

    if (EQUALIZE)
    {
        TicToc t_c;
        clahe->apply(_img, img);
        RCLCPP_DEBUG(rclcpp::get_logger("feature_tracker"), "CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img;

    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }

    forw_pts.clear();

    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;

        if (USE_ADVANCED_FLOW)
        {
            // Advanced LK: stricter termination and optional forward-backward consistency
            cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3,
                                     cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 40, 0.001));

            if (USE_BIDIRECTIONAL_FLOW)
            {
                vector<uchar> reverse_status;
                vector<cv::Point2f> reverse_pts = cur_pts;
                cv::calcOpticalFlowPyrLK(forw_img, cur_img, forw_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3,
                                         cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 40, 0.001));

                const size_t flow_count = tracking_safety::commonSize(
                    {status.size(), reverse_status.size(), cur_pts.size(), reverse_pts.size()});
                if (flow_count != status.size())
                {
                    RCLCPP_WARN(
                        rclcpp::get_logger("feature_tracker"),
                        "[FLOW] vector mismatch: forward=%zu reverse=%zu cur=%zu reverse_pts=%zu; rejecting tail",
                        status.size(), reverse_status.size(), cur_pts.size(), reverse_pts.size());
                    std::fill(status.begin() + flow_count, status.end(), 0);
                }
                for (size_t i = 0; i < flow_count; i++)
                {
                    if (status[i] && reverse_status[i])
                    {
                        cv::Point2f dist_pt = cur_pts[i] - reverse_pts[i];
                        float dist = (dist_pt.x * dist_pt.x + dist_pt.y * dist_pt.y);
                        if (dist > 0.5) // 0.5 pixel squared error threshold (approx 0.7 px distance)
                        {
                            status[i] = 0;
                        }
                    }
                    else
                    {
                        status[i] = 0;
                    }
                }
            }
        }
        else
        {
            // Basic LK flow (original behavior) without advanced termination or backward check
            cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);
        }

        for (int i = 0; i < int(forw_pts.size()); i++)
            if (status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status);
        
        // Estimate velocity based on optical flow magnitude (adaptive_max_cnt already initialized above)
        if (ENABLE_VELOCITY_CHECK && cur_pts.size() > 10)
        {
            double total_flow = 0.0;
            int valid_count = 0;
            for (size_t i = 0; i < cur_pts.size(); i++)
            {
                cv::Point2f diff = forw_pts[i] - cur_pts[i];
                double flow_mag = sqrt(diff.x * diff.x + diff.y * diff.y);
                if (flow_mag < 100.0)  // sanity check: ignore outliers
                {
                    total_flow += flow_mag;
                    valid_count++;
                }
            }
            
            if (valid_count > 0)
            {
                double avg_flow = total_flow / valid_count;
                // Rough estimate: flow (pixels/frame) * focal_length_scale → velocity
                // Assuming ~30fps and depth ~3m: pixel_flow * 0.01 ≈ velocity
                double dt = cur_time - prev_time;
                current_velocity = (dt > 0.001) ? (avg_flow / (FOCAL_LENGTH * dt)) * 3.0 : 0.0;
                
                // Adaptive feature count based on velocity
                if (current_velocity > MAX_VELOCITY_THRESHOLD)
                {
                    adaptive_max_cnt = MAX_CNT + VELOCITY_BOOST_FEATURES;
                    RCLCPP_WARN_SKIPFIRST_THROTTLE(rclcpp::get_logger("feature_tracker"), *(rclcpp::Clock::make_shared()), 2000, "High velocity detected: %.2f m/s, boosting features to %d",
                                      current_velocity, adaptive_max_cnt);
                }
            }
        }
        
        RCLCPP_DEBUG(rclcpp::get_logger("feature_tracker"), "temporal optical flow costs: %fms", t_o.toc());
    }

    for (auto &n : track_cnt)
        n++;

    if (PUB_THIS_FRAME)
    {
        rejectWithF();
        RCLCPP_DEBUG(rclcpp::get_logger("feature_tracker"), "set mask begins");
        TicToc t_m;
        setMask();
        RCLCPP_DEBUG(rclcpp::get_logger("feature_tracker"), "set mask costs %fms", t_m.toc());

        RCLCPP_DEBUG(rclcpp::get_logger("feature_tracker"), "detect feature begins");
        TicToc t_t;
        int n_max_cnt = adaptive_max_cnt - static_cast<int>(forw_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if(mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;

            n_pts.clear();

            if (USE_ADVANCED_FLOW)
            {
                // AGAST + grid selection (current enhanced method)
                vector<cv::KeyPoint> keypoints;
                cv::AGAST(forw_img, keypoints, FAST_THRESHOLD, true);

                int grid_size = MIN_DIST; 
                int grid_cols = COL / grid_size + 1;
                std::unordered_map<int, cv::KeyPoint> grid_best_features;

                for (const auto& kp : keypoints) {
                    const int x = cvRound(kp.pt.x);
                    const int y = cvRound(kp.pt.y);
                    if (!tracking_safety::pixelIsInside(x, y, mask.cols, mask.rows))
                        continue;
                    if (mask.at<uchar>(y, x) == 0) continue;
                    int r = y / grid_size;
                    int c = x / grid_size;
                    int idx = r * grid_cols + c;
                    auto it = grid_best_features.find(idx);
                    if (it == grid_best_features.end() || kp.response > it->second.response) {
                        grid_best_features[idx] = kp;
                    }
                }

                for (const auto& grid_cell : grid_best_features) {
                    n_pts.push_back(grid_cell.second.pt);
                    if (int(n_pts.size()) >= n_max_cnt) break;
                }

                if (!n_pts.empty())
                {
                    // Subpixel refine strongest points only
                    int refine_count = std::min(50, (int)n_pts.size());
                    if (refine_count > 0) {
                        vector<cv::Point2f> pts_to_refine(n_pts.begin(), n_pts.begin() + refine_count);
                        cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001);
                        cv::cornerSubPix(forw_img, pts_to_refine, cv::Size(5, 5), cv::Size(-1, -1), criteria);
                        std::copy(pts_to_refine.begin(), pts_to_refine.end(), n_pts.begin());
                    }
                }
            }
            else
            {
                // Original VINS-Fusion style: Shi-Tomasi (goodFeaturesToTrack)
                cv::goodFeaturesToTrack(
                    forw_img, n_pts, n_max_cnt, GFTT_QUALITY_LEVEL, MIN_DIST, mask);
            }
        }
        else
        {
            n_pts.clear();
        }

        RCLCPP_DEBUG(rclcpp::get_logger("feature_tracker"), "detect feature costs: %fms", t_t.toc());

        RCLCPP_DEBUG(rclcpp::get_logger("feature_tracker"), "add feature begins");
        TicToc t_a;
        addPoints();
        RCLCPP_DEBUG(rclcpp::get_logger("feature_tracker"), "selectFeature costs: %fms", t_a.toc());
    }
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    cur_img = forw_img;
    cur_pts = forw_pts;
    undistortedPoints();
    prev_time = cur_time;
}

void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("feature_tracker"), "FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        if (status.size() != forw_pts.size())
        {
            RCLCPP_WARN(rclcpp::get_logger("feature_tracker"),
                        "[RANSAC] invalid status size=%zu for points=%zu; keeping current tracks",
                        status.size(), forw_pts.size());
            return;
        }
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        RCLCPP_DEBUG(rclcpp::get_logger("feature_tracker"), "FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        RCLCPP_DEBUG(rclcpp::get_logger("feature_tracker"), "FM ransac costs: %fms", t_f.toc());
    }
}

bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    RCLCPP_INFO(rclcpp::get_logger("feature_tracker"), "reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}

void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }
    // caculate points velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        if (!tracking_safety::timestampStepIsValid(prev_time, cur_time))
        {
            RCLCPP_WARN(
                rclcpp::get_logger("feature_tracker"),
                "[TRACKER] non-increasing image timestamps: previous=%.9f current=%.9f; velocities forced to zero",
                prev_time, cur_time);
            dt = 0.0;
        }
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1)
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())
                {
                    double v_x = dt > 0.0 ? (cur_un_pts[i].x - it->second.x) / dt : 0.0;
                    double v_y = dt > 0.0 ? (cur_un_pts[i].y - it->second.y) / dt : 0.0;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map = cur_un_pts_map;
}
