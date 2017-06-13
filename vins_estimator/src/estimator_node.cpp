#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"
#include "loop-closure/loop_closure.h"
#include "loop-closure/keyframe.h"
#include "loop-closure/keyframe_database.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

Estimator estimator;

std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
std::mutex m_posegraph_buf;
queue<int> optimize_posegraph_buf;

int sum_of_wait = 0;

std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_loop_drift;
std::mutex m_keyframedatabase_resample;
std::mutex m_update_visualization;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;

queue<pair<cv::Mat, double>> image_buf;
LoopClosure *loop_closure;
KeyFrameDatabase keyframe_database;

int process_keyframe_cnt = 0;
int miss_keyframe_num = 0;
int keyframe_freq = 0;
int global_frame_cnt = 0;
int loop_check_cnt = 0;
//camera param
camodocal::CameraPtr m_camera;
vector<int> erase_index;
Eigen::Vector3d loop_correct_t = Eigen::Vector3d(0, 0, 0);
Eigen::Matrix3d loop_correct_r = Eigen::Matrix3d::Identity();

void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba - tmp_Q.inverse() * estimator.g);

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba - tmp_Q.inverse() * estimator.g);

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = loop_correct_r * estimator.Ps[WINDOW_SIZE] + loop_correct_t;
    tmp_Q = loop_correct_r * estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());

}

std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;

        if (!(imu_buf.back()->header.stamp > feature_buf.front()->header.stamp))
        {
            ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }

        if (!(imu_buf.front()->header.stamp < feature_buf.front()->header.stamp))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp <= img_msg->header.stamp)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }

        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }
}

void process_loop_detection()
{
    if(loop_closure == NULL)
    {
        const char *voc_file = VOC_FILE.c_str();
        TicToc t_load_voc;
        ROS_DEBUG("loop start loop");
        cout << "voc file: " << voc_file << endl;
        loop_closure = new LoopClosure(voc_file, IMAGE_COL, IMAGE_ROW);
        ROS_DEBUG("loop load vocbulary %lf", t_load_voc.toc());
        loop_closure->initCameraModel(CAM_NAMES);
    }

    while(LOOP_CLOSURE)
    {
        m_posegraph_buf.lock();
        int index = -1;
        if (!optimize_posegraph_buf.empty())
        {
            index = optimize_posegraph_buf.front();
            optimize_posegraph_buf.pop();
        }
        m_posegraph_buf.unlock();
        if(index != -1)
        {
            Vector3d correct_t = Vector3d::Zero();
            Matrix3d correct_r = Matrix3d::Identity();
            TicToc t_posegraph;
            keyframe_database.optimize4DoFLoopPoseGraph(index,
                                                    correct_t,
                                                    correct_r);
            ROS_DEBUG("t_posegraph %f ms", t_posegraph.toc());
            m_loop_drift.lock();
            loop_correct_r = correct_r;
            loop_correct_t = correct_t;
            m_loop_drift.unlock();
            m_update_visualization.lock();
            keyframe_database.updateVisualization();
            m_update_visualization.unlock();
            nav_msgs::Path refine_path = keyframe_database.getPath();
            updateLoopPath(refine_path);
        }

        if (loop_check_cnt < global_frame_cnt)
        {
            erase_index.clear();
            m_keyframedatabase_resample.lock();
            keyframe_database.resample(erase_index);
            m_keyframedatabase_resample.unlock();
            
            m_update_visualization.lock();
            keyframe_database.updateVisualization();
            m_update_visualization.unlock();

            if(!erase_index.empty())
                loop_closure->eraseIndex(erase_index);

            KeyFrame* cur_kf = keyframe_database.getLastUncheckKeyframe();
            assert(loop_check_cnt == cur_kf->global_index);
            loop_check_cnt++;
            cur_kf->check_loop = 1;

            cv::Mat current_image;
            current_image = cur_kf->image;

            std::vector<cv::Point2f> measurements_old;
            std::vector<cv::Point2f> measurements_old_norm;
            std::vector<cv::Point2f> measurements_cur;
            std::vector<int> features_id;     
            std::vector<cv::Point2f> measurements_cur_origin = cur_kf->measurements;

            bool loop_succ = false;
            int old_index = -1;
            TicToc t_loop;
            vector<cv::Point2f> cur_pts;
            vector<cv::Point2f> old_pts;
            TicToc t_brief;
            cur_kf->extractBrief(current_image);
            //printf("loop extract %d feature using %lf\n", cur_kf->keypoints.size(), t_brief.toc());
            TicToc t_loopdetect;
            loop_succ = loop_closure->startLoopClosure(cur_kf->keypoints, cur_kf->descriptors, cur_pts, old_pts, old_index);
            ROS_DEBUG("t_loopdetect %f ms", t_loopdetect.toc());
            if(loop_succ)
            {
                KeyFrame* old_kf = keyframe_database.getKeyframe(old_index);
                if (old_kf == NULL)
                {
                    ROS_WARN("NO such frame in keyframe_database");
                    ROS_BREAK();
                }
                ROS_DEBUG("loop succ %d with %drd image", loop_check_cnt, old_index);
                assert(old_index!=-1);
                
                Vector3d T_w_i_old, T_w_i_refine;
                Matrix3d R_w_i_old, R_w_i_refine;

                old_kf->getOriginPose(T_w_i_old, R_w_i_old);
                cur_kf->findConnectionWithOldFrame(old_kf, cur_pts, old_pts,
                                                   measurements_old, measurements_old_norm, m_camera);
                measurements_cur = cur_kf->measurements;
                features_id = cur_kf->features_id;
                
                /**
                *** send features and ids to VINS
                **/
                //if(measurements_old_norm.size()>MIN_LOOP_NUM && process_keyframe_cnt - old_index > 50 && old_index > 50)
                int loop_fusion = 0;
                if( (int)measurements_old_norm.size() > MIN_LOOP_NUM && loop_check_cnt - old_index > 35 && old_index > 30)
                {

                    Quaterniond Q_loop_old(R_w_i_old);
                    Quaterniond Q_loop_cur(R_w_i_refine);
                    RetriveData retrive_data;
                    retrive_data.cur_index = cur_kf->global_index;
                    retrive_data.header = cur_kf->header;
                    retrive_data.P_old = T_w_i_old;
                    retrive_data.Q_old = Q_loop_old;
                    retrive_data.P_cur = T_w_i_refine;
                    retrive_data.Q_cur = Q_loop_cur;
                    retrive_data.use = true;
                    retrive_data.measurements = measurements_old_norm;
                    retrive_data.features_ids = features_id;
                    estimator.retrive_pose_data = (retrive_data);    

                    //cout << "old pose " << T_w_i_old.transpose() << endl;
                    //cout << "refinded pose " << T_w_i_refine.transpose() << endl;
                    // add loop edge in current frame
                    cur_kf->detectLoop(old_index);
                    //keyframe_database.addLoop(old_index);
                    old_kf->is_looped = 1;
                    loop_fusion = 1;
                }
                if(0)
                {
                    int COL = current_image.cols;
                    //int ROW = current_image.rows;
                    cv::Mat gray_img, loop_match_img;
                    cv::Mat old_img = old_kf->image;
                    cv::hconcat(old_img, current_image, gray_img);
                    cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
                    cv::Mat loop_match_img2;
                    loop_match_img2 = loop_match_img.clone();

                    for(int i = 0; i< (int)cur_pts.size(); i++)
                    {
                        cv::Point2f cur_pt = cur_pts[i];
                        cur_pt.x += COL;
                        cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
                    }
                    for(int i = 0; i< (int)old_pts.size(); i++)
                    {
                        cv::circle(loop_match_img, old_pts[i], 5, cv::Scalar(0, 255, 0));
                    }
                    for (int i = 0; i< (int)old_pts.size(); i++)
                    {
                        cv::Point2f cur_pt = cur_pts[i];
                        cur_pt.x += COL ;
                        cv::line(loop_match_img, old_pts[i], cur_pt, cv::Scalar(0, 255, 0), 1, 8, 0);
                    }
                    ostringstream convert;
                    convert << "/home/tony-ws/raw_data/loop_image/"
                            << cur_kf->global_index << "-" 
                            << old_index << "-" << loop_fusion <<".jpg";
                    cv::imwrite( convert.str().c_str(), loop_match_img);


                    
                    for(int i = 0; i< (int)measurements_cur.size(); i++)
                    {
                        cv::Point2f cur_pt = measurements_cur[i];
                        cur_pt.x += COL;
                        cv::circle(loop_match_img2, cur_pt, 5, cv::Scalar(0, 255, 0));
                    }
                    for(int i = 0; i< (int)measurements_old.size(); i++)
                    {
                        cv::circle(loop_match_img2, measurements_old[i], 5, cv::Scalar(0, 255, 0));
                    }
                    for (int i = 0; i< (int)measurements_old.size(); i++)
                    {
                        cv::Point2f cur_pt = measurements_cur[i];
                        cur_pt.x += COL ;
                        cv::line(loop_match_img2, measurements_old[i], cur_pt, cv::Scalar(0, 255, 0), 1, 8, 0);
                    }
                    ostringstream convert2;
                    convert2 << "/home/tony-ws/raw_data/loop_image/"
                            << cur_kf->global_index << "-" 
                            << old_index << "-" << loop_fusion <<"-2.jpg";
                    cv::imwrite( convert2.str().c_str(), loop_match_img2);
                }
                  
            }
            //release memory
            cur_kf->image.release();
        }

        std::chrono::milliseconds dura(10);
        std::this_thread::sleep_for(dura);
    }
}

void raw_image_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    //image_pool[img_msg->header.stamp.toNSec()] = img_ptr->image;
    if(LOOP_CLOSURE)
    {
        i_buf.lock();
        image_buf.push(make_pair(img_ptr->image, img_msg->header.stamp.toSec()));
        i_buf.unlock();
    }
}

void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

void send_imu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (current_time < 0)
        current_time = t;
    double dt = t - current_time;
    current_time = t;

    double ba[]{0.0, 0.0, 0.0};
    double bg[]{0.0, 0.0, 0.0};

    double dx = imu_msg->linear_acceleration.x - ba[0];
    double dy = imu_msg->linear_acceleration.y - ba[1];
    double dz = imu_msg->linear_acceleration.z - ba[2];

    double rx = imu_msg->angular_velocity.x - bg[0];
    double ry = imu_msg->angular_velocity.y - bg[1];
    double rz = imu_msg->angular_velocity.z - bg[2];
    //ROS_DEBUG("IMU %f, dt: %f, acc: %f %f %f, gyr: %f %f %f", t, dt, dx, dy, dz, rx, ry, rz);

    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
}

void process()
{
    while (true)
    {
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;
                 });
        lk.unlock();

        for (auto &measurement : measurements)
        {
            for (auto &imu_msg : measurement.first)
                send_imu(imu_msg);

            auto img_msg = measurement.second;
            ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());

            TicToc t_s;
            map<int, vector<pair<int, Vector3d>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                ROS_ASSERT(z == 1);
                image[feature_id].emplace_back(camera_id, Vector3d(x, y, z));
            }

            estimator.processImage(image, img_msg->header);
            /**
            *** start build keyframe database for loop closure
            **/
            if(LOOP_CLOSURE)
            {
                if(estimator.marginalization_flag == 0 && estimator.solver_flag == estimator.NON_LINEAR)
                {   
                    if (keyframe_freq % 3 == 0)
                    {
                        keyframe_freq = 0;
                        /**
                        ** save the newest keyframe to the keyframe database
                        ** only need to save the pose to the keyframe database
                        **/
                        Vector3d T_w_i = estimator.Ps[WINDOW_SIZE - 2];
                        Matrix3d R_w_i = estimator.Rs[WINDOW_SIZE - 2];
                        i_buf.lock();
                        while(!image_buf.empty() && image_buf.front().second < estimator.Headers[WINDOW_SIZE - 2].stamp.toSec())
                        {
                            image_buf.pop();
                        }
                        i_buf.unlock();
                        //assert(estimator.Headers[WINDOW_SIZE - 1].stamp.toSec() == image_buf.front().second);
                        // relative_T   i-1_T_i relative_R  i-1_R_i
                        cv::Mat KeyFrame_image;
                        KeyFrame_image = image_buf.front().first;
                        
                        const char *pattern_file = PATTERN_FILE.c_str();
                        KeyFrame* keyframe = new KeyFrame(estimator.Headers[WINDOW_SIZE - 2].stamp.toSec(), global_frame_cnt, T_w_i, R_w_i, image_buf.front().first, pattern_file);
                        keyframe->setExtrinsic(TIC[0], RIC[0]);
                        /*
                        ** we still need save the measurement to the keyframe(not database) for add connection with looped old pose
                        ** and save the pointcloud to the keyframe for reprojection search correspondance
                        */
                        keyframe->buildKeyFrameFeatures(estimator, m_camera);
                        m_keyframedatabase_resample.lock();
                        keyframe_database.add(keyframe);
                        m_keyframedatabase_resample.unlock();
                        global_frame_cnt++;
                    }

                    // update loop info
                    m_keyframedatabase_resample.lock();
                    for (int i = 0; i < WINDOW_SIZE; i++)
                    {
                        if(estimator.Headers[i].stamp.toSec() == estimator.front_pose.header)
                        {
                            KeyFrame* cur_kf = keyframe_database.getKeyframe(estimator.front_pose.cur_index);                            
                            if (abs(estimator.front_pose.relative_yaw) > 30.0 || estimator.front_pose.relative_t.norm() > 20.0)
                            {
                                ROS_DEBUG("Wrong loop");
                                cur_kf->removeLoop();
                                break;
                            }
                            cur_kf->updateLoopConnection( estimator.front_pose.relative_t, 
                                                          estimator.front_pose.relative_q, 
                                                          estimator.front_pose.relative_yaw);
                            break;
                        }
                    }
                    /*
                    ** update the keyframe pose when this frame slides out the window and optimize loop graph
                    */
                    int search_cnt = 0;
                    
                    for(int i = 0; i < keyframe_database.size(); i++)
                    {
                        search_cnt++;
                        KeyFrame* kf = keyframe_database.getLastKeyframe(i);
                        if(kf->header == estimator.Headers[0].stamp.toSec())
                        {
                            kf->updateOriginPose(estimator.Ps[0], estimator.Rs[0]);
                            //update edge
                            // if loop happens in this frame, update pose graph;
                            if (kf->update_loop_info)
                            {
                                m_posegraph_buf.lock();
                                optimize_posegraph_buf.push(kf->global_index);
                                m_posegraph_buf.unlock();
                            }
                            break; 
                        }
                        else
                        {
                            if(search_cnt > 2 * WINDOW_SIZE)
                                break;
                        }
                    }
                    m_keyframedatabase_resample.unlock();
                    keyframe_freq++;
                }
                std_msgs::Header header = img_msg->header;
                m_update_visualization.lock();
                CameraPoseVisualization* posegraph_visualization = keyframe_database.getPosegraphVisualization();
                //ROS_WARN("pub visualization begin!");
                pubPoseGraph(posegraph_visualization, header);  
                //ROS_WARN("pub visualization end!"); 
                m_update_visualization.unlock();
            }
            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t);
            std_msgs::Header header = img_msg->header;
            header.frame_id = "world";
            pubOdometry(estimator, header, loop_correct_t, loop_correct_r);
            m_loop_drift.lock();
            pubKeyPoses(estimator, header, loop_correct_t, loop_correct_r);
            pubCameraPose(estimator, header, loop_correct_t, loop_correct_r);
            pubPointCloud(estimator, header, loop_correct_t, loop_correct_r);
            pubTF(estimator, header, loop_correct_t, loop_correct_r);
            m_loop_drift.unlock();

            //ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);
    estimator.setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_image = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_raw_image = n.subscribe(IMAGE_TOPIC, 2000, raw_image_callback);

    std::thread measurement_process{process};
    std::thread loop_detection;
    if (LOOP_CLOSURE)
    {
        ROS_WARN("LOOP_CLOSURE true");
        loop_detection = std::thread(process_loop_detection);   
        m_camera = CameraFactory::instance()->generateCameraFromYamlFile(CAM_NAMES);
    }
    ros::spin();

    return 0;
}
