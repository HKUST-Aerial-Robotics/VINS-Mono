#ifndef __KEY_FRAME_
#define __KEY_FRAME_

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include "../utility/utility.h"
#include <algorithm>
#include "math.h"
#include "../estimator.h"
#include "../parameters.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include <mutex>
#include "loop_closure.h"

using namespace Eigen;

// This functor extracts BRIEF descriptors in the required format
class BriefExtractor: public FeatureExtractor<FBrief::TDescriptor>
{
public:
  virtual void operator()(const cv::Mat &im, const std::vector<cv::Point2f> window_pts,
    vector<cv::KeyPoint> &keys, vector<BRIEF::bitset> &descriptors) const;
  BriefExtractor(const std::string &pattern_file);

private:
  DVision::BRIEF m_brief;
};

class KeyFrame
{
public:
	KeyFrame(double _header, Eigen::Vector3d _vio_T_w_c, Eigen::Matrix3d _vio_R_w_c, 
				Eigen::Vector3d _cur_T_w_c, Eigen::Matrix3d _cur_R_w_c,cv::Mat &_image, const char *_brief_pattern_file);
	void setExtrinsic(Eigen::Vector3d T, Eigen::Matrix3d R);	
	void FundmantalMatrixRANSAC(vector<cv::Point2f> &measurements_old, vector<cv::Point2f> &measurements_old_norm,
                 	 const camodocal::CameraPtr &m_camera);

	void extractBrief(cv::Mat &image);
	
	void buildKeyFrameFeatures(Estimator &estimator, const camodocal::CameraPtr &m_camera);
	
	bool inAera(cv::Point2f pt, cv::Point2f center, float area_size);

	bool searchInAera(cv::Point2f center_cur, float area_size,
                      const BRIEF::bitset window_descriptor,
                      const std::vector<BRIEF::bitset> &descriptors_old,
                      const std::vector<cv::KeyPoint> &keypoints_old,
                      cv::Point2f &best_match);

	void searchByDes(std::vector<cv::Point2f> &measurements_old,
					 std::vector<cv::Point2f> &measurements_old_norm,
                     const std::vector<BRIEF::bitset> &descriptors_old,
                     const std::vector<cv::KeyPoint> &keypoints_old,
                     const camodocal::CameraPtr &m_camera);

	bool findConnectionWithOldFrame(const KeyFrame* old_kf,
	                                std::vector<cv::Point2f> &measurements_old, std::vector<cv::Point2f> &measurements_old_norm,
	                                Eigen::Vector3d &PnP_T_old, Eigen::Matrix3d &PnP_R_old,
	                                const camodocal::CameraPtr &m_camera);

	void PnPRANSAC(vector<cv::Point2f> &measurements_old,
	               std::vector<cv::Point2f> &measurements_old_norm, 
	               Eigen::Vector3d &PnP_T_old, Eigen::Matrix3d &PnP_R_old);

	void updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);

	void updateOriginPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);

	void getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);

	void getOriginPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);

	void addConnection(int index, KeyFrame* connected_kf);

	void addConnection(int index, KeyFrame* connected_kf, Vector3d relative_t, Quaterniond relative_q, double relative_yaw);

	void updateLoopConnection(Vector3d relative_t, Quaterniond relative_q, double relative_yaw);

	void detectLoop(int index);

	void removeLoop();

	int HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b);

	Eigen::Vector3d getLoopRelativeT();

	double getLoopRelativeYaw();

	// data 
	double header;
	std::vector<Eigen::Vector3d> point_clouds, point_clouds_matched;
	//feature in origin image plane
	std::vector<cv::Point2f> measurements, measurements_matched;
	//feature in normalize image plane
	std::vector<cv::Point2f> pts_normalize;
	//feature ID
	std::vector<int> features_id, features_id_matched;
	//feature descriptor
	std::vector<BRIEF::bitset> descriptors;
	//keypoints
	std::vector<cv::KeyPoint> keypoints;

	int global_index;
	cv::Mat image;
	Matrix3d qic;
	Vector3d tic;
	int COL, ROW;
	bool use_retrive;

	bool has_loop;
	int loop_index;
	bool update_loop_info;
	// index t_x t_y t_z q_w q_x q_y q_z yaw
	// old_R_cur old_T_cur

	// looped by other frame
	bool is_looped;
	int resample_index;
	const char *BRIEF_PATTERN_FILE;
	// index t_x t_y t_z q_w q_x q_y q_z yaw
	

private:
	Eigen::Vector3d T_w_i;
	Eigen::Matrix3d R_w_i;
	Eigen::Vector3d vio_T_w_i;
	Eigen::Matrix3d vio_R_w_i;
	std::mutex mMutexPose;
	std::mutex mLoopInfo;
	std::vector<cv::KeyPoint> window_keypoints;
	std::vector<BRIEF::bitset> window_descriptors;
	Eigen::Matrix<double, 8, 1 > loop_info;

};

#endif

