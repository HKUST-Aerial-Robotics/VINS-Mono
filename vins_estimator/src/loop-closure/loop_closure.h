#ifndef __LOOP_CLOSURE__
#define __LOOP_CLOSURE__

#include <iostream>
#include <vector>
#include <string>

// DLoopDetector and DBoW2
#include "ThirdParty/DBoW/DBoW2.h" // defines BriefVocabulary
#include "DLoopDetector.h" // defines BriefLoopDetector
#include "ThirdParty/DVision/DVision.h" // Brief

// OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "demoDetector.h"
//#include "brief_extractor.h"

using namespace DLoopDetector;
using namespace DBoW2;
using namespace DVision;
using namespace std;


class LoopClosure
{
public:
	LoopClosure(const char *voc_file, int _image_w, int _image_h);

	bool startLoopClosure(std::vector<cv::KeyPoint> &keys, std::vector<BRIEF::bitset> &descriptors,
                                   std::vector<cv::Point2f> &cur_pts,
                                   std::vector<cv::Point2f> &old_pts,
                                   int &old_index);
	void initCameraModel(const std::string &calib_file);

	void eraseIndex(std::vector<int> &erase_index);
	/* data */
	demoDetector<BriefVocabulary, BriefLoopDetector, FBrief::TDescriptor> demo;
	int IMAGE_W;
	int IMAGE_H;
};

#endif