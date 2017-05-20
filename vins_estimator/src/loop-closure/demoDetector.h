/**
 * File: demoDetector.h
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: demo application of DLoopDetector
 * License: see the LICENSE.txt file
 */

#ifndef __DEMO_DETECTOR__
#define __DEMO_DETECTOR__

#include <iostream>
#include <vector>
#include <string>

// OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>

// DLoopDetector and DBoW2
#include "ThirdParty/DBoW/DBoW2.h"
#include "DLoopDetector.h"

//for time
#include <cctype>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "dirent.h"
#include <unistd.h>
#include <string>
#include <ctime>
#include <cstdlib>
#include <chrono>
#include "../utility/tic_toc.h"

using namespace DLoopDetector;
using namespace DBoW2;
using namespace std;


/// Generic class to create functors to extract features
template<class TDescriptor>
class FeatureExtractor
{
public:
  /**
   * Extracts features
   * @param im image
   * @param keys keypoints extracted
   * @param descriptors descriptors extracted
   */
  virtual void operator()(const cv::Mat &im, const std::vector<cv::Point2f> window_pts,
    vector<cv::KeyPoint> &keys, vector<TDescriptor> &descriptors) const = 0;
};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

/// @param TVocabulary vocabulary class (e.g: Surf64Vocabulary)
/// @param TDetector detector class (e.g: Surf64LoopDetector)
/// @param TDescriptor descriptor class (e.g: vector<float> for SURF)
template<class TVocabulary, class TDetector, class TDescriptor>
/// Class to run the demo 
class demoDetector
{
public:

  /**
   * @param vocfile vocabulary file to load
   * @param imagedir directory to read images from
   * @param posefile pose file
   * @param width image width
   * @param height image height
   */
  demoDetector(const std::string &vocfile, int width, int height);
    
  ~demoDetector(){}

  void initCameraModel(const std::string &calib_file);

  /**
   * Runs the demo
   * @param name demo name
   * @param extractor functor to extract features
   */
  bool run(const std::string &name,
           const std::vector<cv::KeyPoint> &keys, 
           const std::vector<TDescriptor> &descriptors,
           std::vector<cv::Point2f> &cur_pts,
           std::vector<cv::Point2f> &old_pts,
           int &old_index);

  void eraseIndex(std::vector<int> &erase_index);
  /*Data*/
  std::string m_vocfile;
  int m_width;
  int m_height;
  typename TDetector::Parameters params;
  TVocabulary voc;
  TDetector detector;


protected:

  /**
   * Reads the robot poses from a file
   * @param filename file
   * @param xs
   * @param ys
   */
  void readPoseFile(const char *filename, std::vector<double> &xs, 
    std::vector<double> &ys) const;

protected:
  //std::string m_imagedir;
  //std::string m_posefile;
};

// ---------------------------------------------------------------------------

template<class TVocabulary, class TDetector, class TDescriptor>
demoDetector<TVocabulary, TDetector, TDescriptor>::demoDetector
  (const std::string &vocfile, int width, int height)
  : m_vocfile(vocfile), m_width(width), m_height(height),
    params(height, width), voc(vocfile), detector(voc, params)
{
    //params.use_nss = true; // use normalized similarity score instead of raw score
    //params.alpha = 0.3; // nss threshold
    //params.k = 1; // a loop must be consistent with 1 previous matches
    //params.geom_check = GEOM_FLANN; // use direct index for geometrical checking
    //params.di_levels = 2; // use two direct index levels
    //printf("load vocfile %s finish\n", vocfile);
    //printf("loop image size width: %d height: %d\n", params.image_cols,params.image_rows);
}

// ---------------------------------------------------------------------------

template<class TVocabulary, class TDetector, class TDescriptor>
void demoDetector<TVocabulary, TDetector, TDescriptor>::initCameraModel
  (const std::string &calib_file)
{
    detector.initCameraModel(calib_file);
}

// ---------------------------------------------------------------------------
template<class TVocabulary, class TDetector, class TDescriptor>
void demoDetector<TVocabulary, TDetector, TDescriptor>::eraseIndex
(std::vector<int> &erase_index)
{
    detector.eraseIndex(erase_index);
}


// ---------------------------------------------------------------------------

template<class TVocabulary, class TDetector, class TDescriptor>
bool demoDetector<TVocabulary, TDetector, TDescriptor>::run
  (const std::string &name, const std::vector<cv::KeyPoint> &keys, 
   const std::vector<TDescriptor> &descriptors,
   std::vector<cv::Point2f> &cur_pts,
   std::vector<cv::Point2f> &old_pts,
   int &old_index)
{  
  int count = 0;

  DetectionResult result;

  detector.detectLoop(keys, descriptors, result, cur_pts, old_pts); 
    
  if(result.detection())
  {
      //cout << "- loop found with image " << result.match << "!"
      //  << endl;
      ++count;
      old_index = result.match;
      return true;
  }
  else
  {
      //cout << "- No loop: ";
      switch(result.status)
      {
        case CLOSE_MATCHES_ONLY:
          //cout << "All the images in the database are very recent" << endl;
          break;
          
        case NO_DB_RESULTS:
          //cout << "There are no matches against the database (few features in"
          //  " the image?)" << endl;
          break;
          
        case LOW_NSS_FACTOR:
          //cout << "Little overlap between this image and the previous one"
          //  << endl;
          break;
            
        case LOW_SCORES:
          //cout << "No match reaches the score threshold (alpha: " <<
          //  params.alpha << ")" << endl;
          break;
          
        case NO_GROUPS:
          //cout << "Not enough close matches to create groups. "
          //  << "Best candidate: " << result.match << endl;
          break;
          
        case NO_TEMPORAL_CONSISTENCY:
          //cout << "No temporal consistency (k: " << params.k << "). "
          //  << "Best candidate: " << result.match << endl;
          break;
          
        case NO_GEOMETRICAL_CONSISTENCY:
          //cout << "No geometrical consistency. Best candidate: " 
          //  << result.match << endl;
          break;
          
        default:
          break;
      }
      return false;
  }
}

// ---------------------------------------------------------------------------

template<class TVocabulary, class TDetector, class TDescriptor>
void demoDetector<TVocabulary, TDetector, TDescriptor>::readPoseFile
  (const char *filename, std::vector<double> &xs, std::vector<double> &ys)
  const
{
  xs.clear();
  ys.clear();
  
  fstream f(filename, ios::in);
  
  string s;
  double ts, x, y, t;
  while(!f.eof())
  {
    getline(f, s);
    if(!f.eof() && !s.empty())
    {
      sscanf(s.c_str(), "%lf, %lf, %lf, %lf", &ts, &x, &y, &t);
      xs.push_back(x);
      ys.push_back(y);
    }
  }
  
  f.close();
}

// ---------------------------------------------------------------------------

#endif

