#include "cv_bridge/rgb_colors.h"
#include <opencv2/opencv.hpp>
#include <gtest/gtest.h>


TEST(RGBColors, testGetRGBColor)
{
  cv::Vec3d color;
  // red
  color = cv_bridge::rgb_colors::getRGBColor(cv_bridge::rgb_colors::RED);
  EXPECT_EQ(1, color[0]);
  EXPECT_EQ(0, color[1]);
  EXPECT_EQ(0, color[2]);
  // gray
  color = cv_bridge::rgb_colors::getRGBColor(cv_bridge::rgb_colors::GRAY);
  EXPECT_EQ(0.502, color[0]);
  EXPECT_EQ(0.502, color[1]);
  EXPECT_EQ(0.502, color[2]);
}
