/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "module.hpp"

PyObject *mod_opencv;

bp::object
cvtColor2Wrap(bp::object obj_in, const std::string & encoding_in, const std::string & encoding_out) {
  // Convert the Python input to an image
  cv::Mat mat_in;
  convert_to_CvMat2(obj_in.ptr(), mat_in);

  // Call cv_bridge for color conversion
  cv_bridge::CvImagePtr cv_image(new cv_bridge::CvImage(std_msgs::Header(), encoding_in, mat_in));

  cv::Mat mat = cv_bridge::cvtColor(cv_image, encoding_out)->image;

  return bp::object(boost::python::handle<>(pyopencv_from(mat)));
}

bp::object
cvtColorForDisplayWrap(bp::object obj_in,
                       const std::string & encoding_in,
                       const std::string & encoding_out,
                       bool do_dynamic_scaling = false,
                       double min_image_value = 0.0,
                       double max_image_value = 0.0) {
  // Convert the Python input to an image
  cv::Mat mat_in;
  convert_to_CvMat2(obj_in.ptr(), mat_in);

  cv_bridge::CvImagePtr cv_image(new cv_bridge::CvImage(std_msgs::Header(), encoding_in, mat_in));

  cv_bridge::CvtColorForDisplayOptions options;
  options.do_dynamic_scaling = do_dynamic_scaling;
  options.min_image_value = min_image_value;
  options.max_image_value = max_image_value;
  cv::Mat mat = cv_bridge::cvtColorForDisplay(/*source=*/cv_image,
                                              /*encoding_out=*/encoding_out,
                                              /*options=*/options)->image;

  return bp::object(boost::python::handle<>(pyopencv_from(mat)));
}

BOOST_PYTHON_FUNCTION_OVERLOADS(cvtColorForDisplayWrap_overloads, cvtColorForDisplayWrap, 3, 6)

int CV_MAT_CNWrap(int i) {
  return CV_MAT_CN(i);
}

int CV_MAT_DEPTHWrap(int i) {
  return CV_MAT_DEPTH(i);
}

BOOST_PYTHON_MODULE(cv_bridge_boost)
{
  do_numpy_import();
  mod_opencv = PyImport_ImportModule("cv2");

  // Wrap the function to get encodings as OpenCV types
  boost::python::def("getCvType", cv_bridge::getCvType);
  boost::python::def("cvtColor2", cvtColor2Wrap);
  boost::python::def("CV_MAT_CNWrap", CV_MAT_CNWrap);
  boost::python::def("CV_MAT_DEPTHWrap", CV_MAT_DEPTHWrap);
  boost::python::def("cvtColorForDisplay", cvtColorForDisplayWrap,
                     cvtColorForDisplayWrap_overloads(
                       boost::python::args("source", "encoding_in", "encoding_out", "do_dynamic_scaling",
                                           "min_image_value", "max_image_value"),
                       "Convert image to display with specified encodings.\n\n"
                       "Args:\n"
                       "  - source (numpy.ndarray): input image\n"
                       "  - encoding_in (str): input image encoding\n"
                       "  - encoding_out (str): encoding to which the image conveted\n"
                       "  - do_dynamic_scaling (bool): flag to do dynamic scaling with min/max value\n"
                       "  - min_image_value (float): minimum pixel value for dynamic scaling\n"
                       "  - max_image_value (float): maximum pixel value for dynamic scaling\n"
                     ));
}
