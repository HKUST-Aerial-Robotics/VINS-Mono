/*
 * Copyright 2014 Open Source Robotics Foundation, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CV_BRIDGE_MODULE_HPP_
#define CV_BRIDGE_MODULE_HPP_

#include <iostream>
#include <boost/python.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Python.h>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/ndarrayobject.h>

#include <opencv2/core/core.hpp>

namespace bp = boost::python;

int convert_to_CvMat2(const PyObject* o, cv::Mat& m);

PyObject* pyopencv_from(const cv::Mat& m);

#if PYTHON3
static int do_numpy_import( )
{
    import_array( );
}
#else
static void do_numpy_import( )
{
    import_array( );
}
#endif

#endif
