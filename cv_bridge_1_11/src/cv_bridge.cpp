/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
*  Copyright (c) 2015, Tal Regev.
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

#include "boost/endian/conversion.hpp"

#include <map>

#include <boost/make_shared.hpp>
#include <boost/regex.hpp>

#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/rgb_colors.h>

namespace enc = sensor_msgs::image_encodings;

namespace cv_bridge {

static int depthStrToInt(const std::string depth) {
  if (depth == "8U") {
    return 0;
  } else if (depth == "8S") {
    return 1;
  } else if (depth == "16U") {
    return 2;
  } else if (depth == "16S") {
    return 3;
  } else if (depth == "32S") {
    return 4;
  } else if (depth == "32F") {
    return 5;
  }
  return 6;
}

int getCvType(const std::string& encoding)
{
  // Check for the most common encodings first
  if (encoding == enc::BGR8)   return CV_8UC3;
  if (encoding == enc::MONO8)  return CV_8UC1;
  if (encoding == enc::RGB8)   return CV_8UC3;
  if (encoding == enc::MONO16) return CV_16UC1;
  if (encoding == enc::BGR16)  return CV_16UC3;
  if (encoding == enc::RGB16)  return CV_16UC3;
  if (encoding == enc::BGRA8)  return CV_8UC4;
  if (encoding == enc::RGBA8)  return CV_8UC4;
  if (encoding == enc::BGRA16) return CV_16UC4;
  if (encoding == enc::RGBA16) return CV_16UC4;

  // For bayer, return one-channel
  if (encoding == enc::BAYER_RGGB8) return CV_8UC1;
  if (encoding == enc::BAYER_BGGR8) return CV_8UC1;
  if (encoding == enc::BAYER_GBRG8) return CV_8UC1;
  if (encoding == enc::BAYER_GRBG8) return CV_8UC1;
  if (encoding == enc::BAYER_RGGB16) return CV_16UC1;
  if (encoding == enc::BAYER_BGGR16) return CV_16UC1;
  if (encoding == enc::BAYER_GBRG16) return CV_16UC1;
  if (encoding == enc::BAYER_GRBG16) return CV_16UC1;

  // Miscellaneous
  if (encoding == enc::YUV422) return CV_8UC2;

  // Check all the generic content encodings
  boost::cmatch m;

  if (boost::regex_match(encoding.c_str(), m,
        boost::regex("(8U|8S|16U|16S|32S|32F|64F)C([0-9]+)"))) {
    return CV_MAKETYPE(depthStrToInt(m[1].str()), atoi(m[2].str().c_str()));
  }

  if (boost::regex_match(encoding.c_str(), m,
        boost::regex("(8U|8S|16U|16S|32S|32F|64F)"))) {
    return CV_MAKETYPE(depthStrToInt(m[1].str()), 1);
  }

  throw Exception("Unrecognized image encoding [" + encoding + "]");
}

/// @cond DOXYGEN_IGNORE

enum Encoding { INVALID = -1, GRAY = 0, RGB, BGR, RGBA, BGRA, YUV422, BAYER_RGGB, BAYER_BGGR, BAYER_GBRG, BAYER_GRBG};

Encoding getEncoding(const std::string& encoding)
{
  if ((encoding == enc::MONO8) || (encoding == enc::MONO16)) return GRAY;
  if ((encoding == enc::BGR8) || (encoding == enc::BGR16))  return BGR;
  if ((encoding == enc::RGB8) || (encoding == enc::RGB16))  return RGB;
  if ((encoding == enc::BGRA8) || (encoding == enc::BGRA16))  return BGRA;
  if ((encoding == enc::RGBA8) || (encoding == enc::RGBA16))  return RGBA;
  if (encoding == enc::YUV422) return YUV422;

  if ((encoding == enc::BAYER_RGGB8) || (encoding == enc::BAYER_RGGB16)) return BAYER_RGGB;
  if ((encoding == enc::BAYER_BGGR8) || (encoding == enc::BAYER_BGGR16)) return BAYER_BGGR;
  if ((encoding == enc::BAYER_GBRG8) || (encoding == enc::BAYER_GBRG16)) return BAYER_GBRG;
  if ((encoding == enc::BAYER_GRBG8) || (encoding == enc::BAYER_GRBG16)) return BAYER_GRBG;

  // We don't support conversions to/from other types
  return INVALID;
}

static const int SAME_FORMAT = -1;

/** Return a lit of OpenCV conversion codes to get from one Format to the other
 * The key is a pair: <FromFormat, ToFormat> and the value a succession of OpenCV code conversion
 * It's not efficient code but it is only called once and the structure is small enough
 */
std::map<std::pair<Encoding, Encoding>, std::vector<int> > getConversionCodes() {
  std::map<std::pair<Encoding, Encoding>, std::vector<int> > res;
  for(int i=0; i<=5; ++i)
    res[std::pair<Encoding, Encoding>(Encoding(i),Encoding(i))].push_back(SAME_FORMAT);

  res[std::make_pair(GRAY, RGB)].push_back(cv::COLOR_GRAY2RGB);
  res[std::make_pair(GRAY, BGR)].push_back(cv::COLOR_GRAY2BGR);
  res[std::make_pair(GRAY, RGBA)].push_back(cv::COLOR_GRAY2RGBA);
  res[std::make_pair(GRAY, BGRA)].push_back(cv::COLOR_GRAY2BGRA);

  res[std::make_pair(RGB, GRAY)].push_back(cv::COLOR_RGB2GRAY);
  res[std::make_pair(RGB, BGR)].push_back(cv::COLOR_RGB2BGR);
  res[std::make_pair(RGB, RGBA)].push_back(cv::COLOR_RGB2RGBA);
  res[std::make_pair(RGB, BGRA)].push_back(cv::COLOR_RGB2BGRA);

  res[std::make_pair(BGR, GRAY)].push_back(cv::COLOR_BGR2GRAY);
  res[std::make_pair(BGR, RGB)].push_back(cv::COLOR_BGR2RGB);
  res[std::make_pair(BGR, RGBA)].push_back(cv::COLOR_BGR2RGBA);
  res[std::make_pair(BGR, BGRA)].push_back(cv::COLOR_BGR2BGRA);

  res[std::make_pair(RGBA, GRAY)].push_back(cv::COLOR_RGBA2GRAY);
  res[std::make_pair(RGBA, RGB)].push_back(cv::COLOR_RGBA2RGB);
  res[std::make_pair(RGBA, BGR)].push_back(cv::COLOR_RGBA2BGR);
  res[std::make_pair(RGBA, BGRA)].push_back(cv::COLOR_RGBA2BGRA);

  res[std::make_pair(BGRA, GRAY)].push_back(cv::COLOR_BGRA2GRAY);
  res[std::make_pair(BGRA, RGB)].push_back(cv::COLOR_BGRA2RGB);
  res[std::make_pair(BGRA, BGR)].push_back(cv::COLOR_BGRA2BGR);
  res[std::make_pair(BGRA, RGBA)].push_back(cv::COLOR_BGRA2RGBA);

  res[std::make_pair(YUV422, GRAY)].push_back(cv::COLOR_YUV2GRAY_UYVY);
  res[std::make_pair(YUV422, RGB)].push_back(cv::COLOR_YUV2RGB_UYVY);
  res[std::make_pair(YUV422, BGR)].push_back(cv::COLOR_YUV2BGR_UYVY);
  res[std::make_pair(YUV422, RGBA)].push_back(cv::COLOR_YUV2RGBA_UYVY);
  res[std::make_pair(YUV422, BGRA)].push_back(cv::COLOR_YUV2BGRA_UYVY);

  // Deal with Bayer
  res[std::make_pair(BAYER_RGGB, GRAY)].push_back(cv::COLOR_BayerBG2GRAY);
  res[std::make_pair(BAYER_RGGB, RGB)].push_back(cv::COLOR_BayerBG2RGB);
  res[std::make_pair(BAYER_RGGB, BGR)].push_back(cv::COLOR_BayerBG2BGR);

  res[std::make_pair(BAYER_BGGR, GRAY)].push_back(cv::COLOR_BayerRG2GRAY);
  res[std::make_pair(BAYER_BGGR, RGB)].push_back(cv::COLOR_BayerRG2RGB);
  res[std::make_pair(BAYER_BGGR, BGR)].push_back(cv::COLOR_BayerRG2BGR);

  res[std::make_pair(BAYER_GBRG, GRAY)].push_back(cv::COLOR_BayerGR2GRAY);
  res[std::make_pair(BAYER_GBRG, RGB)].push_back(cv::COLOR_BayerGR2RGB);
  res[std::make_pair(BAYER_GBRG, BGR)].push_back(cv::COLOR_BayerGR2BGR);

  res[std::make_pair(BAYER_GRBG, GRAY)].push_back(cv::COLOR_BayerGB2GRAY);
  res[std::make_pair(BAYER_GRBG, RGB)].push_back(cv::COLOR_BayerGB2RGB);
  res[std::make_pair(BAYER_GRBG, BGR)].push_back(cv::COLOR_BayerGB2BGR);

  return res;
}

const std::vector<int> getConversionCode(std::string src_encoding, std::string dst_encoding)
{
  Encoding src_encod = getEncoding(src_encoding);
  Encoding dst_encod = getEncoding(dst_encoding);
  bool is_src_color_format = enc::isColor(src_encoding) || enc::isMono(src_encoding) ||
                             enc::isBayer(src_encoding) || (src_encoding == enc::YUV422);
  bool is_dst_color_format = enc::isColor(dst_encoding) || enc::isMono(dst_encoding) ||
                             enc::isBayer(dst_encoding) || (dst_encoding == enc::YUV422);
  bool is_num_channels_the_same = (enc::numChannels(src_encoding) == enc::numChannels(dst_encoding));

  // If we have no color info in the source, we can only convert to the same format which
  // was resolved in the previous condition. Otherwise, fail
  if (!is_src_color_format) {
    if (is_dst_color_format)
      throw Exception("[" + src_encoding + "] is not a color format. but [" + dst_encoding +
                      "] is. The conversion does not make sense");
    if (!is_num_channels_the_same)
      throw Exception("[" + src_encoding + "] and [" + dst_encoding + "] do not have the same number of channel");
    return std::vector<int>(1, SAME_FORMAT);
  }

  // If we are converting from a color type to a non color type, we can only do so if we stick
  // to the number of channels
  if (!is_dst_color_format) {
    if (!is_num_channels_the_same)
      throw Exception("[" + src_encoding + "] is a color format but [" + dst_encoding + "] " +
                      "is not so they must have the same OpenCV type, CV_8UC3, CV16UC1 ....");
    return std::vector<int>(1, SAME_FORMAT);
  }

  // If we are converting from a color type to another type, then everything is fine
  static const std::map<std::pair<Encoding, Encoding>, std::vector<int> > CONVERSION_CODES = getConversionCodes();

  std::pair<Encoding, Encoding> key(src_encod, dst_encod);
  std::map<std::pair<Encoding, Encoding>, std::vector<int> >::const_iterator val = CONVERSION_CODES.find(key);
  if (val == CONVERSION_CODES.end())
    throw Exception("Unsupported conversion from [" + src_encoding +
                      "] to [" + dst_encoding + "]");

  // And deal with depth differences if the colors are different
  std::vector<int> res = val->second;
  if ((enc::bitDepth(src_encoding) != enc::bitDepth(dst_encoding)) && (getEncoding(src_encoding) != getEncoding(dst_encoding)))
    res.push_back(SAME_FORMAT);

  return res;
}

/////////////////////////////////////// Image ///////////////////////////////////////////

// Converts a ROS Image to a cv::Mat by sharing the data or chaning its endianness if needed
cv::Mat matFromImage(const sensor_msgs::Image& source)
{
  int source_type = getCvType(source.encoding);
  int byte_depth = enc::bitDepth(source.encoding) / 8;
  int num_channels = enc::numChannels(source.encoding);

  if (source.step < source.width * byte_depth * num_channels)
  {
    std::stringstream ss;
    ss << "Image is wrongly formed: step < width * byte_depth * num_channels  or  " << source.step << " != " <<
        source.width << " * " << byte_depth << " * " << num_channels;
    throw Exception(ss.str());
  }

  if (source.height * source.step != source.data.size())
  {
    std::stringstream ss;
    ss << "Image is wrongly formed: height * step != size  or  " << source.height << " * " <<
              source.step << " != " << source.data.size();
    throw Exception(ss.str());
  }

  // If the endianness is the same as locally, share the data
  cv::Mat mat(source.height, source.width, source_type, const_cast<uchar*>(&source.data[0]), source.step);
  if ((boost::endian::order::native == boost::endian::order::big && source.is_bigendian) ||
      (boost::endian::order::native == boost::endian::order::little && !source.is_bigendian) ||
      byte_depth == 1)
    return mat;

  // Otherwise, reinterpret the data as bytes and switch the channels accordingly
  mat = cv::Mat(source.height, source.width, CV_MAKETYPE(CV_8U, num_channels*byte_depth),
                const_cast<uchar*>(&source.data[0]), source.step);
  cv::Mat mat_swap(source.height, source.width, mat.type());

  std::vector<int> fromTo;
  fromTo.reserve(num_channels*byte_depth);
  for(int i = 0; i < num_channels; ++i)
    for(int j = 0; j < byte_depth; ++j)
    {
      fromTo.push_back(byte_depth*i + j);
      fromTo.push_back(byte_depth*i + byte_depth - 1 - j);
    }
  cv::mixChannels(std::vector<cv::Mat>(1, mat), std::vector<cv::Mat>(1, mat_swap), fromTo);

  // Interpret mat_swap back as the proper type
  mat_swap = cv::Mat(source.height, source.width, source_type, mat_swap.data, mat_swap.step);

  return mat_swap;
}

// Internal, used by toCvCopy and cvtColor
CvImagePtr toCvCopyImpl(const cv::Mat& source,
                        const std_msgs::Header& src_header,
                        const std::string& src_encoding,
                        const std::string& dst_encoding)
{
  // Copy metadata
  CvImagePtr ptr = boost::make_shared<CvImage>();
  ptr->header = src_header;
  
  // Copy to new buffer if same encoding requested
  if (dst_encoding.empty() || dst_encoding == src_encoding)
  {
    ptr->encoding = src_encoding;
    source.copyTo(ptr->image);
  }
  else
  {
    // Convert the source data to the desired encoding
    const std::vector<int> conversion_codes = getConversionCode(src_encoding, dst_encoding);
    cv::Mat image1 = source;
    cv::Mat image2;
    for(size_t i=0; i<conversion_codes.size(); ++i) {
      int conversion_code = conversion_codes[i];
      if (conversion_code == SAME_FORMAT)
      {
        // Same number of channels, but different bit depth
        int src_depth = enc::bitDepth(src_encoding);
        int dst_depth = enc::bitDepth(dst_encoding);
        // Keep the number of channels for now but changed to the final depth
        int image2_type = CV_MAKETYPE(CV_MAT_DEPTH(getCvType(dst_encoding)), image1.channels());

        // Do scaling between CV_8U [0,255] and CV_16U [0,65535] images.
        if (src_depth == 8 && dst_depth == 16)
          image1.convertTo(image2, image2_type, 65535. / 255.);
        else if (src_depth == 16 && dst_depth == 8)
          image1.convertTo(image2, image2_type, 255. / 65535.);
        else
          image1.convertTo(image2, image2_type);
      }
      else
      {
        // Perform color conversion
        cv::cvtColor(image1, image2, conversion_code);
      }
      image1 = image2;
    }
    ptr->image = image2;
    ptr->encoding = dst_encoding;
  }

  return ptr;
}

/// @endcond

sensor_msgs::ImagePtr CvImage::toImageMsg() const
{
  sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
  toImageMsg(*ptr);
  return ptr;
}

void CvImage::toImageMsg(sensor_msgs::Image& ros_image) const
{
  ros_image.header = header;
  ros_image.height = image.rows;
  ros_image.width = image.cols;
  ros_image.encoding = encoding;
  ros_image.is_bigendian = (boost::endian::order::native == boost::endian::order::big);
  ros_image.step = image.cols * image.elemSize();
  size_t size = ros_image.step * image.rows;
  ros_image.data.resize(size);

  if (image.isContinuous())
  {
    memcpy((char*)(&ros_image.data[0]), image.data, size);
  }
  else
  {
    // Copy by row by row
    uchar* ros_data_ptr = (uchar*)(&ros_image.data[0]);
    uchar* cv_data_ptr = image.data;
    for (int i = 0; i < image.rows; ++i)
    {
      memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
      ros_data_ptr += ros_image.step;
      cv_data_ptr += image.step;
    }
  }
}

// Deep copy data, returnee is mutable
CvImagePtr toCvCopy(const sensor_msgs::ImageConstPtr& source,
                    const std::string& encoding)
{
  return toCvCopy(*source, encoding);
}

CvImagePtr toCvCopy(const sensor_msgs::Image& source,
                    const std::string& encoding)
{
  // Construct matrix pointing to source data
  return toCvCopyImpl(matFromImage(source), source.header, source.encoding, encoding);
}

// Share const data, returnee is immutable
CvImageConstPtr toCvShare(const sensor_msgs::ImageConstPtr& source,
                          const std::string& encoding)
{
  return toCvShare(*source, source, encoding);
}

CvImageConstPtr toCvShare(const sensor_msgs::Image& source,
                          const boost::shared_ptr<void const>& tracked_object,
                          const std::string& encoding)
{
  // If the encoding different or the endianness different, you have to copy
  if ((!encoding.empty() && source.encoding != encoding) || (source.is_bigendian &&
      (boost::endian::order::native != boost::endian::order::big)))
    return toCvCopy(source, encoding);

  CvImagePtr ptr = boost::make_shared<CvImage>();
  ptr->header = source.header;
  ptr->encoding = source.encoding;
  ptr->tracked_object_ = tracked_object;
  ptr->image = matFromImage(source);
  return ptr;
}

CvImagePtr cvtColor(const CvImageConstPtr& source,
                    const std::string& encoding)
{
  return toCvCopyImpl(source->image, source->header, source->encoding, encoding);
}

/////////////////////////////////////// CompressedImage ///////////////////////////////////////////

cv::Mat matFromImage(const sensor_msgs::CompressedImage& source)
{
    cv::Mat jpegData(1,source.data.size(),CV_8UC1);
    jpegData.data     = const_cast<uchar*>(&source.data[0]);
    cv::InputArray data(jpegData);
    cv::Mat bgrMat     = cv::imdecode(data,cv::IMREAD_ANYCOLOR);
    return bgrMat;
}

sensor_msgs::CompressedImagePtr CvImage::toCompressedImageMsg(const Format dst_format) const
{
  sensor_msgs::CompressedImagePtr ptr = boost::make_shared<sensor_msgs::CompressedImage>();
  toCompressedImageMsg(*ptr,dst_format);
  return ptr;
}

std::string getFormat(Format format) {

	switch (format) {
		case DIB:
			return "dib";
		case BMP:
			return "bmp";
		case JPG:
			return "jpg";
		case JPEG:
			return "jpeg";
		case JPE:
			return "jpe";
		case JP2:
			return "jp2";
		case PNG:
			return "png";
		case PBM:
			return "pbm";
		case PGM:
			return "pgm";
		case PPM:
			return "ppm";
		case RAS:
			return "ras";
		case SR:
			return "sr";
		case TIF:
			return "tif";
		case TIFF:
			return "tiff";
	}

	throw Exception("Unrecognized image format");
}

void CvImage::toCompressedImageMsg(sensor_msgs::CompressedImage& ros_image, const Format dst_format) const
{
  ros_image.header = header;
  cv::Mat image;
  if(encoding != enc::BGR8)
  {
      CvImagePtr tempThis = boost::make_shared<CvImage>(*this);
      CvImagePtr temp = cvtColor(tempThis,enc::BGR8);
      image = temp->image;
  }
  else
  {
      image = this->image;
  }
  std::vector<uchar> buf;

  std::string format = getFormat(dst_format);
  ros_image.format = format;
  cv::imencode("." + format, image, buf);

  ros_image.data = buf;
}

// Deep copy data, returnee is mutable
CvImagePtr toCvCopy(const sensor_msgs::CompressedImageConstPtr& source,
                    const std::string& encoding)
{
  return toCvCopy(*source, encoding);
}

CvImagePtr toCvCopy(const sensor_msgs::CompressedImage& source,
                    const std::string& encoding)
{
  // Construct matrix pointing to source data
  return toCvCopyImpl(matFromImage(source), source.header, enc::BGR8, encoding);
}

CvImageConstPtr cvtColorForDisplay(const CvImageConstPtr& source,
                                   const std::string& encoding_out,
                                   const CvtColorForDisplayOptions options)
{
  double min_image_value = options.min_image_value;
  double max_image_value = options.max_image_value;

  if (!source)
    throw Exception("cv_bridge.cvtColorForDisplay() called with empty image.");
  // let's figure out what to do with the empty encoding
  std::string encoding = encoding_out;
  if (encoding.empty())
  {
    try
    {
      // Let's decide upon an output format
      if (enc::numChannels(source->encoding) == 1)
      {
        if ((source->encoding == enc::TYPE_32SC1) ||
            (enc::bitDepth(source->encoding) == 8) ||
            (enc::bitDepth(source->encoding) == 16) ||
            (enc::bitDepth(source->encoding) == 32))
          encoding = enc::BGR8;
        else
          throw std::runtime_error("Unsupported depth of the source encoding " + encoding);
      }
      else
      {
        // We choose BGR by default here as we assume people will use OpenCV
        if ((enc::bitDepth(source->encoding) == 8) ||
            (enc::bitDepth(source->encoding) == 16))
          encoding = enc::BGR8;
        else
          throw std::runtime_error("Unsupported depth of the source encoding " + encoding);
      }
    }
    // We could have cv_bridge exception or std_runtime_error from sensor_msgs::image_codings routines
    catch (const std::runtime_error& e)
    {
      throw Exception("cv_bridge.cvtColorForDisplay() output encoding is empty and cannot be guessed.");
    }
  }
  else
  {
    if ((!enc::isColor(encoding_out) && !enc::isMono(encoding_out)) ||
        (enc::bitDepth(encoding) != 8))
      throw Exception("cv_bridge.cvtColorForDisplay() does not have an output encoding that is color or mono, and has is bit in depth");

  }

  // Convert label to bgr image
  if (encoding == sensor_msgs::image_encodings::BGR8 &&
      source->encoding == enc::TYPE_32SC1)
  {
    CvImagePtr result(new CvImage());
    result->header = source->header;
    result->encoding = encoding;
    result->image = cv::Mat(source->image.rows, source->image.cols, CV_8UC3);
    for (size_t j = 0; j < source->image.rows; ++j) {
      for (size_t i = 0; i < source->image.cols; ++i) {
        int label = source->image.at<int>(j, i);
        if (label == options.bg_label) {  // background label
          result->image.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 0, 0);
        }
        else
        {
          cv::Vec3d rgb = rgb_colors::getRGBColor(label);
          // result image should be BGR
          result->image.at<cv::Vec3b>(j, i) = cv::Vec3b(int(rgb[2] * 255), int(rgb[1] * 255), int(rgb[0] * 255));
        }
      }
    }
    return result;
  }

  // Perform scaling if asked for
  if (options.do_dynamic_scaling)
  {
    cv::minMaxLoc(source->image, &min_image_value, &max_image_value);
    if (min_image_value == max_image_value)
    {
      CvImagePtr result(new CvImage());
      result->header = source->header;
      result->encoding = encoding;
      if (enc::bitDepth(encoding) == 1)
      {
        result->image = cv::Mat(source->image.size(), CV_8UC1);
        result->image.setTo(255./2.);
      } else {
        result->image = cv::Mat(source->image.size(), CV_8UC3);
        result->image.setTo(cv::Scalar(1., 1., 1.)*255./2.);
      }
      return result;
    }
  }

  if (min_image_value != max_image_value)
  {
    if (enc::numChannels(source->encoding) != 1)
      throw Exception("cv_bridge.cvtColorForDisplay() scaling for images with more than one channel is unsupported");
    CvImagePtr img_scaled(new CvImage());
    img_scaled->header = source->header;
    if (options.colormap == -1) {
      img_scaled->encoding = enc::MONO8;
      cv::Mat(source->image-min_image_value).convertTo(img_scaled->image, CV_8UC1, 255.0 /
        (max_image_value - min_image_value));
    } else {
      img_scaled->encoding = enc::BGR8;
      cv::Mat(source->image-min_image_value).convertTo(img_scaled->image, CV_8UC3, 255.0 /
        (max_image_value - min_image_value));
      cv::applyColorMap(img_scaled->image, img_scaled->image, options.colormap);
      // Fill black color to the nan region.
      if (source->encoding == enc::TYPE_32FC1) {
        for (size_t j = 0; j < source->image.rows; ++j) {
          for (size_t i = 0; i < source->image.cols; ++i) {
            float float_value = source->image.at<float>(j, i);
            if (std::isnan(float_value)) {
              img_scaled->image.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 0, 0);
            }
          }
        }
      }
    }
    return cvtColor(img_scaled, encoding);
  }

  // If no color conversion is possible, we must "guess" the input format
  CvImagePtr source_typed(new CvImage());
  source_typed->image = source->image;
  source_typed->header = source->header;
  source_typed->encoding = source->encoding;

  // If we get the OpenCV format, if we have 1,3 or 4 channels, we are most likely in mono, BGR or BGRA modes
  if (source->encoding == "CV_8UC1")
    source_typed->encoding = enc::MONO8;
  else if (source->encoding == "16UC1")
    source_typed->encoding = enc::MONO16;
  else if (source->encoding == "CV_8UC3")
    source_typed->encoding = enc::BGR8;
  else if (source->encoding == "CV_8UC4")
    source_typed->encoding = enc::BGRA8;
  else if (source->encoding == "CV_16UC3")
    source_typed->encoding = enc::BGR8;
  else if (source->encoding == "CV_16UC4")
    source_typed->encoding = enc::BGRA8;

  // If no conversion is needed, don't convert
  if (source_typed->encoding == encoding)
    return source;

  try
  {
    // Now that the output is a proper color format, try to see if any conversion is possible
    return cvtColor(source_typed, encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    throw Exception("cv_bridge.cvtColorForDisplay() while trying to convert image from '" + source->encoding + "' to '" + encoding + "' an exception was thrown (" + e.what() + ")");
  }
}

} //namespace cv_bridge
