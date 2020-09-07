#include "camodocal/camera_models/ScaramuzzaCamera.h"

#include <cmath>
#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include "camodocal/gpl/gpl.h"

namespace camodocal
{

OCAMCamera::Parameters::Parameters()
 : Camera::Parameters(SCARAMUZZA)
 , m_C(0.0)
 , m_D(0.0)
 , m_E(0.0)
 , m_center_x(0.0)
 , m_center_y(0.0)
{
    memset(m_poly, 0, sizeof(double) * SCARAMUZZA_POLY_SIZE);
    memset(m_inv_poly, 0, sizeof(double) * SCARAMUZZA_INV_POLY_SIZE);
}



bool
OCAMCamera::Parameters::readFromYamlFile(const std::string& filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        return false;
    }

    if (!fs["model_type"].isNone())
    {
        std::string sModelType;
        fs["model_type"] >> sModelType;

        if (!boost::iequals(sModelType, "scaramuzza"))
        {
            return false;
        }
    }

    m_modelType = SCARAMUZZA;
    fs["camera_name"] >> m_cameraName;
    m_imageWidth = static_cast<int>(fs["image_width"]);
    m_imageHeight = static_cast<int>(fs["image_height"]);

    cv::FileNode n = fs["poly_parameters"];
    for(int i=0; i < SCARAMUZZA_POLY_SIZE; i++)
        m_poly[i] = static_cast<double>(n[std::string("p") + boost::lexical_cast<std::string>(i)]);

    n = fs["inv_poly_parameters"];
    for(int i=0; i < SCARAMUZZA_INV_POLY_SIZE; i++)
        m_inv_poly[i] = static_cast<double>(n[std::string("p") + boost::lexical_cast<std::string>(i)]);

    n = fs["affine_parameters"];
    m_C = static_cast<double>(n["ac"]);
    m_D = static_cast<double>(n["ad"]);
    m_E = static_cast<double>(n["ae"]);

    m_center_x = static_cast<double>(n["cx"]);
    m_center_y = static_cast<double>(n["cy"]);

    return true;
}

void
OCAMCamera::Parameters::writeToYamlFile(const std::string& filename) const
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

    fs << "model_type" << "scaramuzza";
    fs << "camera_name" << m_cameraName;
    fs << "image_width" << m_imageWidth;
    fs << "image_height" << m_imageHeight;

    fs << "poly_parameters";
    fs << "{";
    for(int i=0; i < SCARAMUZZA_POLY_SIZE; i++)
        fs << std::string("p") + boost::lexical_cast<std::string>(i) << m_poly[i];
    fs << "}";

    fs << "inv_poly_parameters";
    fs << "{";
    for(int i=0; i < SCARAMUZZA_INV_POLY_SIZE; i++)
        fs << std::string("p") + boost::lexical_cast<std::string>(i) << m_inv_poly[i];
    fs << "}";

    fs << "affine_parameters";
    fs << "{" << "ac" << m_C
              << "ad" << m_D
              << "ae" << m_E
              << "cx" << m_center_x
              << "cy" << m_center_y << "}";

    fs.release();
}

OCAMCamera::Parameters&
OCAMCamera::Parameters::operator=(const OCAMCamera::Parameters& other)
{
    if (this != &other)
    {
        m_modelType = other.m_modelType;
        m_cameraName = other.m_cameraName;
        m_imageWidth = other.m_imageWidth;
        m_imageHeight = other.m_imageHeight;
        m_C = other.m_C;
        m_D = other.m_D;
        m_E = other.m_E;
        m_center_x = other.m_center_x;
        m_center_y = other.m_center_y;

        memcpy(m_poly, other.m_poly, sizeof(double) * SCARAMUZZA_POLY_SIZE);
        memcpy(m_inv_poly, other.m_inv_poly, sizeof(double) * SCARAMUZZA_INV_POLY_SIZE);
    }

    return *this;
}

std::ostream&
operator<< (std::ostream& out, const OCAMCamera::Parameters& params)
{
    out << "Camera Parameters:" << std::endl;
    out << "    model_type " << "scaramuzza" << std::endl;
    out << "   camera_name " << params.m_cameraName << std::endl;
    out << "   image_width " << params.m_imageWidth << std::endl;
    out << "  image_height " << params.m_imageHeight << std::endl;

    out << std::fixed << std::setprecision(10);

    out << "Poly Parameters" << std::endl;
    for(int i=0; i < SCARAMUZZA_POLY_SIZE; i++)
        out << std::string("p") + boost::lexical_cast<std::string>(i) << params.m_poly[i] << std::endl;

    out << "Inverse Poly Parameters" << std::endl;
    for(int i=0; i < SCARAMUZZA_INV_POLY_SIZE; i++)
        out << std::string("p") + boost::lexical_cast<std::string>(i) << params.m_inv_poly[i] << std::endl;

    out << "Affine Parameters" << std::endl;
    out << "            ac " << params.m_C << std::endl
        << "            ad " << params.m_D << std::endl
        << "            ae " << params.m_E << std::endl;
    out << "            cx " << params.m_center_x << std::endl
        << "            cy " << params.m_center_y << std::endl;

    return out;
}

OCAMCamera::OCAMCamera()
 : m_inv_scale(0.0)
{

}

OCAMCamera::OCAMCamera(const OCAMCamera::Parameters& params)
 : mParameters(params)
{
    m_inv_scale = 1.0 / (params.C() - params.D() * params.E());
}

Camera::ModelType
OCAMCamera::modelType(void) const
{
    return mParameters.modelType();
}

const std::string&
OCAMCamera::cameraName(void) const
{
    return mParameters.cameraName();
}

int
OCAMCamera::imageWidth(void) const
{
    return mParameters.imageWidth();
}

int
OCAMCamera::imageHeight(void) const
{
    return mParameters.imageHeight();
}

void
OCAMCamera::estimateIntrinsics(const cv::Size& /*boardSize*/,
                               const std::vector< std::vector<cv::Point3f> >& /*objectPoints*/,
                               const std::vector< std::vector<cv::Point2f> >& /*imagePoints*/)
{
    std::cout << "OCAMCamera::estimateIntrinsics - NOT IMPLEMENTED" << std::endl;
    throw std::string("OCAMCamera::estimateIntrinsics - NOT IMPLEMENTED");
}

/** 
 * \brief Lifts a point from the image plane to the unit sphere
 *
 * \param p image coordinates
 * \param P coordinates of the point on the sphere
 */
void
OCAMCamera::liftSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P) const
{
    liftProjective(p, P);
    P.normalize();
}

/** 
 * \brief Lifts a point from the image plane to its projective ray
 *
 * \param p image coordinates
 * \param P coordinates of the projective ray
 */
void
OCAMCamera::liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const
{
    // Relative to Center
    Eigen::Vector2d xc(p[0] - mParameters.center_x(), p[1] - mParameters.center_y());

    // Affine Transformation
    // xc_a = inv(A) * xc;
    Eigen::Vector2d xc_a(
        m_inv_scale * (xc[0] - mParameters.D() * xc[1]),
        m_inv_scale * (-mParameters.E() * xc[0] + mParameters.C() * xc[1])
    );

    double phi = std::sqrt(xc_a[0] * xc_a[0] + xc_a[1] * xc_a[1]);
    double phi_i = 1.0;
    double z = 0.0;

    for (int i = 0; i < SCARAMUZZA_POLY_SIZE; i++)
    {
        z += phi_i * mParameters.poly(i);
        phi_i *= phi;
    }

    P << xc[0], xc[1], -z;
}


/** 
 * \brief Project a 3D point (\a x,\a y,\a z) to the image plane in (\a u,\a v)
 *
 * \param P 3D point coordinates
 * \param p return value, contains the image point coordinates
 */
void
OCAMCamera::spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const
{
    double norm = std::sqrt(P[0] * P[0] + P[1] * P[1]);
    double theta = std::atan2(-P[2], norm);
    double rho = 0.0;
    double theta_i = 1.0;

    for (int i = 0; i < SCARAMUZZA_INV_POLY_SIZE; i++)
    {
        rho += theta_i * mParameters.inv_poly(i);
        theta_i *= theta;
    }

    double invNorm = 1.0 / norm;
    Eigen::Vector2d xn(
        P[0] * invNorm * rho,
        P[1] * invNorm * rho
    );

    p << xn[0] * mParameters.C() + xn[1] * mParameters.D() + mParameters.center_x(),
         xn[0] * mParameters.E() + xn[1]                   + mParameters.center_y();
}


/** 
 * \brief Projects an undistorted 2D point p_u to the image plane
 *
 * \param p_u 2D point coordinates
 * \return image point coordinates
 */
void
OCAMCamera::undistToPlane(const Eigen::Vector2d& p_u, Eigen::Vector2d& p) const
{
    Eigen::Vector3d P(p_u[0], p_u[1], 1.0);
    spaceToPlane(P, p);
}


#if 0
void
OCAMCamera::initUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale) const
{
    cv::Size imageSize(mParameters.imageWidth(), mParameters.imageHeight());

    cv::Mat mapX = cv::Mat::zeros(imageSize, CV_32F);
    cv::Mat mapY = cv::Mat::zeros(imageSize, CV_32F);

    for (int v = 0; v < imageSize.height; ++v)
    {
        for (int u = 0; u < imageSize.width; ++u)
        {
            double mx_u = m_inv_K11 / fScale * u + m_inv_K13 / fScale;
            double my_u = m_inv_K22 / fScale * v + m_inv_K23 / fScale;

            double xi = mParameters.xi();
            double d2 = mx_u * mx_u + my_u * my_u;

            Eigen::Vector3d P;
            P << mx_u, my_u, 1.0 - xi * (d2 + 1.0) / (xi + sqrt(1.0 + (1.0 - xi * xi) * d2));

            Eigen::Vector2d p;
            spaceToPlane(P, p);

            mapX.at<float>(v,u) = p(0);
            mapY.at<float>(v,u) = p(1);
        }
    }

    cv::convertMaps(mapX, mapY, map1, map2, CV_32FC1, false);
}
#endif

cv::Mat
OCAMCamera::initUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2,
                                    float fx, float fy,
                                    cv::Size imageSize,
                                    float cx, float cy,
                                    cv::Mat rmat) const
{
    if (imageSize == cv::Size(0, 0))
    {
        imageSize = cv::Size(mParameters.imageWidth(), mParameters.imageHeight());
    }

    cv::Mat mapX = cv::Mat::zeros(imageSize.height, imageSize.width, CV_32F);
    cv::Mat mapY = cv::Mat::zeros(imageSize.height, imageSize.width, CV_32F);

    Eigen::Matrix3f K_rect;

    K_rect << fx, 0, cx < 0 ? imageSize.width / 2 : cx,
              0, fy, cy < 0 ? imageSize.height / 2 : cy,
              0, 0, 1;

    if (fx < 0 || fy < 0)
    {
        throw std::string(std::string(__FUNCTION__) + ": Focal length must be specified");
    }

    Eigen::Matrix3f K_rect_inv = K_rect.inverse();

    Eigen::Matrix3f R, R_inv;
    cv::cv2eigen(rmat, R);
    R_inv = R.inverse();

    for (int v = 0; v < imageSize.height; ++v)
    {
        for (int u = 0; u < imageSize.width; ++u)
        {
            Eigen::Vector3f xo;
            xo << u, v, 1;

            Eigen::Vector3f uo = R_inv * K_rect_inv * xo;

            Eigen::Vector2d p;
            spaceToPlane(uo.cast<double>(), p);

            mapX.at<float>(v,u) = p(0);
            mapY.at<float>(v,u) = p(1);
        }
    }

    cv::convertMaps(mapX, mapY, map1, map2, CV_32FC1, false);

    cv::Mat K_rect_cv;
    cv::eigen2cv(K_rect, K_rect_cv);
    return K_rect_cv;
}

int
OCAMCamera::parameterCount(void) const
{
    return SCARAMUZZA_CAMERA_NUM_PARAMS;
}

const OCAMCamera::Parameters&
OCAMCamera::getParameters(void) const
{
    return mParameters;
}

void
OCAMCamera::setParameters(const OCAMCamera::Parameters& parameters)
{
    mParameters = parameters;

    m_inv_scale = 1.0 / (parameters.C() - parameters.D() * parameters.E());
}

void
OCAMCamera::readParameters(const std::vector<double>& parameterVec)
{
    if ((int)parameterVec.size() != parameterCount())
    {
        return;
    }

    Parameters params = getParameters();

    params.C() = parameterVec.at(0);
    params.D() = parameterVec.at(1);
    params.E() = parameterVec.at(2);
    params.center_x() = parameterVec.at(3);
    params.center_y() = parameterVec.at(4);
    for (int i=0; i < SCARAMUZZA_POLY_SIZE; i++)
        params.poly(i) = parameterVec.at(5+i);
    for (int i=0; i < SCARAMUZZA_INV_POLY_SIZE; i++)
        params.inv_poly(i) = parameterVec.at(5 + SCARAMUZZA_POLY_SIZE + i);

    setParameters(params);
}

void
OCAMCamera::writeParameters(std::vector<double>& parameterVec) const
{
    parameterVec.resize(parameterCount());
    parameterVec.at(0) = mParameters.C();
    parameterVec.at(1) = mParameters.D();
    parameterVec.at(2) = mParameters.E();
    parameterVec.at(3) = mParameters.center_x();
    parameterVec.at(4) = mParameters.center_y();
    for (int i=0; i < SCARAMUZZA_POLY_SIZE; i++)
        parameterVec.at(5+i) = mParameters.poly(i);
    for (int i=0; i < SCARAMUZZA_INV_POLY_SIZE; i++)
        parameterVec.at(5 + SCARAMUZZA_POLY_SIZE + i) = mParameters.inv_poly(i);
}

void
OCAMCamera::writeParametersToYamlFile(const std::string& filename) const
{
    mParameters.writeToYamlFile(filename);
}

std::string
OCAMCamera::parametersToString(void) const
{
    std::ostringstream oss;
    oss << mParameters;

    return oss.str();
}

}
