#include "camodocal/camera_models/PinholeCamera.h"

#include <cmath>
#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camodocal/gpl/gpl.h"

namespace camodocal
{

PinholeCamera::Parameters::Parameters()
 : Camera::Parameters(PINHOLE)
 , m_k1(0.0)
 , m_k2(0.0)
 , m_p1(0.0)
 , m_p2(0.0)
 , m_fx(0.0)
 , m_fy(0.0)
 , m_cx(0.0)
 , m_cy(0.0)
{

}

PinholeCamera::Parameters::Parameters(const std::string& cameraName,
                                      int w, int h,
                                      double k1, double k2,
                                      double p1, double p2,
                                      double fx, double fy,
                                      double cx, double cy)
 : Camera::Parameters(PINHOLE, cameraName, w, h)
 , m_k1(k1)
 , m_k2(k2)
 , m_p1(p1)
 , m_p2(p2)
 , m_fx(fx)
 , m_fy(fy)
 , m_cx(cx)
 , m_cy(cy)
{
}

double&
PinholeCamera::Parameters::k1(void)
{
    return m_k1;
}

double&
PinholeCamera::Parameters::k2(void)
{
    return m_k2;
}

double&
PinholeCamera::Parameters::p1(void)
{
    return m_p1;
}

double&
PinholeCamera::Parameters::p2(void)
{
    return m_p2;
}

double&
PinholeCamera::Parameters::fx(void)
{
    return m_fx;
}

double&
PinholeCamera::Parameters::fy(void)
{
    return m_fy;
}

double&
PinholeCamera::Parameters::cx(void)
{
    return m_cx;
}

double&
PinholeCamera::Parameters::cy(void)
{
    return m_cy;
}

double
PinholeCamera::Parameters::k1(void) const
{
    return m_k1;
}

double
PinholeCamera::Parameters::k2(void) const
{
    return m_k2;
}

double
PinholeCamera::Parameters::p1(void) const
{
    return m_p1;
}

double
PinholeCamera::Parameters::p2(void) const
{
    return m_p2;
}

double
PinholeCamera::Parameters::fx(void) const
{
    return m_fx;
}

double
PinholeCamera::Parameters::fy(void) const
{
    return m_fy;
}

double
PinholeCamera::Parameters::cx(void) const
{
    return m_cx;
}

double
PinholeCamera::Parameters::cy(void) const
{
    return m_cy;
}

bool
PinholeCamera::Parameters::readFromYamlFile(const std::string& filename)
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

        if (sModelType.compare("PINHOLE") != 0)
        {
            return false;
        }
    }

    m_modelType = PINHOLE;
    fs["camera_name"] >> m_cameraName;
    m_imageWidth = static_cast<int>(fs["image_width"]);
    m_imageHeight = static_cast<int>(fs["image_height"]);

    cv::FileNode n = fs["distortion_parameters"];
    m_k1 = static_cast<double>(n["k1"]);
    m_k2 = static_cast<double>(n["k2"]);
    m_p1 = static_cast<double>(n["p1"]);
    m_p2 = static_cast<double>(n["p2"]);

    n = fs["projection_parameters"];
    m_fx = static_cast<double>(n["fx"]);
    m_fy = static_cast<double>(n["fy"]);
    m_cx = static_cast<double>(n["cx"]);
    m_cy = static_cast<double>(n["cy"]);

    return true;
}

void
PinholeCamera::Parameters::writeToYamlFile(const std::string& filename) const
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

    fs << "model_type" << "PINHOLE";
    fs << "camera_name" << m_cameraName;
    fs << "image_width" << m_imageWidth;
    fs << "image_height" << m_imageHeight;

    // radial distortion: k1, k2
    // tangential distortion: p1, p2
    fs << "distortion_parameters";
    fs << "{" << "k1" << m_k1
              << "k2" << m_k2
              << "p1" << m_p1
              << "p2" << m_p2 << "}";

    // projection: fx, fy, cx, cy
    fs << "projection_parameters";
    fs << "{" << "fx" << m_fx
              << "fy" << m_fy
              << "cx" << m_cx
              << "cy" << m_cy << "}";

    fs.release();
}

PinholeCamera::Parameters&
PinholeCamera::Parameters::operator=(const PinholeCamera::Parameters& other)
{
    if (this != &other)
    {
        m_modelType = other.m_modelType;
        m_cameraName = other.m_cameraName;
        m_imageWidth = other.m_imageWidth;
        m_imageHeight = other.m_imageHeight;
        m_k1 = other.m_k1;
        m_k2 = other.m_k2;
        m_p1 = other.m_p1;
        m_p2 = other.m_p2;
        m_fx = other.m_fx;
        m_fy = other.m_fy;
        m_cx = other.m_cx;
        m_cy = other.m_cy;
    }

    return *this;
}

std::ostream&
operator<< (std::ostream& out, const PinholeCamera::Parameters& params)
{
    out << "Camera Parameters:" << std::endl;
    out << "    model_type " << "PINHOLE" << std::endl;
    out << "   camera_name " << params.m_cameraName << std::endl;
    out << "   image_width " << params.m_imageWidth << std::endl;
    out << "  image_height " << params.m_imageHeight << std::endl;

    // radial distortion: k1, k2
    // tangential distortion: p1, p2
    out << "Distortion Parameters" << std::endl;
    out << "            k1 " << params.m_k1 << std::endl
        << "            k2 " << params.m_k2 << std::endl
        << "            p1 " << params.m_p1 << std::endl
        << "            p2 " << params.m_p2 << std::endl;

    // projection: fx, fy, cx, cy
    out << "Projection Parameters" << std::endl;
    out << "            fx " << params.m_fx << std::endl
        << "            fy " << params.m_fy << std::endl
        << "            cx " << params.m_cx << std::endl
        << "            cy " << params.m_cy << std::endl;

    return out;
}

PinholeCamera::PinholeCamera()
 : m_inv_K11(1.0)
 , m_inv_K13(0.0)
 , m_inv_K22(1.0)
 , m_inv_K23(0.0)
 , m_noDistortion(true)
{

}

PinholeCamera::PinholeCamera(const std::string& cameraName,
                             int imageWidth, int imageHeight,
                             double k1, double k2, double p1, double p2,
                             double fx, double fy, double cx, double cy)
 : mParameters(cameraName, imageWidth, imageHeight,
               k1, k2, p1, p2, fx, fy, cx, cy)
{
    if ((mParameters.k1() == 0.0) &&
        (mParameters.k2() == 0.0) &&
        (mParameters.p1() == 0.0) &&
        (mParameters.p2() == 0.0))
    {
        m_noDistortion = true;
    }
    else
    {
        m_noDistortion = false;
    }

    // Inverse camera projection matrix parameters
    m_inv_K11 = 1.0 / mParameters.fx();
    m_inv_K13 = -mParameters.cx() / mParameters.fx();
    m_inv_K22 = 1.0 / mParameters.fy();
    m_inv_K23 = -mParameters.cy() / mParameters.fy();
}

PinholeCamera::PinholeCamera(const PinholeCamera::Parameters& params)
 : mParameters(params)
{
    if ((mParameters.k1() == 0.0) &&
        (mParameters.k2() == 0.0) &&
        (mParameters.p1() == 0.0) &&
        (mParameters.p2() == 0.0))
    {
        m_noDistortion = true;
    }
    else
    {
        m_noDistortion = false;
    }

    // Inverse camera projection matrix parameters
    m_inv_K11 = 1.0 / mParameters.fx();
    m_inv_K13 = -mParameters.cx() / mParameters.fx();
    m_inv_K22 = 1.0 / mParameters.fy();
    m_inv_K23 = -mParameters.cy() / mParameters.fy();
}

Camera::ModelType
PinholeCamera::modelType(void) const
{
    return mParameters.modelType();
}

const std::string&
PinholeCamera::cameraName(void) const
{
    return mParameters.cameraName();
}

int
PinholeCamera::imageWidth(void) const
{
    return mParameters.imageWidth();
}

int
PinholeCamera::imageHeight(void) const
{
    return mParameters.imageHeight();
}

void
PinholeCamera::estimateIntrinsics(const cv::Size& boardSize,
                                  const std::vector< std::vector<cv::Point3f> >& objectPoints,
                                  const std::vector< std::vector<cv::Point2f> >& imagePoints)
{
    // Z. Zhang, A Flexible New Technique for Camera Calibration, PAMI 2000

    Parameters params = getParameters();

    params.k1() = 0.0;
    params.k2() = 0.0;
    params.p1() = 0.0;
    params.p2() = 0.0;

    double cx = params.imageWidth() / 2.0;
    double cy = params.imageHeight() / 2.0;
    params.cx() = cx;
    params.cy() = cy;

    size_t nImages = imagePoints.size();

    cv::Mat A(nImages * 2, 2, CV_64F);
    cv::Mat b(nImages * 2, 1, CV_64F);

    for (size_t i = 0; i < nImages; ++i)
    {
        const std::vector<cv::Point3f>& oPoints = objectPoints.at(i);

        std::vector<cv::Point2f> M(oPoints.size());
        for (size_t j = 0; j < M.size(); ++j)
        {
            M.at(j) = cv::Point2f(oPoints.at(j).x, oPoints.at(j).y);
        }

        cv::Mat H = cv::findHomography(M, imagePoints.at(i));

        H.at<double>(0,0) -= H.at<double>(2,0) * cx;
        H.at<double>(0,1) -= H.at<double>(2,1) * cx;
        H.at<double>(0,2) -= H.at<double>(2,2) * cx;
        H.at<double>(1,0) -= H.at<double>(2,0) * cy;
        H.at<double>(1,1) -= H.at<double>(2,1) * cy;
        H.at<double>(1,2) -= H.at<double>(2,2) * cy;

        double h[3], v[3], d1[3], d2[3];
        double n[4] = {0,0,0,0};

        for (int j = 0; j < 3; ++j)
        {
            double t0 = H.at<double>(j,0);
            double t1 = H.at<double>(j,1);
            h[j] = t0; v[j] = t1;
            d1[j] = (t0 + t1) * 0.5;
            d2[j] = (t0 - t1) * 0.5;
            n[0] += t0 * t0; n[1] += t1 * t1;
            n[2] += d1[j] * d1[j]; n[3] += d2[j] * d2[j];
        }

        for (int j = 0; j < 4; ++j)
        {
            n[j] = 1.0 / sqrt(n[j]);
        }

        for (int j = 0; j < 3; ++j)
        {
            h[j] *= n[0]; v[j] *= n[1];
            d1[j] *= n[2]; d2[j] *= n[3];
        }

        A.at<double>(i * 2, 0) = h[0] * v[0];
        A.at<double>(i * 2, 1) = h[1] * v[1];
        A.at<double>(i * 2 + 1, 0) = d1[0] * d2[0];
        A.at<double>(i * 2 + 1, 1) = d1[1] * d2[1];
        b.at<double>(i * 2, 0) = -h[2] * v[2];
        b.at<double>(i * 2 + 1, 0) = -d1[2] * d2[2];
    }

    cv::Mat f(2, 1, CV_64F);
    cv::solve(A, b, f, cv::DECOMP_NORMAL | cv::DECOMP_LU);

    params.fx() = sqrt(fabs(1.0 / f.at<double>(0)));
    params.fy() = sqrt(fabs(1.0 / f.at<double>(1)));

    setParameters(params);
}

/**
 * \brief Lifts a point from the image plane to the unit sphere
 *
 * \param p image coordinates
 * \param P coordinates of the point on the sphere
 */
void
PinholeCamera::liftSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P) const
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
PinholeCamera::liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const
{
    double mx_d, my_d,mx2_d, mxy_d, my2_d, mx_u, my_u;
    double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;
    //double lambda;

    // Lift points to normalised plane
    mx_d = m_inv_K11 * p(0) + m_inv_K13;
    my_d = m_inv_K22 * p(1) + m_inv_K23;

    if (m_noDistortion)
    {
        mx_u = mx_d;
        my_u = my_d;
    }
    else
    {
        if (0)
        {
            double k1 = mParameters.k1();
            double k2 = mParameters.k2();
            double p1 = mParameters.p1();
            double p2 = mParameters.p2();

            // Apply inverse distortion model
            // proposed by Heikkila
            mx2_d = mx_d*mx_d;
            my2_d = my_d*my_d;
            mxy_d = mx_d*my_d;
            rho2_d = mx2_d+my2_d;
            rho4_d = rho2_d*rho2_d;
            radDist_d = k1*rho2_d+k2*rho4_d;
            Dx_d = mx_d*radDist_d + p2*(rho2_d+2*mx2_d) + 2*p1*mxy_d;
            Dy_d = my_d*radDist_d + p1*(rho2_d+2*my2_d) + 2*p2*mxy_d;
            inv_denom_d = 1/(1+4*k1*rho2_d+6*k2*rho4_d+8*p1*my_d+8*p2*mx_d);

            mx_u = mx_d - inv_denom_d*Dx_d;
            my_u = my_d - inv_denom_d*Dy_d;
        }
        else
        {
            // Recursive distortion model
            int n = 8;
            Eigen::Vector2d d_u;
            distortion(Eigen::Vector2d(mx_d, my_d), d_u);
            // Approximate value
            mx_u = mx_d - d_u(0);
            my_u = my_d - d_u(1);

            for (int i = 1; i < n; ++i)
            {
                distortion(Eigen::Vector2d(mx_u, my_u), d_u);
                mx_u = mx_d - d_u(0);
                my_u = my_d - d_u(1);
            }
        }
    }

    // Obtain a projective ray
    P << mx_u, my_u, 1.0;
}


/**
 * \brief Project a 3D point (\a x,\a y,\a z) to the image plane in (\a u,\a v)
 *
 * \param P 3D point coordinates
 * \param p return value, contains the image point coordinates
 */
void
PinholeCamera::spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const
{
    Eigen::Vector2d p_u, p_d;

    // Project points to the normalised plane
    p_u << P(0) / P(2), P(1) / P(2);

    if (m_noDistortion)
    {
        p_d = p_u;
    }
    else
    {
        // Apply distortion
        Eigen::Vector2d d_u;
        distortion(p_u, d_u);
        p_d = p_u + d_u;
    }

    // Apply generalised projection matrix
    p << mParameters.fx() * p_d(0) + mParameters.cx(),
         mParameters.fy() * p_d(1) + mParameters.cy();
}

#if 0
/**
 * \brief Project a 3D point to the image plane and calculate Jacobian
 *
 * \param P 3D point coordinates
 * \param p return value, contains the image point coordinates
 */
void
PinholeCamera::spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p,
                            Eigen::Matrix<double,2,3>& J) const
{
    Eigen::Vector2d p_u, p_d;
    double norm, inv_denom;
    double dxdmx, dydmx, dxdmy, dydmy;

    norm = P.norm();
    // Project points to the normalised plane
    inv_denom = 1.0 / P(2);
    p_u << inv_denom * P(0), inv_denom * P(1);

    // Calculate jacobian
    double dudx = inv_denom;
    double dvdx = 0.0;
    double dudy = 0.0;
    double dvdy = inv_denom;
    inv_denom = - inv_denom * inv_denom;
    double dudz = P(0) * inv_denom;
    double dvdz = P(1) * inv_denom;

    if (m_noDistortion)
    {
        p_d = p_u;
    }
    else
    {
        // Apply distortion
        Eigen::Vector2d d_u;
        distortion(p_u, d_u);
        p_d = p_u + d_u;
    }

    double fx = mParameters.fx();
    double fy = mParameters.fy();

    // Make the product of the jacobians
    // and add projection matrix jacobian
    inv_denom = fx * (dudx * dxdmx + dvdx * dxdmy); // reuse
    dvdx = fy * (dudx * dydmx + dvdx * dydmy);
    dudx = inv_denom;

    inv_denom = fx * (dudy * dxdmx + dvdy * dxdmy); // reuse
    dvdy = fy * (dudy * dydmx + dvdy * dydmy);
    dudy = inv_denom;

    inv_denom = fx * (dudz * dxdmx + dvdz * dxdmy); // reuse
    dvdz = fy * (dudz * dydmx + dvdz * dydmy);
    dudz = inv_denom;

    // Apply generalised projection matrix
    p << fx * p_d(0) + mParameters.cx(),
         fy * p_d(1) + mParameters.cy();

    J << dudx, dudy, dudz,
         dvdx, dvdy, dvdz;
}
#endif

/**
 * \brief Projects an undistorted 2D point p_u to the image plane
 *
 * \param p_u 2D point coordinates
 * \return image point coordinates
 */
void
PinholeCamera::undistToPlane(const Eigen::Vector2d& p_u, Eigen::Vector2d& p) const
{
    Eigen::Vector2d p_d;

    if (m_noDistortion)
    {
        p_d = p_u;
    }
    else
    {
        // Apply distortion
        Eigen::Vector2d d_u;
        distortion(p_u, d_u);
        p_d = p_u + d_u;
    }

    // Apply generalised projection matrix
    p << mParameters.fx() * p_d(0) + mParameters.cx(),
         mParameters.fy() * p_d(1) + mParameters.cy();
}

/**
 * \brief Apply distortion to input point (from the normalised plane)
 *
 * \param p_u undistorted coordinates of point on the normalised plane
 * \return to obtain the distorted point: p_d = p_u + d_u
 */
void
PinholeCamera::distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const
{
    double k1 = mParameters.k1();
    double k2 = mParameters.k2();
    double p1 = mParameters.p1();
    double p2 = mParameters.p2();

    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = p_u(0) * p_u(0);
    my2_u = p_u(1) * p_u(1);
    mxy_u = p_u(0) * p_u(1);
    rho2_u = mx2_u + my2_u;
    rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
    d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
           p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}

/**
 * \brief Apply distortion to input point (from the normalised plane)
 *        and calculate Jacobian
 *
 * \param p_u undistorted coordinates of point on the normalised plane
 * \return to obtain the distorted point: p_d = p_u + d_u
 */
void
PinholeCamera::distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u,
                          Eigen::Matrix2d& J) const
{
    double k1 = mParameters.k1();
    double k2 = mParameters.k2();
    double p1 = mParameters.p1();
    double p2 = mParameters.p2();

    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = p_u(0) * p_u(0);
    my2_u = p_u(1) * p_u(1);
    mxy_u = p_u(0) * p_u(1);
    rho2_u = mx2_u + my2_u;
    rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
    d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
           p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);

    double dxdmx = 1.0 + rad_dist_u + k1 * 2.0 * mx2_u + k2 * rho2_u * 4.0 * mx2_u + 2.0 * p1 * p_u(1) + 6.0 * p2 * p_u(0);
    double dydmx = k1 * 2.0 * p_u(0) * p_u(1) + k2 * 4.0 * rho2_u * p_u(0) * p_u(1) + p1 * 2.0 * p_u(0) + 2.0 * p2 * p_u(1);
    double dxdmy = dydmx;
    double dydmy = 1.0 + rad_dist_u + k1 * 2.0 * my2_u + k2 * rho2_u * 4.0 * my2_u + 6.0 * p1 * p_u(1) + 2.0 * p2 * p_u(0);

    J << dxdmx, dxdmy,
         dydmx, dydmy;
}

void
PinholeCamera::initUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale) const
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

            Eigen::Vector3d P;
            P << mx_u, my_u, 1.0;

            Eigen::Vector2d p;
            spaceToPlane(P, p);

            mapX.at<float>(v,u) = p(0);
            mapY.at<float>(v,u) = p(1);
        }
    }

    cv::convertMaps(mapX, mapY, map1, map2, CV_32FC1, false);
}

cv::Mat
PinholeCamera::initUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2,
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

    Eigen::Matrix3f R, R_inv;
    cv::cv2eigen(rmat, R);
    R_inv = R.inverse();

    // assume no skew
    Eigen::Matrix3f K_rect;

    if (cx == -1.0f || cy == -1.0f)
    {
        K_rect << fx, 0, imageSize.width / 2,
                  0, fy, imageSize.height / 2,
                  0, 0, 1;
    }
    else
    {
        K_rect << fx, 0, cx,
                  0, fy, cy,
                  0, 0, 1;
    }

    if (fx == -1.0f || fy == -1.0f)
    {
        K_rect(0,0) = mParameters.fx();
        K_rect(1,1) = mParameters.fy();
    }

    Eigen::Matrix3f K_rect_inv = K_rect.inverse();

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
PinholeCamera::parameterCount(void) const
{
    return 8;
}

const PinholeCamera::Parameters&
PinholeCamera::getParameters(void) const
{
    return mParameters;
}

void
PinholeCamera::setParameters(const PinholeCamera::Parameters& parameters)
{
    mParameters = parameters;

    if ((mParameters.k1() == 0.0) &&
        (mParameters.k2() == 0.0) &&
        (mParameters.p1() == 0.0) &&
        (mParameters.p2() == 0.0))
    {
        m_noDistortion = true;
    }
    else
    {
        m_noDistortion = false;
    }

    m_inv_K11 = 1.0 / mParameters.fx();
    m_inv_K13 = -mParameters.cx() / mParameters.fx();
    m_inv_K22 = 1.0 / mParameters.fy();
    m_inv_K23 = -mParameters.cy() / mParameters.fy();
}

void
PinholeCamera::readParameters(const std::vector<double>& parameterVec)
{
    if ((int)parameterVec.size() != parameterCount())
    {
        return;
    }

    Parameters params = getParameters();

    params.k1() = parameterVec.at(0);
    params.k2() = parameterVec.at(1);
    params.p1() = parameterVec.at(2);
    params.p2() = parameterVec.at(3);
    params.fx() = parameterVec.at(4);
    params.fy() = parameterVec.at(5);
    params.cx() = parameterVec.at(6);
    params.cy() = parameterVec.at(7);

    setParameters(params);
}

void
PinholeCamera::writeParameters(std::vector<double>& parameterVec) const
{
    parameterVec.resize(parameterCount());
    parameterVec.at(0) = mParameters.k1();
    parameterVec.at(1) = mParameters.k2();
    parameterVec.at(2) = mParameters.p1();
    parameterVec.at(3) = mParameters.p2();
    parameterVec.at(4) = mParameters.fx();
    parameterVec.at(5) = mParameters.fy();
    parameterVec.at(6) = mParameters.cx();
    parameterVec.at(7) = mParameters.cy();
}

void
PinholeCamera::writeParametersToYamlFile(const std::string& filename) const
{
    mParameters.writeToYamlFile(filename);
}

std::string
PinholeCamera::parametersToString(void) const
{
    std::ostringstream oss;
    oss << mParameters;

    return oss.str();
}

}
