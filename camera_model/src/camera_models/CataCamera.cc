#include "camodocal/camera_models/CataCamera.h"

#include <cmath>
#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camodocal/gpl/gpl.h"

namespace camodocal
{

CataCamera::Parameters::Parameters()
 : Camera::Parameters(MEI)
 , m_xi(0.0)
 , m_k1(0.0)
 , m_k2(0.0)
 , m_p1(0.0)
 , m_p2(0.0)
 , m_gamma1(0.0)
 , m_gamma2(0.0)
 , m_u0(0.0)
 , m_v0(0.0)
{

}

CataCamera::Parameters::Parameters(const std::string& cameraName,
                                   int w, int h,
                                   double xi,
                                   double k1, double k2,
                                   double p1, double p2,
                                   double gamma1, double gamma2,
                                   double u0, double v0)
 : Camera::Parameters(MEI, cameraName, w, h)
 , m_xi(xi)
 , m_k1(k1)
 , m_k2(k2)
 , m_p1(p1)
 , m_p2(p2)
 , m_gamma1(gamma1)
 , m_gamma2(gamma2)
 , m_u0(u0)
 , m_v0(v0)
{
}

double&
CataCamera::Parameters::xi(void)
{
    return m_xi;
}

double&
CataCamera::Parameters::k1(void)
{
    return m_k1;
}

double&
CataCamera::Parameters::k2(void)
{
    return m_k2;
}

double&
CataCamera::Parameters::p1(void)
{
    return m_p1;
}

double&
CataCamera::Parameters::p2(void)
{
    return m_p2;
}

double&
CataCamera::Parameters::gamma1(void)
{
    return m_gamma1;
}

double&
CataCamera::Parameters::gamma2(void)
{
    return m_gamma2;
}

double&
CataCamera::Parameters::u0(void)
{
    return m_u0;
}

double&
CataCamera::Parameters::v0(void)
{
    return m_v0;
}

double
CataCamera::Parameters::xi(void) const
{
    return m_xi;
}

double
CataCamera::Parameters::k1(void) const
{
    return m_k1;
}

double
CataCamera::Parameters::k2(void) const
{
    return m_k2;
}

double
CataCamera::Parameters::p1(void) const
{
    return m_p1;
}

double
CataCamera::Parameters::p2(void) const
{
    return m_p2;
}

double
CataCamera::Parameters::gamma1(void) const
{
    return m_gamma1;
}

double
CataCamera::Parameters::gamma2(void) const
{
    return m_gamma2;
}

double
CataCamera::Parameters::u0(void) const
{
    return m_u0;
}

double
CataCamera::Parameters::v0(void) const
{
    return m_v0;
}

bool
CataCamera::Parameters::readFromYamlFile(const std::string& filename)
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

        if (sModelType.compare("MEI") != 0)
        {
            return false;
        }
    }

    m_modelType = MEI;
    fs["camera_name"] >> m_cameraName;
    m_imageWidth = static_cast<int>(fs["image_width"]);
    m_imageHeight = static_cast<int>(fs["image_height"]);

    cv::FileNode n = fs["mirror_parameters"];
    m_xi = static_cast<double>(n["xi"]);

    n = fs["distortion_parameters"];
    m_k1 = static_cast<double>(n["k1"]);
    m_k2 = static_cast<double>(n["k2"]);
    m_p1 = static_cast<double>(n["p1"]);
    m_p2 = static_cast<double>(n["p2"]);

    n = fs["projection_parameters"];
    m_gamma1 = static_cast<double>(n["gamma1"]);
    m_gamma2 = static_cast<double>(n["gamma2"]);
    m_u0 = static_cast<double>(n["u0"]);
    m_v0 = static_cast<double>(n["v0"]);

    return true;
}

void
CataCamera::Parameters::writeToYamlFile(const std::string& filename) const
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

    fs << "model_type" << "MEI";
    fs << "camera_name" << m_cameraName;
    fs << "image_width" << m_imageWidth;
    fs << "image_height" << m_imageHeight;

    // mirror: xi
    fs << "mirror_parameters";
    fs << "{" << "xi" << m_xi << "}";

    // radial distortion: k1, k2
    // tangential distortion: p1, p2
    fs << "distortion_parameters";
    fs << "{" << "k1" << m_k1
              << "k2" << m_k2
              << "p1" << m_p1
              << "p2" << m_p2 << "}";

    // projection: gamma1, gamma2, u0, v0
    fs << "projection_parameters";
    fs << "{" << "gamma1" << m_gamma1
              << "gamma2" << m_gamma2
              << "u0" << m_u0
              << "v0" << m_v0 << "}";

    fs.release();
}

CataCamera::Parameters&
CataCamera::Parameters::operator=(const CataCamera::Parameters& other)
{
    if (this != &other)
    {
        m_modelType = other.m_modelType;
        m_cameraName = other.m_cameraName;
        m_imageWidth = other.m_imageWidth;
        m_imageHeight = other.m_imageHeight;
        m_xi = other.m_xi;
        m_k1 = other.m_k1;
        m_k2 = other.m_k2;
        m_p1 = other.m_p1;
        m_p2 = other.m_p2;
        m_gamma1 = other.m_gamma1;
        m_gamma2 = other.m_gamma2;
        m_u0 = other.m_u0;
        m_v0 = other.m_v0;
    }

    return *this;
}

std::ostream&
operator<< (std::ostream& out, const CataCamera::Parameters& params)
{
    out << "Camera Parameters:" << std::endl;
    out << "    model_type " << "MEI" << std::endl;
    out << "   camera_name " << params.m_cameraName << std::endl;
    out << "   image_width " << params.m_imageWidth << std::endl;
    out << "  image_height " << params.m_imageHeight << std::endl;

    out << "Mirror Parameters" << std::endl;
    out << std::fixed << std::setprecision(10);
    out << "            xi " << params.m_xi << std::endl;

    // radial distortion: k1, k2
    // tangential distortion: p1, p2
    out << "Distortion Parameters" << std::endl;
    out << "            k1 " << params.m_k1 << std::endl
        << "            k2 " << params.m_k2 << std::endl
        << "            p1 " << params.m_p1 << std::endl
        << "            p2 " << params.m_p2 << std::endl;

    // projection: gamma1, gamma2, u0, v0
    out << "Projection Parameters" << std::endl;
    out << "        gamma1 " << params.m_gamma1 << std::endl
        << "        gamma2 " << params.m_gamma2 << std::endl
        << "            u0 " << params.m_u0 << std::endl
        << "            v0 " << params.m_v0 << std::endl;

    return out;
}

CataCamera::CataCamera()
 : m_inv_K11(1.0)
 , m_inv_K13(0.0)
 , m_inv_K22(1.0)
 , m_inv_K23(0.0)
 , m_noDistortion(true)
{

}

CataCamera::CataCamera(const std::string& cameraName,
                       int imageWidth, int imageHeight,
                       double xi, double k1, double k2, double p1, double p2,
                       double gamma1, double gamma2, double u0, double v0)
 : mParameters(cameraName, imageWidth, imageHeight,
               xi, k1, k2, p1, p2, gamma1, gamma2, u0, v0)
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
    m_inv_K11 = 1.0 / mParameters.gamma1();
    m_inv_K13 = -mParameters.u0() / mParameters.gamma1();
    m_inv_K22 = 1.0 / mParameters.gamma2();
    m_inv_K23 = -mParameters.v0() / mParameters.gamma2();
}

CataCamera::CataCamera(const CataCamera::Parameters& params)
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
    m_inv_K11 = 1.0 / mParameters.gamma1();
    m_inv_K13 = -mParameters.u0() / mParameters.gamma1();
    m_inv_K22 = 1.0 / mParameters.gamma2();
    m_inv_K23 = -mParameters.v0() / mParameters.gamma2();
}

Camera::ModelType
CataCamera::modelType(void) const
{
    return mParameters.modelType();
}

const std::string&
CataCamera::cameraName(void) const
{
    return mParameters.cameraName();
}

int
CataCamera::imageWidth(void) const
{
    return mParameters.imageWidth();
}

int
CataCamera::imageHeight(void) const
{
    return mParameters.imageHeight();
}

void
CataCamera::estimateIntrinsics(const cv::Size& boardSize,
                               const std::vector< std::vector<cv::Point3f> >& objectPoints,
                               const std::vector< std::vector<cv::Point2f> >& imagePoints)
{
    Parameters params = getParameters();

    double u0 = params.imageWidth() / 2.0;
    double v0 = params.imageHeight() / 2.0;

    double gamma0 = 0.0;
    double minReprojErr = std::numeric_limits<double>::max();

    std::vector<cv::Mat> rvecs, tvecs;
    rvecs.assign(objectPoints.size(), cv::Mat());
    tvecs.assign(objectPoints.size(), cv::Mat());

    params.xi() = 1.0;
    params.k1() = 0.0;
    params.k2() = 0.0;
    params.p1() = 0.0;
    params.p2() = 0.0;
    params.u0() = u0;
    params.v0() = v0;

    // Initialize gamma (focal length)
    // Use non-radial line image and xi = 1
    for (size_t i = 0; i < imagePoints.size(); ++i)
    {
        for (int r = 0; r < boardSize.height; ++r)
        {
            cv::Mat P(boardSize.width, 4, CV_64F);
            for (int c = 0; c < boardSize.width; ++c)
            {
                const cv::Point2f& imagePoint = imagePoints.at(i).at(r * boardSize.width + c);

                double u = imagePoint.x - u0;
                double v = imagePoint.y - v0;

                P.at<double>(c, 0) = u;
                P.at<double>(c, 1) = v;
                P.at<double>(c, 2) = 0.5;
                P.at<double>(c, 3) = -0.5 * (square(u) + square(v));
            }

            cv::Mat C;
            cv::SVD::solveZ(P, C);

            double t = square(C.at<double>(0)) + square(C.at<double>(1)) + C.at<double>(2) * C.at<double>(3);
            if (t < 0.0)
            {
                continue;
            }

            // check that line image is not radial
            double d = sqrt(1.0 / t);
            double nx = C.at<double>(0) * d;
            double ny = C.at<double>(1) * d;
            if (hypot(nx, ny) > 0.95)
            {
                continue;
            }

            double gamma = sqrt(C.at<double>(2) / C.at<double>(3));

            params.gamma1() = gamma;
            params.gamma2() = gamma;
            setParameters(params);

            for (size_t j = 0; j < objectPoints.size(); ++j)
            {
                estimateExtrinsics(objectPoints.at(j), imagePoints.at(j), rvecs.at(j), tvecs.at(j));
            }

            double reprojErr = reprojectionError(objectPoints, imagePoints, rvecs, tvecs, cv::noArray());

            if (reprojErr < minReprojErr)
            {
                minReprojErr = reprojErr;
                gamma0 = gamma;
            }
        }
    }

    if (gamma0 <= 0.0 && minReprojErr >= std::numeric_limits<double>::max())
    {
        std::cout << "[" << params.cameraName() << "] "
                  << "# INFO: CataCamera model fails with given data. " << std::endl;

        return;
    }

    params.gamma1() = gamma0;
    params.gamma2() = gamma0;
    setParameters(params);
}

/** 
 * \brief Lifts a point from the image plane to the unit sphere
 *
 * \param p image coordinates
 * \param P coordinates of the point on the sphere
 */
void
CataCamera::liftSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P) const
{
    double mx_d, my_d,mx2_d, mxy_d, my2_d, mx_u, my_u;
    double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;
    double lambda;

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
        // Apply inverse distortion model
        if (0)
        {
            double k1 = mParameters.k1();
            double k2 = mParameters.k2();
            double p1 = mParameters.p1();
            double p2 = mParameters.p2();

            // Inverse distortion model
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
            int n = 6;
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

    // Lift normalised points to the sphere (inv_hslash)
    double xi = mParameters.xi();
    if (xi == 1.0)
    {
        lambda = 2.0 / (mx_u * mx_u + my_u * my_u + 1.0);
        P << lambda * mx_u, lambda * my_u, lambda - 1.0;
    }
    else
    {
        lambda = (xi + sqrt(1.0 + (1.0 - xi * xi) * (mx_u * mx_u + my_u * my_u))) / (1.0 + mx_u * mx_u + my_u * my_u);
        P << lambda * mx_u, lambda * my_u, lambda - xi;
    }
}

/** 
 * \brief Lifts a point from the image plane to its projective ray
 *
 * \param p image coordinates
 * \param P coordinates of the projective ray
 */
void
CataCamera::liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const
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
    double xi = mParameters.xi();
    if (xi == 1.0)
    {
        P << mx_u, my_u, (1.0 - mx_u * mx_u - my_u * my_u) / 2.0;
    }
    else
    {
        // Reuse variable
        rho2_d = mx_u * mx_u + my_u * my_u;
        P << mx_u, my_u, 1.0 - xi * (rho2_d + 1.0) / (xi + sqrt(1.0 + (1.0 - xi * xi) * rho2_d));
    }
}


/** 
 * \brief Project a 3D point (\a x,\a y,\a z) to the image plane in (\a u,\a v)
 *
 * \param P 3D point coordinates
 * \param p return value, contains the image point coordinates
 */
void
CataCamera::spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const
{
    Eigen::Vector2d p_u, p_d;

    // Project points to the normalised plane
    double z = P(2) + mParameters.xi() * P.norm();
    p_u << P(0) / z, P(1) / z;

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
    p << mParameters.gamma1() * p_d(0) + mParameters.u0(),
         mParameters.gamma2() * p_d(1) + mParameters.v0();
}

#if 0
/** 
 * \brief Project a 3D point to the image plane and calculate Jacobian
 *
 * \param P 3D point coordinates
 * \param p return value, contains the image point coordinates
 */
void
CataCamera::spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p,
                        Eigen::Matrix<double,2,3>& J) const
{
    double xi = mParameters.xi();

    Eigen::Vector2d p_u, p_d;
    double norm, inv_denom;
    double dxdmx, dydmx, dxdmy, dydmy;

    norm = P.norm();
    // Project points to the normalised plane
    inv_denom = 1.0 / (P(2) + xi * norm);
    p_u << inv_denom * P(0), inv_denom * P(1);

    // Calculate jacobian
    inv_denom = inv_denom * inv_denom / norm;
    double dudx = inv_denom * (norm * P(2) + xi * (P(1) * P(1) + P(2) * P(2)));
    double dvdx = -inv_denom * xi * P(0) * P(1);
    double dudy = dvdx;
    double dvdy = inv_denom * (norm * P(2) + xi * (P(0) * P(0) + P(2) * P(2)));
    inv_denom = inv_denom * (-xi * P(2) - norm); // reuse variable
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

    double gamma1 = mParameters.gamma1();
    double gamma2 = mParameters.gamma2();

    // Make the product of the jacobians
    // and add projection matrix jacobian
    inv_denom = gamma1 * (dudx * dxdmx + dvdx * dxdmy); // reuse
    dvdx = gamma2 * (dudx * dydmx + dvdx * dydmy);
    dudx = inv_denom;

    inv_denom = gamma1 * (dudy * dxdmx + dvdy * dxdmy); // reuse
    dvdy = gamma2 * (dudy * dydmx + dvdy * dydmy);
    dudy = inv_denom;

    inv_denom = gamma1 * (dudz * dxdmx + dvdz * dxdmy); // reuse
    dvdz = gamma2 * (dudz * dydmx + dvdz * dydmy);
    dudz = inv_denom;
    
    // Apply generalised projection matrix
    p << gamma1 * p_d(0) + mParameters.u0(),
         gamma2 * p_d(1) + mParameters.v0();

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
CataCamera::undistToPlane(const Eigen::Vector2d& p_u, Eigen::Vector2d& p) const
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
    p << mParameters.gamma1() * p_d(0) + mParameters.u0(),
         mParameters.gamma2() * p_d(1) + mParameters.v0();
}

/** 
 * \brief Apply distortion to input point (from the normalised plane)
 *  
 * \param p_u undistorted coordinates of point on the normalised plane
 * \return to obtain the distorted point: p_d = p_u + d_u
 */
void
CataCamera::distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const
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
CataCamera::distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u,
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
CataCamera::initUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale) const
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

cv::Mat
CataCamera::initUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2,
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

    if (cx == -1.0f && cy == -1.0f)
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
        K_rect(0,0) = mParameters.gamma1();
        K_rect(1,1) = mParameters.gamma2();
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
CataCamera::parameterCount(void) const
{
    return 9;
}

const CataCamera::Parameters&
CataCamera::getParameters(void) const
{
    return mParameters;
}

void
CataCamera::setParameters(const CataCamera::Parameters& parameters)
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

    m_inv_K11 = 1.0 / mParameters.gamma1();
    m_inv_K13 = -mParameters.u0() / mParameters.gamma1();
    m_inv_K22 = 1.0 / mParameters.gamma2();
    m_inv_K23 = -mParameters.v0() / mParameters.gamma2();
}

void
CataCamera::readParameters(const std::vector<double>& parameterVec)
{
    if ((int)parameterVec.size() != parameterCount())
    {
        return;
    }

    Parameters params = getParameters();

    params.xi() = parameterVec.at(0);
    params.k1() = parameterVec.at(1);
    params.k2() = parameterVec.at(2);
    params.p1() = parameterVec.at(3);
    params.p2() = parameterVec.at(4);
    params.gamma1() = parameterVec.at(5);
    params.gamma2() = parameterVec.at(6);
    params.u0() = parameterVec.at(7);
    params.v0() = parameterVec.at(8);

    setParameters(params);
}

void
CataCamera::writeParameters(std::vector<double>& parameterVec) const
{
    parameterVec.resize(parameterCount());
    parameterVec.at(0) = mParameters.xi();
    parameterVec.at(1) = mParameters.k1();
    parameterVec.at(2) = mParameters.k2();
    parameterVec.at(3) = mParameters.p1();
    parameterVec.at(4) = mParameters.p2();
    parameterVec.at(5) = mParameters.gamma1();
    parameterVec.at(6) = mParameters.gamma2();
    parameterVec.at(7) = mParameters.u0();
    parameterVec.at(8) = mParameters.v0();
}

void
CataCamera::writeParametersToYamlFile(const std::string& filename) const
{
    mParameters.writeToYamlFile(filename);
}

std::string
CataCamera::parametersToString(void) const
{
    std::ostringstream oss;
    oss << mParameters;

    return oss.str();
}

}
