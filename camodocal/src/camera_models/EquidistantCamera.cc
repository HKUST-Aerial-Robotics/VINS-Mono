#include "camodocal/camera_models/EquidistantCamera.h"

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

EquidistantCamera::Parameters::Parameters()
 : Camera::Parameters(KANNALA_BRANDT)
 , m_k2(0.0)
 , m_k3(0.0)
 , m_k4(0.0)
 , m_k5(0.0)
 , m_mu(0.0)
 , m_mv(0.0)
 , m_u0(0.0)
 , m_v0(0.0)
{

}

EquidistantCamera::Parameters::Parameters(const std::string& cameraName,
                                          int w, int h,
                                          double k2, double k3, double k4, double k5,
                                          double mu, double mv,
                                          double u0, double v0)
 : Camera::Parameters(KANNALA_BRANDT, cameraName, w, h)
 , m_k2(k2)
 , m_k3(k3)
 , m_k4(k4)
 , m_k5(k5)
 , m_mu(mu)
 , m_mv(mv)
 , m_u0(u0)
 , m_v0(v0)
{

}

double&
EquidistantCamera::Parameters::k2(void)
{
    return m_k2;
}

double&
EquidistantCamera::Parameters::k3(void)
{
    return m_k3;
}

double&
EquidistantCamera::Parameters::k4(void)
{
    return m_k4;
}

double&
EquidistantCamera::Parameters::k5(void)
{
    return m_k5;
}

double&
EquidistantCamera::Parameters::mu(void)
{
    return m_mu;
}

double&
EquidistantCamera::Parameters::mv(void)
{
    return m_mv;
}

double&
EquidistantCamera::Parameters::u0(void)
{
    return m_u0;
}

double&
EquidistantCamera::Parameters::v0(void)
{
    return m_v0;
}

double
EquidistantCamera::Parameters::k2(void) const
{
    return m_k2;
}

double
EquidistantCamera::Parameters::k3(void) const
{
    return m_k3;
}

double
EquidistantCamera::Parameters::k4(void) const
{
    return m_k4;
}

double
EquidistantCamera::Parameters::k5(void) const
{
    return m_k5;
}

double
EquidistantCamera::Parameters::mu(void) const
{
    return m_mu;
}

double
EquidistantCamera::Parameters::mv(void) const
{
    return m_mv;
}

double
EquidistantCamera::Parameters::u0(void) const
{
    return m_u0;
}

double
EquidistantCamera::Parameters::v0(void) const
{
    return m_v0;
}

bool
EquidistantCamera::Parameters::readFromYamlFile(const std::string& filename)
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

        if (sModelType.compare("KANNALA_BRANDT") != 0)
        {
            return false;
        }
    }

    m_modelType = KANNALA_BRANDT;
    fs["camera_name"] >> m_cameraName;
    m_imageWidth = static_cast<int>(fs["image_width"]);
    m_imageHeight = static_cast<int>(fs["image_height"]);

    cv::FileNode n = fs["projection_parameters"];
    m_k2 = static_cast<double>(n["k2"]);
    m_k3 = static_cast<double>(n["k3"]);
    m_k4 = static_cast<double>(n["k4"]);
    m_k5 = static_cast<double>(n["k5"]);
    m_mu = static_cast<double>(n["mu"]);
    m_mv = static_cast<double>(n["mv"]);
    m_u0 = static_cast<double>(n["u0"]);
    m_v0 = static_cast<double>(n["v0"]);

    return true;
}

void
EquidistantCamera::Parameters::writeToYamlFile(const std::string& filename) const
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

    fs << "model_type" << "KANNALA_BRANDT";
    fs << "camera_name" << m_cameraName;
    fs << "image_width" << m_imageWidth;
    fs << "image_height" << m_imageHeight;

    // projection: k2, k3, k4, k5, mu, mv, u0, v0
    fs << "projection_parameters";
    fs << "{" << "k2" << m_k2
              << "k3" << m_k3
              << "k4" << m_k4
              << "k5" << m_k5
              << "mu" << m_mu
              << "mv" << m_mv
              << "u0" << m_u0
              << "v0" << m_v0 << "}";

    fs.release();
}

EquidistantCamera::Parameters&
EquidistantCamera::Parameters::operator=(const EquidistantCamera::Parameters& other)
{
    if (this != &other)
    {
        m_modelType = other.m_modelType;
        m_cameraName = other.m_cameraName;
        m_imageWidth = other.m_imageWidth;
        m_imageHeight = other.m_imageHeight;
        m_k2 = other.m_k2;
        m_k3 = other.m_k3;
        m_k4 = other.m_k4;
        m_k5 = other.m_k5;
        m_mu = other.m_mu;
        m_mv = other.m_mv;
        m_u0 = other.m_u0;
        m_v0 = other.m_v0;
    }

    return *this;
}

std::ostream&
operator<< (std::ostream& out, const EquidistantCamera::Parameters& params)
{
    out << "Camera Parameters:" << std::endl;
    out << "    model_type " << "KANNALA_BRANDT" << std::endl;
    out << "   camera_name " << params.m_cameraName << std::endl;
    out << "   image_width " << params.m_imageWidth << std::endl;
    out << "  image_height " << params.m_imageHeight << std::endl;

    // projection: k2, k3, k4, k5, mu, mv, u0, v0
    out << "Projection Parameters" << std::endl;
    out << "            k2 " << params.m_k2 << std::endl
        << "            k3 " << params.m_k3 << std::endl
        << "            k4 " << params.m_k4 << std::endl
        << "            k5 " << params.m_k5 << std::endl
        << "            mu " << params.m_mu << std::endl
        << "            mv " << params.m_mv << std::endl
        << "            u0 " << params.m_u0 << std::endl
        << "            v0 " << params.m_v0 << std::endl;

    return out;
}

EquidistantCamera::EquidistantCamera()
 : m_inv_K11(1.0)
 , m_inv_K13(0.0)
 , m_inv_K22(1.0)
 , m_inv_K23(0.0)
{

}

EquidistantCamera::EquidistantCamera(const std::string& cameraName,
                                     int imageWidth, int imageHeight,
                                     double k2, double k3, double k4, double k5,
                                     double mu, double mv,
                                     double u0, double v0)
 : mParameters(cameraName, imageWidth, imageHeight,
               k2, k3, k4, k5, mu, mv, u0, v0)
{
    // Inverse camera projection matrix parameters
    m_inv_K11 = 1.0 / mParameters.mu();
    m_inv_K13 = -mParameters.u0() / mParameters.mu();
    m_inv_K22 = 1.0 / mParameters.mv();
    m_inv_K23 = -mParameters.v0() / mParameters.mv();
}

EquidistantCamera::EquidistantCamera(const EquidistantCamera::Parameters& params)
 : mParameters(params)
{
    // Inverse camera projection matrix parameters
    m_inv_K11 = 1.0 / mParameters.mu();
    m_inv_K13 = -mParameters.u0() / mParameters.mu();
    m_inv_K22 = 1.0 / mParameters.mv();
    m_inv_K23 = -mParameters.v0() / mParameters.mv();
}

Camera::ModelType
EquidistantCamera::modelType(void) const
{
    return mParameters.modelType();
}

const std::string&
EquidistantCamera::cameraName(void) const
{
    return mParameters.cameraName();
}

int
EquidistantCamera::imageWidth(void) const
{
    return mParameters.imageWidth();
}

int
EquidistantCamera::imageHeight(void) const
{
    return mParameters.imageHeight();
}

void
EquidistantCamera::estimateIntrinsics(const cv::Size& boardSize,
                                      const std::vector< std::vector<cv::Point3f> >& objectPoints,
                                      const std::vector< std::vector<cv::Point2f> >& imagePoints)
{
    Parameters params = getParameters();

    double u0 = params.imageWidth() / 2.0;
    double v0 = params.imageHeight() / 2.0;

    double minReprojErr = std::numeric_limits<double>::max();

    std::vector<cv::Mat> rvecs, tvecs;
    rvecs.assign(objectPoints.size(), cv::Mat());
    tvecs.assign(objectPoints.size(), cv::Mat());

    params.k2() = 0.0;
    params.k3() = 0.0;
    params.k4() = 0.0;
    params.k5() = 0.0;
    params.u0() = u0;
    params.v0() = v0;

    // Initialize focal length
    // C. Hughes, P. Denny, M. Glavin, and E. Jones,
    // Equidistant Fish-Eye Calibration and Rectification by Vanishing Point
    // Extraction, PAMI 2010
    // Find circles from rows of chessboard corners, and for each pair
    // of circles, find vanishing points: v1 and v2.
    // f = ||v1 - v2|| / PI;
    double f0 = 0.0;
    for (size_t i = 0; i < imagePoints.size(); ++i)
    {
        std::vector<Eigen::Vector2d> center(boardSize.height);
        double radius[boardSize.height];
        for (int r = 0; r < boardSize.height; ++r)
        {
            std::vector<cv::Point2d> circle;
            for (int c = 0; c < boardSize.width; ++c)
            {
                circle.push_back(imagePoints.at(i).at(r * boardSize.width + c));
            }

            fitCircle(circle, center[r](0), center[r](1), radius[r]);
        }

        for (int j = 0; j < boardSize.height; ++j)
        {
            for (int k = j + 1; k < boardSize.height; ++k)
            {
                // find distance between pair of vanishing points which
                // correspond to intersection points of 2 circles
                std::vector<cv::Point2d> ipts;
                ipts = intersectCircles(center[j](0), center[j](1), radius[j],
                                        center[k](0), center[k](1), radius[k]);

                if (ipts.size() < 2)
                {
                    continue;
                }

                double f = cv::norm(ipts.at(0) - ipts.at(1)) / M_PI;

                params.mu() = f;
                params.mv() = f;

                setParameters(params);

                for (size_t l = 0; l < objectPoints.size(); ++l)
                {
                    estimateExtrinsics(objectPoints.at(l), imagePoints.at(l), rvecs.at(l), tvecs.at(l));
                }

                double reprojErr = reprojectionError(objectPoints, imagePoints, rvecs, tvecs, cv::noArray());

                if (reprojErr < minReprojErr)
                {
                    minReprojErr = reprojErr;
                    f0 = f;
                }
            }
        }
    }

    if (f0 <= 0.0 && minReprojErr >= std::numeric_limits<double>::max())
    {
        std::cout << "[" << params.cameraName() << "] "
                  << "# INFO: kannala-Brandt model fails with given data. " << std::endl;

        return;
    }

    params.mu() = f0;
    params.mv() = f0;

    setParameters(params);
}

/**
 * \brief Lifts a point from the image plane to the unit sphere
 *
 * \param p image coordinates
 * \param P coordinates of the point on the sphere
 */
void
EquidistantCamera::liftSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P) const
{
    liftProjective(p, P);
}

/** 
 * \brief Lifts a point from the image plane to its projective ray
 *
 * \param p image coordinates
 * \param P coordinates of the projective ray
 */
void
EquidistantCamera::liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const
{
    // Lift points to normalised plane
    Eigen::Vector2d p_u;
    p_u << m_inv_K11 * p(0) + m_inv_K13,
           m_inv_K22 * p(1) + m_inv_K23;

    // Obtain a projective ray
    double theta, phi;
    backprojectSymmetric(p_u, theta, phi);

    P(0) = sin(theta) * cos(phi);
    P(1) = sin(theta) * sin(phi);
    P(2) = cos(theta);
}

/** 
 * \brief Project a 3D point (\a x,\a y,\a z) to the image plane in (\a u,\a v)
 *
 * \param P 3D point coordinates
 * \param p return value, contains the image point coordinates
 */
void
EquidistantCamera::spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const
{
    double theta = acos(P(2) / P.norm());
    double phi = atan2(P(1), P(0));

    Eigen::Vector2d p_u = r(mParameters.k2(), mParameters.k3(), mParameters.k4(), mParameters.k5(), theta) * Eigen::Vector2d(cos(phi), sin(phi));

    // Apply generalised projection matrix
    p << mParameters.mu() * p_u(0) + mParameters.u0(),
         mParameters.mv() * p_u(1) + mParameters.v0();
}


/** 
 * \brief Project a 3D point to the image plane and calculate Jacobian
 *
 * \param P 3D point coordinates
 * \param p return value, contains the image point coordinates
 */
void
EquidistantCamera::spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p,
                                Eigen::Matrix<double,2,3>& J) const
{
    double theta = acos(P(2) / P.norm());
    double phi = atan2(P(1), P(0));

    Eigen::Vector2d p_u = r(mParameters.k2(), mParameters.k3(), mParameters.k4(), mParameters.k5(), theta) * Eigen::Vector2d(cos(phi), sin(phi));

    // Apply generalised projection matrix
    p << mParameters.mu() * p_u(0) + mParameters.u0(),
         mParameters.mv() * p_u(1) + mParameters.v0();
}

/** 
 * \brief Projects an undistorted 2D point p_u to the image plane
 *
 * \param p_u 2D point coordinates
 * \return image point coordinates
 */
void
EquidistantCamera::undistToPlane(const Eigen::Vector2d& p_u, Eigen::Vector2d& p) const
{
//    Eigen::Vector2d p_d;
//
//    if (m_noDistortion)
//    {
//        p_d = p_u;
//    }
//    else
//    {
//        // Apply distortion
//        Eigen::Vector2d d_u;
//        distortion(p_u, d_u);
//        p_d = p_u + d_u;
//    }
//
//    // Apply generalised projection matrix
//    p << mParameters.gamma1() * p_d(0) + mParameters.u0(),
//         mParameters.gamma2() * p_d(1) + mParameters.v0();
}

void
EquidistantCamera::initUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale) const
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

            double theta, phi;
            backprojectSymmetric(Eigen::Vector2d(mx_u, my_u), theta, phi);

            Eigen::Vector3d P;
            P << sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta);

            Eigen::Vector2d p;
            spaceToPlane(P, p);

            mapX.at<float>(v,u) = p(0);
            mapY.at<float>(v,u) = p(1);
        }
    }

    cv::convertMaps(mapX, mapY, map1, map2, CV_32FC1, false);
}

cv::Mat
EquidistantCamera::initUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2,
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
        K_rect(0,0) = mParameters.mu();
        K_rect(1,1) = mParameters.mv();
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
EquidistantCamera::parameterCount(void) const
{
    return 8;
}

const EquidistantCamera::Parameters&
EquidistantCamera::getParameters(void) const
{
    return mParameters;
}

void
EquidistantCamera::setParameters(const EquidistantCamera::Parameters& parameters)
{
    mParameters = parameters;

    // Inverse camera projection matrix parameters
    m_inv_K11 = 1.0 / mParameters.mu();
    m_inv_K13 = -mParameters.u0() / mParameters.mu();
    m_inv_K22 = 1.0 / mParameters.mv();
    m_inv_K23 = -mParameters.v0() / mParameters.mv();
}

void
EquidistantCamera::readParameters(const std::vector<double>& parameterVec)
{
    if (parameterVec.size() != parameterCount())
    {
        return;
    }

    Parameters params = getParameters();

    params.k2() = parameterVec.at(0);
    params.k3() = parameterVec.at(1);
    params.k4() = parameterVec.at(2);
    params.k5() = parameterVec.at(3);
    params.mu() = parameterVec.at(4);
    params.mv() = parameterVec.at(5);
    params.u0() = parameterVec.at(6);
    params.v0() = parameterVec.at(7);

    setParameters(params);
}

void
EquidistantCamera::writeParameters(std::vector<double>& parameterVec) const
{
    parameterVec.resize(parameterCount());
    parameterVec.at(0) = mParameters.k2();
    parameterVec.at(1) = mParameters.k3();
    parameterVec.at(2) = mParameters.k4();
    parameterVec.at(3) = mParameters.k5();
    parameterVec.at(4) = mParameters.mu();
    parameterVec.at(5) = mParameters.mv();
    parameterVec.at(6) = mParameters.u0();
    parameterVec.at(7) = mParameters.v0();
}

void
EquidistantCamera::writeParametersToYamlFile(const std::string& filename) const
{
    mParameters.writeToYamlFile(filename);
}

std::string
EquidistantCamera::parametersToString(void) const
{
    std::ostringstream oss;
    oss << mParameters;

    return oss.str();
}

void
EquidistantCamera::fitOddPoly(const std::vector<double>& x, const std::vector<double>& y,
                              int n, std::vector<double>& coeffs) const
{
    std::vector<int> pows;
    for (int i = 1; i <= n; i += 2)
    {
        pows.push_back(i);
    }

    Eigen::MatrixXd X(x.size(), pows.size());
    Eigen::MatrixXd Y(y.size(), 1);
    for (size_t i = 0; i < x.size(); ++i)
    {
        for (size_t j = 0; j < pows.size(); ++j)
        {
            X(i,j) = pow(x.at(i), pows.at(j));
        }
        Y(i,0) = y.at(i);
    }

    Eigen::MatrixXd A = (X.transpose() * X).inverse() * X.transpose() * Y;

    coeffs.resize(A.rows());
    for (int i = 0; i < A.rows(); ++i)
    {
        coeffs.at(i) = A(i,0);
    }
}

void
EquidistantCamera::backprojectSymmetric(const Eigen::Vector2d& p_u,
                                        double& theta, double& phi) const
{
    double tol = 1e-10;
    double p_u_norm = p_u.norm();

    if (p_u_norm < 1e-10)
    {
        phi = 0.0;
    }
    else
    {
        phi = atan2(p_u(1), p_u(0));
    }

    int npow = 9;
    if (mParameters.k5() == 0.0)
    {
        npow -= 2;
    }
    if (mParameters.k4() == 0.0)
    {
        npow -= 2;
    }
    if (mParameters.k3() == 0.0)
    {
        npow -= 2;
    }
    if (mParameters.k2() == 0.0)
    {
        npow -= 2;
    }

    Eigen::MatrixXd coeffs(npow + 1, 1);
    coeffs.setZero();
    coeffs(0) = -p_u_norm;
    coeffs(1) = 1.0;

    if (npow >= 3)
    {
        coeffs(3) = mParameters.k2();
    }
    if (npow >= 5)
    {
        coeffs(5) = mParameters.k3();
    }
    if (npow >= 7)
    {
        coeffs(7) = mParameters.k4();
    }
    if (npow >= 9)
    {
        coeffs(9) = mParameters.k5();
    }

    if (npow == 1)
    {
        theta = p_u_norm;
    }
    else
    {
        // Get eigenvalues of companion matrix corresponding to polynomial.
        // Eigenvalues correspond to roots of polynomial.
        Eigen::MatrixXd A(npow, npow);
        A.setZero();
        A.block(1, 0, npow - 1, npow - 1).setIdentity();
        A.col(npow - 1) = - coeffs.block(0, 0, npow, 1) / coeffs(npow);

        Eigen::EigenSolver<Eigen::MatrixXd> es(A);
        Eigen::MatrixXcd eigval = es.eigenvalues();

        std::vector<double> thetas;
        for (int i = 0; i < eigval.rows(); ++i)
        {
            if (fabs(eigval(i).imag()) > tol)
            {
                continue;
            }

            double t = eigval(i).real();

            if (t < -tol)
            {
                continue;
            }
            else if (t < 0.0)
            {
                t = 0.0;
            }

            thetas.push_back(t);
        }

        if (thetas.empty())
        {
            theta = p_u_norm;
        }
        else
        {
            theta = *std::min_element(thetas.begin(), thetas.end());
        }
    }
}

}
