#ifndef EQUIDISTANTCAMERA_H
#define EQUIDISTANTCAMERA_H

#include <opencv2/core/core.hpp>
#include <string>

#include "ceres/rotation.h"
#include "Camera.h"

namespace camodocal
{

/**
 * J. Kannala, and S. Brandt, A Generic Camera Model and Calibration Method
 * for Conventional, Wide-Angle, and Fish-Eye Lenses, PAMI 2006
 */

class EquidistantCamera: public Camera
{
public:
    class Parameters: public Camera::Parameters
    {
    public:
        Parameters();
        Parameters(const std::string& cameraName,
                   int w, int h,
                   double k2, double k3, double k4, double k5,
                   double mu, double mv,
                   double u0, double v0);

        double& k2(void);
        double& k3(void);
        double& k4(void);
        double& k5(void);
        double& mu(void);
        double& mv(void);
        double& u0(void);
        double& v0(void);

        double k2(void) const;
        double k3(void) const;
        double k4(void) const;
        double k5(void) const;
        double mu(void) const;
        double mv(void) const;
        double u0(void) const;
        double v0(void) const;

        bool readFromYamlFile(const std::string& filename);
        void writeToYamlFile(const std::string& filename) const;

        Parameters& operator=(const Parameters& other);
        friend std::ostream& operator<< (std::ostream& out, const Parameters& params);

    private:
        // projection
        double m_k2;
        double m_k3;
        double m_k4;
        double m_k5;

        double m_mu;
        double m_mv;
        double m_u0;
        double m_v0;
    };

    EquidistantCamera();

    /**
    * \brief Constructor from the projection model parameters
    */
    EquidistantCamera(const std::string& cameraName,
                      int imageWidth, int imageHeight,
                      double k2, double k3, double k4, double k5,
                      double mu, double mv,
                      double u0, double v0);
    /**
    * \brief Constructor from the projection model parameters
    */
    EquidistantCamera(const Parameters& params);

    Camera::ModelType modelType(void) const;
    const std::string& cameraName(void) const;
    int imageWidth(void) const;
    int imageHeight(void) const;

    void estimateIntrinsics(const cv::Size& boardSize,
                            const std::vector< std::vector<cv::Point3f> >& objectPoints,
                            const std::vector< std::vector<cv::Point2f> >& imagePoints);

    // Lift points from the image plane to the sphere
    virtual void liftSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;
    //%output P

    // Lift points from the image plane to the projective space
    void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;
    //%output P

    // Projects 3D points to the image plane (Pi function)
    void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const;
    //%output p

    // Projects 3D points to the image plane (Pi function)
    // and calculates jacobian
    void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p,
                      Eigen::Matrix<double,2,3>& J) const;
    //%output p
    //%output J

    void undistToPlane(const Eigen::Vector2d& p_u, Eigen::Vector2d& p) const;
    //%output p

    template <typename T>
    static void spaceToPlane(const T* const params,
                             const T* const q, const T* const t,
                             const Eigen::Matrix<T, 3, 1>& P,
                             Eigen::Matrix<T, 2, 1>& p);

    void initUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale = 1.0) const;
    cv::Mat initUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2,
                                    float fx = -1.0f, float fy = -1.0f,
                                    cv::Size imageSize = cv::Size(0, 0),
                                    float cx = -1.0f, float cy = -1.0f,
                                    cv::Mat rmat = cv::Mat::eye(3, 3, CV_32F)) const;

    int parameterCount(void) const;

    const Parameters& getParameters(void) const;
    void setParameters(const Parameters& parameters);

    void readParameters(const std::vector<double>& parameterVec);
    void writeParameters(std::vector<double>& parameterVec) const;

    void writeParametersToYamlFile(const std::string& filename) const;

    std::string parametersToString(void) const;

private:
    template<typename T>
    static T r(T k2, T k3, T k4, T k5, T theta);


    void fitOddPoly(const std::vector<double>& x, const std::vector<double>& y,
                    int n, std::vector<double>& coeffs) const;

    void backprojectSymmetric(const Eigen::Vector2d& p_u,
                              double& theta, double& phi) const;

    Parameters mParameters;

    double m_inv_K11, m_inv_K13, m_inv_K22, m_inv_K23;
};

typedef boost::shared_ptr<EquidistantCamera> EquidistantCameraPtr;
typedef boost::shared_ptr<const EquidistantCamera> EquidistantCameraConstPtr;

template<typename T>
T
EquidistantCamera::r(T k2, T k3, T k4, T k5, T theta)
{
    // k1 = 1
    return theta +
           k2 * theta * theta * theta +
           k3 * theta * theta * theta * theta * theta +
           k4 * theta * theta * theta * theta * theta * theta * theta +
           k5 * theta * theta * theta * theta * theta * theta * theta * theta * theta;
}

template <typename T>
void
EquidistantCamera::spaceToPlane(const T* const params,
                                const T* const q, const T* const t,
                                const Eigen::Matrix<T, 3, 1>& P,
                                Eigen::Matrix<T, 2, 1>& p)
{
    T P_w[3];
    P_w[0] = T(P(0));
    P_w[1] = T(P(1));
    P_w[2] = T(P(2));

    // Convert quaternion from Eigen convention (x, y, z, w)
    // to Ceres convention (w, x, y, z)
    T q_ceres[4] = {q[3], q[0], q[1], q[2]};

    T P_c[3];
    ceres::QuaternionRotatePoint(q_ceres, P_w, P_c);

    P_c[0] += t[0];
    P_c[1] += t[1];
    P_c[2] += t[2];

    // project 3D object point to the image plane;
    T k2 = params[0];
    T k3 = params[1];
    T k4 = params[2];
    T k5 = params[3];
    T mu = params[4];
    T mv = params[5];
    T u0 = params[6];
    T v0 = params[7];

    T len = sqrt(P_c[0] * P_c[0] + P_c[1] * P_c[1] + P_c[2] * P_c[2]);
    T theta = acos(P_c[2] / len);
    T phi = atan2(P_c[1], P_c[0]);

    Eigen::Matrix<T,2,1> p_u = r(k2, k3, k4, k5, theta) * Eigen::Matrix<T,2,1>(cos(phi), sin(phi));

    p(0) = mu * p_u(0) + u0;
    p(1) = mv * p_u(1) + v0;
}

}

#endif
