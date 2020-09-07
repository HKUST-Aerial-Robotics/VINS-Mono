#ifndef SCARAMUZZACAMERA_H
#define SCARAMUZZACAMERA_H

#include <opencv2/core/core.hpp>
#include <string>

#include "ceres/rotation.h"
#include "Camera.h"

namespace camodocal
{

#define SCARAMUZZA_POLY_SIZE 5
#define SCARAMUZZA_INV_POLY_SIZE 6

#define SCARAMUZZA_CAMERA_NUM_PARAMS (SCARAMUZZA_POLY_SIZE + SCARAMUZZA_INV_POLY_SIZE + 2 /*center*/ + 3 /*affine*/)

/**
 * Scaramuzza Camera (Omnidirectional)
 * https://sites.google.com/site/scarabotix/ocamcalib-toolbox
 */

class OCAMCamera: public Camera
{
public:
    class Parameters: public Camera::Parameters
    {
    public:
        Parameters();

        double& C(void) { return m_C; }
        double& D(void) { return m_D; }
        double& E(void) { return m_E; }

        double& center_x(void) { return m_center_x; }
        double& center_y(void) { return m_center_y; }

        double& poly(int idx) { return m_poly[idx]; }
        double& inv_poly(int idx) { return m_inv_poly[idx]; }

        double C(void) const { return m_C; }
        double D(void) const { return m_D; }
        double E(void) const { return m_E; }

        double center_x(void) const { return m_center_x; }
        double center_y(void) const { return m_center_y; }

        double poly(int idx) const { return m_poly[idx]; }
        double inv_poly(int idx) const { return m_inv_poly[idx]; }

        bool readFromYamlFile(const std::string& filename);
        void writeToYamlFile(const std::string& filename) const;

        Parameters& operator=(const Parameters& other);
        friend std::ostream& operator<< (std::ostream& out, const Parameters& params);

    private:
        double m_poly[SCARAMUZZA_POLY_SIZE];
        double m_inv_poly[SCARAMUZZA_INV_POLY_SIZE];
        double m_C;
        double m_D;
        double m_E;
        double m_center_x;
        double m_center_y;
    };

    OCAMCamera();

    /**
    * \brief Constructor from the projection model parameters
    */
    OCAMCamera(const Parameters& params);

    Camera::ModelType modelType(void) const;
    const std::string& cameraName(void) const;
    int imageWidth(void) const;
    int imageHeight(void) const;

    void estimateIntrinsics(const cv::Size& boardSize,
                            const std::vector< std::vector<cv::Point3f> >& objectPoints,
                            const std::vector< std::vector<cv::Point2f> >& imagePoints);

    // Lift points from the image plane to the sphere
    void liftSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;
    //%output P

    // Lift points from the image plane to the projective space
    void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;
    //%output P

    // Projects 3D points to the image plane (Pi function)
    void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const;
    //%output p

    // Projects 3D points to the image plane (Pi function)
    // and calculates jacobian
    //void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p,
    //                  Eigen::Matrix<double,2,3>& J) const;
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
    Parameters mParameters;

    double m_inv_scale;
};

typedef boost::shared_ptr<OCAMCamera> OCAMCameraPtr;
typedef boost::shared_ptr<const OCAMCamera> OCAMCameraConstPtr;

template <typename T>
void
OCAMCamera::spaceToPlane(const T* const params,
                         const T* const q, const T* const t,
                         const Eigen::Matrix<T, 3, 1>& P,
                         Eigen::Matrix<T, 2, 1>& p)
{
    T P_c[3];
    {
        T P_w[3];
        P_w[0] = T(P(0));
        P_w[1] = T(P(1));
        P_w[2] = T(P(2));

        // Convert quaternion from Eigen convention (x, y, z, w)
        // to Ceres convention (w, x, y, z)
        T q_ceres[4] = {q[3], q[0], q[1], q[2]};

        ceres::QuaternionRotatePoint(q_ceres, P_w, P_c);

        P_c[0] += t[0];
        P_c[1] += t[1];
        P_c[2] += t[2];
    }

    T c = params[0];
    T d = params[1];
    T e = params[2];
    T xc[2] = { params[3], params[4] };

    //T poly[SCARAMUZZA_POLY_SIZE];
    //for (int i=0; i < SCARAMUZZA_POLY_SIZE; i++)
    //    poly[i] = params[5+i];

    T inv_poly[SCARAMUZZA_INV_POLY_SIZE];
    for (int i=0; i < SCARAMUZZA_INV_POLY_SIZE; i++)
        inv_poly[i] = params[5 + SCARAMUZZA_POLY_SIZE + i];

    T norm_sqr = P_c[0] * P_c[0] + P_c[1] * P_c[1];
    T norm = T(0.0);
    if (norm_sqr > T(0.0))
        norm = sqrt(norm_sqr);

    T theta = atan2(-P_c[2], norm);
    T rho = T(0.0);
    T theta_i = T(1.0);

    for (int i = 0; i < SCARAMUZZA_INV_POLY_SIZE; i++)
    {
        rho += theta_i * inv_poly[i];
        theta_i *= theta;
    }

    T invNorm = T(1.0) / norm;
    T xn[2] = {
        P_c[0] * invNorm * rho,
        P_c[1] * invNorm * rho
    };

    p(0) = xn[0] * c + xn[1] * d + xc[0];
    p(1) = xn[0] * e + xn[1]     + xc[1];
}

}

#endif
