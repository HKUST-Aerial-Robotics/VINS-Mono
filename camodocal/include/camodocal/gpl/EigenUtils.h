#ifndef EIGENUTILS_H
#define EIGENUTILS_H

#include <eigen3/Eigen/Dense>

#include "ceres/rotation.h"
#include "camodocal/gpl/gpl.h"

namespace camodocal
{

// Returns the 3D cross product skew symmetric matrix of a given 3D vector
template<typename T>
Eigen::Matrix<T, 3, 3> skew(const Eigen::Matrix<T, 3, 1>& vec)
{
    return (Eigen::Matrix<T, 3, 3>() << T(0), -vec(2), vec(1),
                                        vec(2), T(0), -vec(0),
                                        -vec(1), vec(0), T(0)).finished();
}

template<typename Derived>
typename Eigen::MatrixBase<Derived>::PlainObject sqrtm(const Eigen::MatrixBase<Derived>& A)
{
    Eigen::SelfAdjointEigenSolver<typename Derived::PlainObject> es(A);

    return es.operatorSqrt();
}

template<typename T>
Eigen::Matrix<T, 3, 3> AngleAxisToRotationMatrix(const Eigen::Matrix<T, 3, 1>& rvec)
{
    T angle = rvec.norm();
    if (angle == T(0))
    {
        return Eigen::Matrix<T, 3, 3>::Identity();
    }

    Eigen::Matrix<T, 3, 1> axis;
    axis = rvec.normalized();

    Eigen::Matrix<T, 3, 3> rmat;
    rmat = Eigen::AngleAxis<T>(angle, axis);

    return rmat;
}

template<typename T>
Eigen::Quaternion<T> AngleAxisToQuaternion(const Eigen::Matrix<T, 3, 1>& rvec)
{
    Eigen::Matrix<T, 3, 3> rmat = AngleAxisToRotationMatrix<T>(rvec);

    return Eigen::Quaternion<T>(rmat);
}

template<typename T>
void AngleAxisToQuaternion(const Eigen::Matrix<T, 3, 1>& rvec, T* q)
{
    Eigen::Quaternion<T> quat = AngleAxisToQuaternion<T>(rvec);

    q[0] = quat.x();
    q[1] = quat.y();
    q[2] = quat.z();
    q[3] = quat.w();
}

template<typename T>
Eigen::Matrix<T, 3, 1> RotationToAngleAxis(const Eigen::Matrix<T, 3, 3> & rmat)
{
    Eigen::AngleAxis<T> angleaxis; 
    angleaxis.fromRotationMatrix(rmat); 
    return angleaxis.angle() * angleaxis.axis(); 
    
}

template<typename T>
void QuaternionToAngleAxis(const T* const q, Eigen::Matrix<T, 3, 1>& rvec)
{
    Eigen::Quaternion<T> quat(q[3], q[0], q[1], q[2]);

    Eigen::Matrix<T, 3, 3> rmat = quat.toRotationMatrix();

    Eigen::AngleAxis<T> angleaxis;
    angleaxis.fromRotationMatrix(rmat);

    rvec = angleaxis.angle() * angleaxis.axis();
}

template<typename T>
Eigen::Matrix<T, 3, 3> QuaternionToRotation(const T* const q)
{
    T R[9];
    ceres::QuaternionToRotation(q, R);

    Eigen::Matrix<T, 3, 3> rmat;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            rmat(i,j) = R[i * 3 + j];
        }
    }

    return rmat;
}

template<typename T>
void QuaternionToRotation(const T* const q, T* rot)
{
    ceres::QuaternionToRotation(q, rot);
}

template<typename T>
Eigen::Matrix<T,4,4> QuaternionMultMatLeft(const Eigen::Quaternion<T>& q)
{
    return (Eigen::Matrix<T,4,4>() << q.w(), -q.z(), q.y(), q.x(),
                                      q.z(), q.w(), -q.x(), q.y(),
                                      -q.y(), q.x(), q.w(), q.z(),
                                      -q.x(), -q.y(), -q.z(), q.w()).finished();
}

template<typename T>
Eigen::Matrix<T,4,4> QuaternionMultMatRight(const Eigen::Quaternion<T>& q)
{
    return (Eigen::Matrix<T,4,4>() << q.w(), q.z(), -q.y(), q.x(),
                                      -q.z(), q.w(), q.x(), q.y(),
                                      q.y(), -q.x(), q.w(), q.z(),
                                      -q.x(), -q.y(), -q.z(), q.w()).finished();
}

/// @param theta - rotation about screw axis
/// @param d - projection of tvec on the rotation axis
/// @param l - screw axis direction
/// @param m - screw axis moment
template<typename T>
void AngleAxisAndTranslationToScrew(const Eigen::Matrix<T, 3, 1>& rvec,
                                    const Eigen::Matrix<T, 3, 1>& tvec,
                                    T& theta, T& d,
                                    Eigen::Matrix<T, 3, 1>& l,
                                    Eigen::Matrix<T, 3, 1>& m)
{

    theta = rvec.norm();
    if (theta == 0)
    {
        l.setZero(); 
        m.setZero(); 
        std::cout << "Warning: Undefined screw! Returned 0. " << std::endl; 
    }

    l = rvec.normalized();

    Eigen::Matrix<T, 3, 1> t = tvec;

    d = t.transpose() * l;

    // point on screw axis - projection of origin on screw axis
    Eigen::Matrix<T, 3, 1> c;
    c = 0.5 * (t - d * l + (1.0 / tan(theta / 2.0) * l).cross(t));

    // c and hence the screw axis is not defined if theta is either 0 or M_PI
    m = c.cross(l);
}

template<typename T>
Eigen::Matrix<T, 3, 3> RPY2mat(T roll, T pitch, T yaw)
{
    Eigen::Matrix<T, 3, 3> m;

    T cr = cos(roll);
    T sr = sin(roll);
    T cp = cos(pitch);
    T sp = sin(pitch);
    T cy = cos(yaw);
    T sy = sin(yaw);

    m(0,0) = cy * cp;
    m(0,1) = cy * sp * sr - sy * cr;
    m(0,2) = cy * sp * cr + sy * sr;
    m(1,0) = sy * cp;
    m(1,1) = sy * sp * sr + cy * cr;
    m(1,2) = sy * sp * cr - cy * sr;
    m(2,0) = - sp;
    m(2,1) = cp * sr;
    m(2,2) = cp * cr;
    return m; 
}

template<typename T>
void mat2RPY(const Eigen::Matrix<T, 3, 3>& m, T& roll, T& pitch, T& yaw)
{
    roll = atan2(m(2,1), m(2,2));
    pitch = atan2(-m(2,0), sqrt(m(2,1) * m(2,1) + m(2,2) * m(2,2)));
    yaw = atan2(m(1,0), m(0,0));
}

template<typename T>
Eigen::Matrix<T, 4, 4> homogeneousTransform(const Eigen::Matrix<T, 3, 3>& R, const Eigen::Matrix<T, 3, 1>& t)
{
    Eigen::Matrix<T, 4, 4> H;
    H.setIdentity();

    H.block(0,0,3,3) = R;
    H.block(0,3,3,1) = t;

    return H;
}

template<typename T>
Eigen::Matrix<T, 4, 4> poseWithCartesianTranslation(const T* const q, const T* const p)
{
    Eigen::Matrix<T, 4, 4> pose = Eigen::Matrix<T, 4, 4>::Identity();

    T rotation[9];
    ceres::QuaternionToRotation(q, rotation);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            pose(i,j) = rotation[i * 3 + j];
        }
    }

    pose(0,3) = p[0];
    pose(1,3) = p[1];
    pose(2,3) = p[2];

    return pose;
}

template<typename T>
Eigen::Matrix<T, 4, 4> poseWithSphericalTranslation(const T* const q, const T* const p, const T scale = T(1.0))
{
    Eigen::Matrix<T, 4, 4> pose = Eigen::Matrix<T, 4, 4>::Identity();

    T rotation[9];
    ceres::QuaternionToRotation(q, rotation);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            pose(i,j) = rotation[i * 3 + j];
        }
    }

    T theta = p[0];
    T phi = p[1];
    pose(0,3) = sin(theta) * cos(phi) * scale;
    pose(1,3) = sin(theta) * sin(phi) * scale;
    pose(2,3) = cos(theta) * scale;

    return pose;
}

// Returns the Sampson error of a given essential matrix and 2 image points
template<typename T>
T sampsonError(const Eigen::Matrix<T, 3, 3>& E,
               const Eigen::Matrix<T, 3, 1>& p1,
               const Eigen::Matrix<T, 3, 1>& p2)
{
    Eigen::Matrix<T, 3, 1> Ex1 = E * p1;
    Eigen::Matrix<T, 3, 1> Etx2 = E.transpose() * p2;

    T x2tEx1 = p2.dot(Ex1);

    // compute Sampson error
    T err = square(x2tEx1) / (square(Ex1(0,0)) + square(Ex1(1,0)) + square(Etx2(0,0)) + square(Etx2(1,0)));

    return err;
}

// Returns the Sampson error of a given rotation/translation and 2 image points
template<typename T>
T sampsonError(const Eigen::Matrix<T, 3, 3>& R,
               const Eigen::Matrix<T, 3, 1>& t,
               const Eigen::Matrix<T, 3, 1>& p1,
               const Eigen::Matrix<T, 3, 1>& p2)
{
    // construct essential matrix
    Eigen::Matrix<T, 3, 3> E = skew(t) * R;

    Eigen::Matrix<T, 3, 1> Ex1 = E * p1;
    Eigen::Matrix<T, 3, 1> Etx2 = E.transpose() * p2;

    T x2tEx1 = p2.dot(Ex1);

    // compute Sampson error
    T err = square(x2tEx1) / (square(Ex1(0,0)) + square(Ex1(1,0)) + square(Etx2(0,0)) + square(Etx2(1,0)));

    return err;
}

// Returns the Sampson error of a given rotation/translation and 2 image points
template<typename T>
T sampsonError(const Eigen::Matrix<T, 4, 4>& H,
               const Eigen::Matrix<T, 3, 1>& p1,
               const Eigen::Matrix<T, 3, 1>& p2)
{
    Eigen::Matrix<T, 3, 3> R = H.block(0, 0, 3, 3);
    Eigen::Matrix<T, 3, 1> t = H.block(0, 3, 3, 1);

    return sampsonError(R, t, p1, p2);
}

template<typename T>
Eigen::Matrix<T, 3, 1>
transformPoint(const Eigen::Matrix<T, 4, 4>& H, const Eigen::Matrix<T, 3, 1>& P)
{
    Eigen::Matrix<T, 3, 1> P_trans = H.block(0, 0, 3, 3) * P + H.block(0, 3, 3, 1);

    return P_trans;
}

template<typename T>
Eigen::Matrix<T, 4, 4>
estimate3DRigidTransform(const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1> > >& points1,
                         const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1> > >& points2)
{
    // compute centroids
    Eigen::Matrix<T, 3, 1> c1, c2;
    c1.setZero(); c2.setZero();

    for (size_t i = 0; i < points1.size(); ++i)
    {
        c1 += points1.at(i);
        c2 += points2.at(i);
    }

    c1 /= points1.size();
    c2 /= points1.size();

    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> X(3, points1.size());
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Y(3, points1.size());
    for (size_t i = 0; i < points1.size(); ++i)
    {
        X.col(i) = points1.at(i) - c1;
        Y.col(i) = points2.at(i) - c2;
    }

    Eigen::Matrix<T, 3, 3> H = X * Y.transpose();

    Eigen::JacobiSVD< Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix<T, 3, 3> U = svd.matrixU();
    Eigen::Matrix<T, 3, 3> V = svd.matrixV();
    if (U.determinant() * V.determinant() < 0.0)
    {
        V.col(2) *= -1.0;
    }

    Eigen::Matrix<T, 3, 3> R = V * U.transpose();
    Eigen::Matrix<T, 3, 1> t = c2 - R * c1;

    return homogeneousTransform(R, t);
}

template<typename T>
Eigen::Matrix<T, 4, 4>
estimate3DRigidSimilarityTransform(const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1> > >& points1,
                                   const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1> > >& points2)
{
    // compute centroids
    Eigen::Matrix<T, 3, 1> c1, c2;
    c1.setZero(); c2.setZero();

    for (size_t i = 0; i < points1.size(); ++i)
    {
        c1 += points1.at(i);
        c2 += points2.at(i);
    }

    c1 /= points1.size();
    c2 /= points1.size();

    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> X(3, points1.size());
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Y(3, points1.size());
    for (size_t i = 0; i < points1.size(); ++i)
    {
        X.col(i) = points1.at(i) - c1;
        Y.col(i) = points2.at(i) - c2;
    }

    Eigen::Matrix<T, 3, 3> H = X * Y.transpose();

    Eigen::JacobiSVD< Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix<T, 3, 3> U = svd.matrixU();
    Eigen::Matrix<T, 3, 3> V = svd.matrixV();
    if (U.determinant() * V.determinant() < 0.0)
    {
        V.col(2) *= -1.0;
    }

    Eigen::Matrix<T, 3, 3> R = V * U.transpose();

    std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1> > > rotatedPoints1(points1.size());
    for (size_t i = 0; i < points1.size(); ++i)
    {
        rotatedPoints1.at(i) = R * (points1.at(i) - c1);
    }

    double sum_ss = 0.0, sum_tt = 0.0;
    for (size_t i = 0; i < points1.size(); ++i)
    {
        sum_ss += (points1.at(i) - c1).squaredNorm();
        sum_tt += (points2.at(i) - c2).dot(rotatedPoints1.at(i));
    }

    double scale = sum_tt / sum_ss;

    Eigen::Matrix<T, 3, 3> sR = scale * R;
    Eigen::Matrix<T, 3, 1> t = c2 - sR * c1;

    return homogeneousTransform(sR, t);
}

}

#endif
