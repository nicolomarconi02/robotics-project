#ifndef ROBOTICS_PROJECT_H
#define ROBOTICS_PROJECT_H

#include <tuple>

#include "Eigen/Dense"

/* Typedefs in order to be more clear */
typedef Eigen::Matrix<double, Eigen::Dynamic, 8> Path;  // same as MatrixX8d
typedef Eigen::Matrix<double, 1, 8>
    PathRow;  // same as RowVector8d
              // docs reference: https://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html

const Eigen::Matrix<double, 6, 1> a_DH(0, -0.425, -0.3922, 0, 0, 0);
const Eigen::Matrix<double, 6, 1> d_DH(0.1625, 0, 0, 0.1333, 0.0997, 0.0996);
const Eigen::Matrix<double, 6, 1> alpha_DH(M_PI / 2, 0, 0, M_PI / 2, -M_PI / 2, 0);

Eigen::Matrix4d generalTransformationMatrix(double theta, double alpha, double d, double a);

Eigen::Matrix<double, 6, 6> getJacobian(const Eigen::Matrix<double, 6, 1>& joints);

std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix4d> directKinematics(
    const Eigen::Matrix<double, 6, 1>& joints);

Eigen::Matrix<double, 8, 1> getJointConfiguration();
#endif