#ifndef ROBOTICS_PROJECT_H
#define ROBOTICS_PROJECT_H

#include <tuple>

#include "Eigen/Dense"

#define TOTAL_TIME 10.0
#define TIME_STEP 0.1

/* Typedefs in order to be more clear */
typedef Eigen::Matrix<double, Eigen::Dynamic, 8> Path;  // same as MatrixX8d
typedef Eigen::Matrix<double, 1, 8>
    PathRow;  // same as RowVector8d
              // docs reference: https://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html

const Eigen::Matrix<double, 6, 1> a_DH(0, -0.425, -0.3922, 0, 0, 0);
const Eigen::Matrix<double, 6, 1> d_DH(0.1625, 0, 0, 0.1333, 0.0997, 0.0996);
const Eigen::Matrix<double, 6, 1> alpha_DH(M_PI / 2, 0, 0, M_PI / 2, -M_PI / 2, 0);

Eigen::Vector3d lerp(const Eigen::Vector3d& start, const Eigen::Vector3d& end, double t);
Eigen::Quaterniond slerp(const Eigen::Quaterniond& start, const Eigen::Quaterniond& end, double t);
Eigen::Matrix4d generalTransformationMatrix(double theta, double alpha, double d, double a);

Eigen::Matrix<double, 6, 6> getJacobian(const Eigen::Matrix<double, 6, 1>& joints);

std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix4d> directKinematics(
    const Eigen::Matrix<double, 6, 1>& joints);

Eigen::Matrix<double, 8, 1> getJointConfiguration();

Eigen::Vector3d worldToBaseCoordinates(const Eigen::Vector3d& point);
Eigen::Matrix4d worldToBaseTransformationMatrix();

void insertPath(Path& path, const Eigen::Matrix<double, 8, 1>& jointConfiguration);
void insertPath(Path& path, const Path& pathToInsert);

Path moveRobot(const Eigen::Vector3d& finalPosition, const Eigen::Matrix3d& finalRotationMatrix);

double calculateDeterminantJJT(const Eigen::Matrix<double, 6, 6>& jacobian);
double calculateDampingFactor(double w, double wt, double lambda0);
Eigen::Matrix<double, 6, 6> calculateDampedPseudoInverse(const Eigen::Matrix<double, 6, 6>& jacobian, double lambda);

Path differentialKinematicsQuaternion(const Eigen::Matrix<double, 8, 1>& jointConfiguration,
                                      const Eigen::Vector3d& initialPosition,
                                      const Eigen::Quaterniond& initialQuaternion, const Eigen::Vector3d& finalPosition,
                                      const Eigen::Quaterniond& finalQuaternion);

Eigen::Matrix<double, 6, 1> computeQdot0(const Eigen::Matrix<double, 6, 1>& jointsState);

Eigen::Matrix<double, 1, 8> optimizeParamDiffKinQuat(const Eigen::Matrix<double, 8, 1>& jointConfiguration,
                                                     const Eigen::Vector3d& initialPosition,
                                                     const Eigen::Quaterniond& initialQuaternion,
                                                     const Eigen::Vector3d& finalPosition,
                                                     const Eigen::Quaterniond& finalQuaternion, const double& wt,
                                                     const double& lambda0);

void runOptimization(const Eigen::Vector3d& finalPosition, const Eigen::Matrix3d& finalRotationMatrix);

#endif