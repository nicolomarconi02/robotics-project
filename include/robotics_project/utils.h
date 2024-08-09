#ifndef ROBOTICS_PROJECT_H
#define ROBOTICS_PROJECT_H

#include <tuple>

#include "Eigen/Dense"

#define X1_Y1_Z2_ "X1-Y1-Z2"
#define X1_Y2_Z1_ "X1-Y2-Z1"
#define X1_Y2_Z2_ "X1-Y2-Z2"
#define X1_Y2_Z2_CHAMFER_ "X1-Y2-Z2-CHAMFER"
#define X1_Y2_Z2_TWINFILLET_ "X1-Y2-Z2-TWINFILLET"
#define X1_Y3_Z2_ "X1-Y3-Z2"
#define X1_Y3_Z2_FILLET_ "X1-Y3-Z2-FILLET"
#define X1_Y4_Z1_ "X1-Y4-Z1"
#define X1_Y4_Z2_ "X1-Y4-Z2"
#define X2_Y2_Z2_ "X2-Y2-Z2"
#define X2_Y2_Z2_FILLET_ "X2-Y2-Z2-FILLET"

#define TOTAL_TIME 10.0
#define TIME_STEP 0.1
#define N_SEGMENTS 40
#define STD_HEIGHT 0.5
#define RADIUS_CIRCLE 0.35
#define TOOL_SIZE 0.12

/* Typedefs in order to be more clear */
typedef Eigen::Matrix<double, Eigen::Dynamic, 8> Path;
typedef Eigen::Matrix<double, Eigen::Dynamic, 3> Trajectory;
typedef Eigen::Matrix<double, 1, 8>
    PathRow;  // same as RowVector8d
              // docs reference: https://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html

const Eigen::Matrix<double, 6, 1> a_DH(0, -0.425, -0.3922, 0, 0, 0);
const Eigen::Matrix<double, 6, 1> d_DH(0.1625, 0, 0, 0.1333, 0.0997, 0.0996);
const Eigen::Matrix<double, 6, 1> alpha_DH(M_PI / 2, 0, 0, M_PI / 2, -M_PI / 2, 0);

typedef int MovementDirection;
enum MovementDirection_ { NONE = 0, CLOCKWISE = 1, COUNTERCLOCKWISE = 2 };

typedef int GripperState;
enum GripperState_ { CLOSE = 0, OPEN = 1 };

typedef int BlockId;
enum BlockId_ {
   X1_Y1_Z2,
   X1_Y2_Z1,
   X1_Y2_Z2,
   X1_Y2_Z2_CHAMFER,
   X1_Y2_Z2_TWINFILLET,
   X1_Y3_Z2,
   X1_Y3_Z2_FILLET,
   X1_Y4_Z1,
   X1_Y4_Z2,
   X2_Y2_Z2,
   X2_Y2_Z2_FILLET,
   LENGTH
};

Eigen::Vector3d lerp(const Eigen::Vector3d& start, const Eigen::Vector3d& end, double t);
Eigen::Quaterniond slerp(const Eigen::Quaterniond& start, const Eigen::Quaterniond& end, double t);
Eigen::Matrix4d generalTransformationMatrix(double theta, double alpha, double d, double a);
Eigen::Matrix3d rotationMatrixAroundZ(double theta);

Eigen::Matrix<double, 6, 6> getJacobian(const Eigen::Matrix<double, 6, 1>& joints);

std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix4d> directKinematics(
    const Eigen::Matrix<double, 6, 1>& joints);

std::tuple<Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 8, 1>> getJointConfiguration();
Eigen::Matrix<double, 6, 1> getJointState(const Eigen::Matrix<double, 8, 1>& jointConfiguration);

Eigen::Vector3d worldToBaseCoordinates(const Eigen::Vector3d& point);
constexpr const Eigen::Matrix4d worldToBaseTransformationMatrix();
BlockId getBlockId(const std::string& blockId);
Eigen::Vector3d getFinalPosition(const std::string& blockId);

void insertTrajectory(Trajectory& trajectory, const Eigen::Vector3d& point);
void insertPath(Path& path, const Eigen::Matrix<double, 8, 1>& jointConfiguration);
void insertPath(Path& path, const Path& pathToInsert);

double distanceBetweenPoints(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2);
MovementDirection getMovementDirection(const Eigen::Vector3d& initialPosition, const Eigen::Vector3d& finalPosition);
constexpr const Eigen::Matrix<double, N_SEGMENTS, 3> getNPointsOnCircle();
Trajectory computeCircularTrajectory(const Eigen::Vector3d& initialPosition, const Eigen::Vector3d& finalPosition);

Eigen::Matrix<double, 8, 1> toggleGripper(const Eigen::Matrix<double, 8, 1>& jointConfiguration,
                                          const GripperState& state);
Path moveRobot(const Eigen::Matrix<double, 8, 1>& jointConfiguration, const Eigen::Vector3d& finalPosition,
               const Eigen::Matrix3d& finalRotationMatrix, const double maxTime = TOTAL_TIME);
Path moveRobot(const Eigen::Matrix<double, 8, 1>& jointConfiguration, const Eigen::Vector3d& finalPosition,
               const Eigen::Quaterniond& finalQuaternion, const double maxTime = TOTAL_TIME);

double calculateDeterminantJJT(const Eigen::Matrix<double, 6, 6>& jacobian);
double calculateDampingFactor(double w, double wt, double lambda0);
Eigen::Matrix<double, 6, 6> calculateDampedPseudoInverse(const Eigen::Matrix<double, 6, 6>& jacobian, double lambda);

Path differentialKinematicsQuaternion(const Eigen::Matrix<double, 8, 1>& jointConfiguration,
                                      const Eigen::Vector3d& initialPosition,
                                      const Eigen::Quaterniond& initialQuaternion, const Eigen::Vector3d& finalPosition,
                                      const Eigen::Quaterniond& finalQuaternion, const double maxTime);

Eigen::Matrix<double, 6, 1> computeQdot0(const Eigen::Matrix<double, 6, 1>& jointsState);

Eigen::Matrix<double, 1, 8> optimizeParamDiffKinQuat(const Eigen::Matrix<double, 8, 1>& jointConfiguration,
                                                     const Eigen::Vector3d& initialPosition,
                                                     const Eigen::Quaterniond& initialQuaternion,
                                                     const Eigen::Vector3d& finalPosition,
                                                     const Eigen::Quaterniond& finalQuaternion, const double maxTime,
                                                     const double& wt, const double& lambda0, int& singularities);

PathRow moveRobotOptimization(const Eigen::Matrix<double, 8, 1>& jointConfiguration,
                              const Eigen::Vector3d& finalPosition, const Eigen::Quaterniond& finalQuaternion,
                              const double& lambda0, const double& wt, int& singularities,
                              const double maxTime = TOTAL_TIME);

#endif