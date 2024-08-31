/*!
    @file utils.h
    @brief Functions declaration for the movement and the high planning level of the robot
    @author Nicolo' Marconi
*/
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
//! @brief Total time to reach the final position
#define TOTAL_TIME 10.0
//! @brief Time step of each iteration of the differential kinematics
#define TIME_STEP 0.1
//! @brief Number of segments on the circle
#define N_SEGMENTS 40
//! @brief Standard height of the robot from the base frame
#define STD_HEIGHT 0.4
//! @brief Radius of the circle to follow
#define RADIUS_CIRCLE 0.3
//! @brief Size of the end effector tool
#define TOOL_SIZE 0.13
//! @brief Close gripper angle for thin blocks (X1-Y...)
#define CLOSE_GRIPPER_ANGLE_THIN -0.45
//! @brief Close gripper angle for thick blocks (X2-Y...)
#define CLOSE_GRIPPER_ANGLE_THICK 0.275
//! @brief Open gripper angle
#define OPEN_GRIPPER_ANGLE M_PI_2

//! @brief Path matrix containing joints configurations
typedef Eigen::Matrix<double, Eigen::Dynamic, 8> Path;
//! @brief Trajectory matrix containing points to follow
typedef Eigen::Matrix<double, Eigen::Dynamic, 3> Trajectory;
//! @brief Row of the path matrix
typedef Eigen::Matrix<double, 1, 8> PathRow;
//! @brief DH parameters a
const Eigen::Matrix<double, 6, 1> a_DH(0, -0.425, -0.3922, 0, 0, 0);
//! @brief DH parameters d
const Eigen::Matrix<double, 6, 1> d_DH(0.1625, 0, 0, 0.1333, 0.0997, 0.0996);
//! @brief DH parameters alpha
const Eigen::Matrix<double, 6, 1> alpha_DH(M_PI / 2, 0, 0, M_PI / 2, -M_PI / 2, 0);

//! @brief Int representing the movement direction
typedef int MovementDirection;
/*!
    @brief Enum representing the movement direction
    NONE if it is impossible to set a movement direction, CLOCKWISE if the robot needs to move clockwise,
   COUNTERCLOCKWISE if the robot needs to move counterclockwise
*/
enum MovementDirection_ { NONE = 0, CLOCKWISE = 1, COUNTERCLOCKWISE = 2 };
//! @brief Int representing the gripper state
typedef int GripperState;
//! @brief Enum representing the gripper state: CLOSE if the gripper is closed, OPEN if the gripper is open
enum GripperState_ { CLOSE = 0, OPEN = 1 };
//! @brief Int representing the block id
typedef int BlockId;
//! @brief Enum representing the block id, LENGTH is the default value used if the block id is not recognized and it
//! represents the length of the enum
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
/*!
    @brief Function to compute linear interpolation
    @param[in] start: initial value
    @param[in] end: final value
    @param[in] t: current time
    @param[in] maxTime: maximum time
    @return Eigen::Vector3d: interpolated value
*/
Eigen::Vector3d lerp(const Eigen::Vector3d& start, const Eigen::Vector3d& end, double t, const double maxTime);
/*!
    @brief Function to compute spherical linear interpolation
    @param[in] start: initial value
    @param[in] end: final value
    @param[in] t: current time
    @param[in] maxTime: maximum time
    @return Eigen::Vector3d: interpolated value
*/
Eigen::Quaterniond slerp(const Eigen::Quaterniond& start, const Eigen::Quaterniond& end, double t,
                         const double maxTime);
/*!
    @brief Function to compute general transformation matrix
    @param[in] theta: rotation angle around z axis
    @param[in] alpha: rotation angle around x axis
    @param[in] d: joint offset z axis
    @param[in] a: joint offset x axis
    @return Eigen::Matrix4d: transformation matrix
*/
Eigen::Matrix4d generalTransformationMatrix(double theta, double alpha, double d, double a);
/*!
    @brief Function to compute rotation matrix around z axis
    @param[in] theta: yaw angle
    @return Eigen::Matrix3d: rotation matrix around z axis
*/
Eigen::Matrix3d rotationMatrixAroundZ(double theta);
/*!
    @brief Function to compute geometric jacobian matrix given joints state
    @param[in] joints: joints state
    @return Eigen::Matrix<double, 6, 6>: geometric jacobian matrix
*/
Eigen::Matrix<double, 6, 6> getJacobian(const Eigen::Matrix<double, 6, 1>& joints);
/*!
    @brief Function to compute direct kinematics
    @param[in] joints: joints state
    @return std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix4d>: [position end effector, rotation matrix end
   effector, transformation matrix]
*/
std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix4d> directKinematics(
    const Eigen::Matrix<double, 6, 1>& joints);
/*!
    @brief Function to get joint configuration from robot
    @return std::tuple<Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 8, 1>>: [joint state (without gripper joints),
   joint configuration (with gripper joints)]
*/
std::tuple<Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 8, 1>> getJointConfiguration();
/*!
    @brief Function to get joint state from joint configuration
    @param[in] jointConfiguration: joint configuration
    @return Eigen::Matrix<double, 6, 1>: joint state (without gripper joints)
*/
Eigen::Matrix<double, 6, 1> getJointState(const Eigen::Matrix<double, 8, 1>& jointConfiguration);
/*!
    @brief Function to get transformation matrix from world to base frame
    @return Eigen::Matrix4d: transformation matrix
*/
constexpr const Eigen::Matrix4d worldToBaseTransformationMatrix();
/*!
    @brief Function to compute points expressed in base frame from points expressed in world frame
    @param[in] point: point expressed in world frame
    @return Eigen::Vector3d: point expressed in base frame
*/
Eigen::Vector3d worldToBaseCoordinates(const Eigen::Vector3d& point);
/*!
    @brief Function to get block id from string
    @param[in] blockId: id string of the block
    @return BlockId: enum representing the block id
*/
BlockId getBlockId(const std::string& blockId);
/*!
    @brief Function to compute final position based on the block id
    @param[in] blockId: id string of the block
    @return Eigen::Vector3d: enum representing the block id (LENGTH if unknown block id)
*/
Eigen::Vector3d getFinalPosition(const std::string& blockId);
/*!
    @brief Function to insert a point in a trajectory
    @param[in] trajectory: trajectory matrix containing points to follow
    @param[in] point: point to be insert in the trajectory
*/
void insertTrajectory(Trajectory& trajectory, const Eigen::Vector3d& point);
/*!
    @brief Function to insert a joint configuration in a path
    @param[in] path: path matrix containing joints configurations
    @param[in] jointConfiguration: joint configuration to be insert in the path
*/
void insertPath(Path& path, const Eigen::Matrix<double, 8, 1>& jointConfiguration);
/*!
    @brief Function to append a path in another path
    @param[in] path: path matrix containing joints configurations
    @param[in] pathToInsert: path matrix containing joints configurations to be insert in the path
*/
void insertPath(Path& path, const Path& pathToInsert);
/*!
    @brief Function to compute the distance between two points
    @param[in] point1: first point
    @param[in] point2: second point
    @return double: distance between the two points
*/
double distanceBetweenPoints(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2);
/*!
    @brief Function to get the movement direction of the robot on a circular trajectory
    @param[in] initialPosition: initial position on the circle
    @param[in] finalPosition: final position on the circle
    @param[in] jointConfiguration: current joint configuration
    @return MovementDirection: enum representing the movement direction (clockwise, counterclockwise or none)
*/
MovementDirection getMovementDirection(const Eigen::Vector3d& initialPosition, const Eigen::Vector3d& finalPosition,
                                       const Eigen::Matrix<double, 8, 1>& jointConfiguration);
/*!
    @brief Function to compute the position of N equally spaced points on a circle
    @return Eigen::Matrix<double, N_SEGMENTS, 3>: matrix containing the N points positions on the circle
*/
constexpr const Eigen::Matrix<double, N_SEGMENTS, 3> getNPointsOnCircle();
/*!
    @brief Function to compute the trajectory of the robot on a circular trajectory
    @param[in] initialPosition: initial position on the circle
    @param[in] finalPosition: final position on the circle
    @param[in] jointConfiguration: current joint configuration
    @return Trajectory: trajectory matrix containing the points on the circle to follow
*/
Trajectory computeCircularTrajectory(const Eigen::Vector3d& initialPosition, const Eigen::Vector3d& finalPosition,
                                     const Eigen::Matrix<double, 8, 1>& jointConfiguration);
/*!
    @brief Function to set the gripper state
    @param[in] jointConfiguration: current joint configuration
    @param[in] GripperState: state of the gripper (open or close)
    @param[in] blockId: block id of the object to be gripped
    @return Path: path matrix containing the joints configurations of the opening/closing gripper movement
*/
Path toggleGripper(const Eigen::Matrix<double, 8, 1>& jointConfiguration, const GripperState& state,
                   const std::string& blockId);
/*!
    @brief Function to compute the movement of the robot with differential kinematics
    @param[in] jointConfiguration: current joint configuration
    @param[in] finalPosition: final position to be reached
    @param[in] finalRotationMatrix: final rotation matrix to be reached
    @param[in] maxTime: maximum time to reach the final position (default: TOTAL_TIME = 10.0)
    @return Path: path matrix containing the joints configurations of the movement
*/
Path moveRobot(const Eigen::Matrix<double, 8, 1>& jointConfiguration, const Eigen::Vector3d& finalPosition,
               const Eigen::Matrix3d& finalRotationMatrix, const double maxTime = TOTAL_TIME);
/*!
    @brief Function to compute the movement of the robot with differential kinematics
    @param[in] jointConfiguration: current joint configuration
    @param[in] finalPosition: final position to be reached
    @param[in] finalQuaternion: final quaternion to be reached
    @param[in] maxTime: maximum time to reach the final position (default: TOTAL_TIME = 10.0)
    @return Path: path matrix containing the joints configurations of the movement
*/
Path moveRobot(const Eigen::Matrix<double, 8, 1>& jointConfiguration, const Eigen::Vector3d& finalPosition,
               const Eigen::Quaterniond& finalQuaternion, const double maxTime = TOTAL_TIME);
/*!
    @brief Function to compute the w parameter representing the squared root of the determinant of the Jacobian matrix
   multiplied by its transpose
    @param[in] jacobian: jacobian matrix
    @return double: w parameter to be used in the damping factor calculation
*/
double calculateDeterminantJJT(const Eigen::Matrix<double, 6, 6>& jacobian);
/*!
    @brief Function to compute the damping factor
    @param[in] w: squared root of the determinant of the Jacobian matrix multiplied by its transpose
    @param[in] wt: predefined threshold value
    @param[in] lambda0: predefined damping factor (maximum damping factor value)
    @return double: lambda damping factor
*/
double calculateDampingFactor(double w, double wt, double lambda0);
/*!
    @brief Function to compute the damped pseudo-inverse of the Jacobian matrix
    @param[in] jacobian: jacobian matrix
    @param[in] lambda: damping factor
    @return Eigen::Matrix<double, 6, 6>: damped pseudo-inverse of the Jacobian matrix
*/
Eigen::Matrix<double, 6, 6> calculateDampedPseudoInverse(const Eigen::Matrix<double, 6, 6>& jacobian, double lambda);
/*!
    @brief Function to compute the differential kinematics of the robot with quaternion representation
    @param[in] jointConfiguration: initial joint configuration
    @param[in] initialPosition: initial position of the robot
    @param[in] initialQuaternion: initial quaternion of the robot
    @param[in] finalPosition: final position to be reached
    @param[in] finalQuaternion: final quaternion to be reached
    @param[in] maxTime: maximum time to reach the final position
    @return Path: path matrix containing the joints configurations of the movement
*/
Path differentialKinematicsQuaternion(const Eigen::Matrix<double, 8, 1>& jointConfiguration,
                                      const Eigen::Vector3d& initialPosition,
                                      const Eigen::Quaterniond& initialQuaternion, const Eigen::Vector3d& finalPosition,
                                      const Eigen::Quaterniond& finalQuaternion, const double maxTime);
/*!
    @brief Function to compute the middle position of the actuators range to stay close to as much as possible
    @param[in] jointState: joints state of the robot
    @return Eigen::Matrix<double, 6, 1>: joints state to stay close to
*/
Eigen::Matrix<double, 6, 1> computeQdot0(const Eigen::Matrix<double, 6, 1>& jointsState);
/*!
    @brief Function to optimize the damping factor and the weight parameter to be used in the differential kinematics
    @param[in] jointConfiguration: initial joint configuration
    @param[in] initialPosition: initial position of the robot
    @param[in] initialQuaternion: initial quaternion of the robot
    @param[in] finalPosition: final position to be reached
    @param[in] finalQuaternion: final quaternion to be reached
    @param[in] maxTime: maximum time to reach the final position
    @param[in] wt: predefined threshold value
    @param[in] lambda0: predefined damping factor (maximum damping factor value)
    @param[out] singularities: number of singularities encountered during the optimization
    @return Eigen::Matrix<double, 1, 8>: final joint configuration
*/
Eigen::Matrix<double, 1, 8> optimizeParamDiffKinQuat(const Eigen::Matrix<double, 8, 1>& jointConfiguration,
                                                     const Eigen::Vector3d& initialPosition,
                                                     const Eigen::Quaterniond& initialQuaternion,
                                                     const Eigen::Vector3d& finalPosition,
                                                     const Eigen::Quaterniond& finalQuaternion, const double maxTime,
                                                     const double& wt, const double& lambda0, int& singularities);
/*!
    @brief Function to move the robot during the optimization process
    @param[in] jointConfiguration: current joint configuration
    @param[in] finalPosition: final position to be reached
    @param[in] finalQuaternion: final quaternion to be reached
    @param[in] lambda0: predefined damping factor (maximum damping factor value)
    @param[in] wt: predefined threshold value
    @param[out] singularities: number of singularities encountered during the optimization
    @param[in] maxTime: maximum time to reach the final position
    @return Eigen::Matrix<double, 1, 8>: joint configuration of the robot
*/
PathRow moveRobotOptimization(const Eigen::Matrix<double, 8, 1>& jointConfiguration,
                              const Eigen::Vector3d& finalPosition, const Eigen::Quaterniond& finalQuaternion,
                              const double& lambda0, const double& wt, int& singularities,
                              const double maxTime = TOTAL_TIME);

#endif