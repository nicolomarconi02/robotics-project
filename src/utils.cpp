/*!
    @file utils.cpp
    @brief Functions definition for the movement and the high planning level of the robot
    @author Nicolo' Marconi
*/
#include "robotics_project/utils.h"

#include <complex>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"

Eigen::Vector3d lerp(const Eigen::Vector3d& start, const Eigen::Vector3d& end, double t, const double maxTime) {
   double t_norm = t / maxTime;
   return start + t_norm * (end - start);
}
Eigen::Quaterniond slerp(const Eigen::Quaterniond& start, const Eigen::Quaterniond& end, double t,
                         const double maxTime) {
   double t_norm = t / maxTime;
   return start.slerp(t_norm, end);
}
Eigen::Matrix4d generalTransformationMatrix(double theta, double alpha, double d, double a) {
   return Eigen::Matrix4d{{cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)},
                          {sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)},
                          {0, sin(alpha), cos(alpha), d},
                          {0, 0, 0, 1}};
}
Eigen::Matrix3d rotationMatrixAroundZ(double theta) {
   return Eigen::Matrix3d{{cos(theta), -sin(theta), 0}, {sin(theta), cos(theta), 0}, {0, 0, 1}};
}
Eigen::Matrix<double, 6, 6> getJacobian(const Eigen::Matrix<double, 6, 1>& joints) {
   Eigen::Matrix<double, 6, 6> jacobian;
   jacobian.setZero();
   jacobian.col(0) << d_DH(4) * (cos(joints(0)) * cos(joints(4)) +
                                 cos(joints(1) + joints(2) + joints(3)) * sin(joints(0)) * sin(joints(4))) +
                          d_DH(3) * cos(joints(0)) - a_DH(1) * cos(joints(1)) * sin(joints(0)) -
                          d_DH(4) * sin(joints(1) + joints(2) + joints(3)) * sin(joints(0)) -
                          a_DH(2) * cos(joints(1)) * cos(joints(2)) * sin(joints(0)) +
                          a_DH(2) * sin(joints(0)) * sin(joints(1)) * sin(joints(2)),
       d_DH(4) * (cos(joints(4)) * sin(joints(0)) -
                  cos(joints(1) + joints(2) + joints(3)) * cos(joints(0)) * sin(joints(4))) +
           d_DH(3) * sin(joints(0)) + a_DH(1) * cos(joints(0)) * cos(joints(1)) +
           d_DH(4) * sin(joints(1) + joints(2) + joints(3)) * cos(joints(0)) +
           a_DH(2) * cos(joints(0)) * cos(joints(1)) * cos(joints(2)) -
           a_DH(2) * cos(joints(0)) * sin(joints(1)) * sin(joints(2)),
       0, 0, 0, 1;
   jacobian.col(1) << -cos(joints(0)) * (a_DH(2) * sin(joints(1) + joints(2)) + a_DH(1) * sin(joints(1)) +
                                         d_DH(4) * (sin(joints(1) + joints(2)) * sin(joints(3)) -
                                                    cos(joints(1) + joints(2)) * cos(joints(3))) -
                                         d_DH(4) * sin(joints(4)) *
                                             (cos(joints(1) + joints(2)) * sin(joints(3)) +
                                              sin(joints(1) + joints(2)) * cos(joints(3)))),
       -sin(joints(0)) *
           (a_DH(2) * sin(joints(1) + joints(2)) + a_DH(1) * sin(joints(1)) +
            d_DH(4) * (sin(joints(1) + joints(2)) * sin(joints(3)) - cos(joints(1) + joints(2)) * cos(joints(3))) -
            d_DH(4) * sin(joints(4)) *
                (cos(joints(1) + joints(2)) * sin(joints(3)) + sin(joints(1) + joints(2)) * cos(joints(3)))),
       a_DH(2) * cos(joints(1) + joints(2)) - (d_DH(4) * sin(joints(1) + joints(2) + joints(3) + joints(4))) / 2 +
           a_DH(1) * cos(joints(1)) + (d_DH(4) * sin(joints(1) + joints(2) + joints(3) - joints(4))) / 2 +
           d_DH(4) * sin(joints(1) + joints(2) + joints(3)),
       sin(joints(0)), -cos(joints(0)), 0;
   jacobian.col(2) << cos(joints(0)) *
                          (d_DH(4) * cos(joints(1) + joints(2) + joints(3)) - a_DH(2) * sin(joints(1) + joints(2)) +
                           d_DH(4) * sin(joints(1) + joints(2) + joints(3)) * sin(joints(4))),
       sin(joints(0)) * (d_DH(4) * cos(joints(1) + joints(2) + joints(3)) - a_DH(2) * sin(joints(1) + joints(2)) +
                         d_DH(4) * sin(joints(1) + joints(2) + joints(3)) * sin(joints(4))),
       a_DH(2) * cos(joints(1) + joints(2)) - (d_DH(4) * sin(joints(1) + joints(2) + joints(3) + joints(4))) / 2 +
           (d_DH(4) * sin(joints(1) + joints(2) + joints(3) - joints(4))) / 2 +
           d_DH(4) * sin(joints(1) + joints(2) + joints(3)),
       sin(joints(0)), -cos(joints(0)), 0;
   jacobian.col(3) << d_DH(4) * cos(joints(0)) *
                          (cos(joints(1) + joints(2) + joints(3)) +
                           sin(joints(1) + joints(2) + joints(3)) * sin(joints(4))),
       d_DH(4) * sin(joints(0)) *
           (cos(joints(1) + joints(2) + joints(3)) + sin(joints(1) + joints(2) + joints(3)) * sin(joints(4))),
       d_DH(4) * (sin(joints(1) + joints(2) + joints(3) - joints(4)) / 2 + sin(joints(1) + joints(2) + joints(3)) -
                  sin(joints(1) + joints(2) + joints(3) + joints(4)) / 2),
       sin(joints(0)), -cos(joints(0)), 0;
   jacobian.col(4) << d_DH(4) * cos(joints(0)) * cos(joints(1)) * cos(joints(4)) * sin(joints(2)) * sin(joints(3)) -
                          d_DH(4) * cos(joints(0)) * cos(joints(1)) * cos(joints(2)) * cos(joints(3)) * cos(joints(4)) -
                          d_DH(4) * sin(joints(0)) * sin(joints(4)) +
                          d_DH(4) * cos(joints(0)) * cos(joints(2)) * cos(joints(4)) * sin(joints(1)) * sin(joints(3)) +
                          d_DH(4) * cos(joints(0)) * cos(joints(3)) * cos(joints(4)) * sin(joints(1)) * sin(joints(2)),
       d_DH(4) * cos(joints(0)) * sin(joints(4)) +
           d_DH(4) * cos(joints(1)) * cos(joints(4)) * sin(joints(0)) * sin(joints(2)) * sin(joints(3)) +
           d_DH(4) * cos(joints(2)) * cos(joints(4)) * sin(joints(0)) * sin(joints(1)) * sin(joints(3)) +
           d_DH(4) * cos(joints(3)) * cos(joints(4)) * sin(joints(0)) * sin(joints(1)) * sin(joints(2)) -
           d_DH(4) * cos(joints(1)) * cos(joints(2)) * cos(joints(3)) * cos(joints(4)) * sin(joints(0)),
       -d_DH(4) * (sin(joints(1) + joints(2) + joints(3) - joints(4)) / 2 +
                   sin(joints(1) + joints(2) + joints(3) + joints(4)) / 2),
       sin(joints(1) + joints(2) + joints(3)) * cos(joints(0)), sin(joints(1) + joints(2) + joints(3)) * sin(joints(0)),
       -cos(joints(1) + joints(2) + joints(3));
   jacobian.col(5) << 0, 0, 0,
       cos(joints(4)) * sin(joints(0)) - cos(joints(1) + joints(2) + joints(3)) * cos(joints(0)) * sin(joints(4)),
       -cos(joints(0)) * cos(joints(4)) - cos(joints(1) + joints(2) + joints(3)) * sin(joints(0)) * sin(joints(4)),
       -sin(joints(1) + joints(2) + joints(3)) * sin(joints(4));
   return jacobian;
}

std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix4d> directKinematics(
    const Eigen::Matrix<double, 6, 1>& joints) {
   Eigen::Matrix4d transformation_matrix = generalTransformationMatrix(joints(0), alpha_DH(0), d_DH(0), a_DH(0));
   for (int i = 1; i < 6; i++) {
      transformation_matrix *= generalTransformationMatrix(joints(i), alpha_DH(i), d_DH(i), a_DH(i));
   }

   // pe: position end effector -> position wrist3_links
   Eigen::Vector3d pe(transformation_matrix.block(0, 3, 3, 1));
   Eigen::Matrix3d Re(transformation_matrix.block(0, 0, 3, 3));
   return std::make_tuple(pe, Re, transformation_matrix);
}

std::tuple<Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 8, 1>> getJointConfiguration() {
   // index from 0 to 7
   // Shoulder pan joint           index:4
   // Shoulder lift joint          index:3
   // Elbow joint                  index:0
   // Wrist1 joint                 index:5
   // Wrist2 joint                 index:6
   // Wrist3 joint                 index:7
   // Hand1 joint                  index:1
   // Hand2 joint                  index:2
   boost::shared_ptr<sensor_msgs::JointState const> joint_configuration =
       ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states");
   return std::make_tuple<Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 8, 1>>(
       Eigen::Matrix<double, 6, 1>{joint_configuration->position[4], joint_configuration->position[3],
                                   joint_configuration->position[0], joint_configuration->position[5],
                                   joint_configuration->position[6], joint_configuration->position[7]},
       Eigen::Matrix<double, 8, 1>{joint_configuration->position[4], joint_configuration->position[3],
                                   joint_configuration->position[0], joint_configuration->position[5],
                                   joint_configuration->position[6], joint_configuration->position[7],
                                   joint_configuration->position[1], joint_configuration->position[2]});
}

Eigen::Matrix<double, 6, 1> getJointState(const Eigen::Matrix<double, 8, 1>& jointConfiguration) {
   return Eigen::Matrix<double, 6, 1>{jointConfiguration(0), jointConfiguration(1), jointConfiguration(2),
                                      jointConfiguration(3), jointConfiguration(4), jointConfiguration(5)};
}
constexpr const Eigen::Matrix4d worldToBaseTransformationMatrix() {
   Eigen::Matrix4d transformationMatrix{
       {1.0, 0.0, 0.0, 0.5}, {0.0, -1.0, 0.0, 0.35}, {0.0, 0.0, -1.0, 1.75}, {0.0, 0.0, 0.0, 1.0}};
   return transformationMatrix;
}

Eigen::Vector3d worldToBaseCoordinates(const Eigen::Vector3d& point) {
   Eigen::Matrix4d transformationMatrix = worldToBaseTransformationMatrix();

   Eigen::Vector4d tmpPoint = transformationMatrix.inverse() * Eigen::Vector4d{point(0), point(1), point(2), 1.0};
   return Eigen::Vector3d{tmpPoint(0), tmpPoint(1), tmpPoint(2)};
}

BlockId getBlockId(const std::string& blockId) {
   if (blockId == X1_Y1_Z2_) {
      return BlockId_::X1_Y1_Z2;
   } else if (blockId == X1_Y2_Z1_) {
      return BlockId_::X1_Y2_Z1;
   } else if (blockId == X1_Y2_Z2_) {
      return BlockId_::X1_Y2_Z2;
   } else if (blockId == X1_Y2_Z2_CHAMFER_) {
      return BlockId_::X1_Y2_Z2_CHAMFER;
   } else if (blockId == X1_Y2_Z2_TWINFILLET_) {
      return BlockId_::X1_Y2_Z2_TWINFILLET;
   } else if (blockId == X1_Y3_Z2_) {
      return BlockId_::X1_Y3_Z2;
   } else if (blockId == X1_Y3_Z2_FILLET_) {
      return BlockId_::X1_Y3_Z2_FILLET;
   } else if (blockId == X1_Y4_Z1_) {
      return BlockId_::X1_Y4_Z1;
   } else if (blockId == X1_Y4_Z2_) {
      return BlockId_::X1_Y4_Z2;
   } else if (blockId == X2_Y2_Z2_) {
      return BlockId_::X2_Y2_Z2;
   } else if (blockId == X2_Y2_Z2_FILLET_) {
      return BlockId_::X2_Y2_Z2_FILLET;
   }
   // unknown block id
   return BlockId_::LENGTH;
}
Eigen::Vector3d getFinalPosition(const std::string& blockId) {
   BlockId block = getBlockId(blockId);
   // check if the block id is valid
   if (block == BlockId_::LENGTH) {
      return Eigen::Vector3d::Zero();
   }
   // size of the elevated part of the table
   Eigen::Vector2d planeSize{1.0, 0.15};
   // size of the drop region, one fixed for each block
   Eigen::Vector2d dropRegionSize{planeSize(0) / (double)BlockId_::LENGTH, planeSize(1)};
   double xPos = dropRegionSize(0) * (double)block + dropRegionSize(0) / 2.0;
   double yPos = dropRegionSize(1) / 2.0;
   double zPos = 1.2;
   Eigen::Vector3d finalPosition{xPos, yPos, zPos};
   return worldToBaseCoordinates(finalPosition);
}

void insertTrajectory(Trajectory& trajectory, const Eigen::Vector3d& point) {
   trajectory.conservativeResize(trajectory.rows() + 1, trajectory.cols());
   trajectory.row(trajectory.rows() - 1) = point;
}

void insertPath(Path& path, const Eigen::Matrix<double, 8, 1>& jointConfiguration) {
   path.conservativeResize(path.rows() + 1, path.cols());
   path.row(path.rows() - 1) = jointConfiguration;
}

void insertPath(Path& path, const Path& pathToInsert) {
   path.conservativeResize(path.rows() + pathToInsert.rows(), path.cols());
   path.bottomRows(pathToInsert.rows()) = pathToInsert;
}

double distanceBetweenPoints(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2) {
   return (point1 - point2).norm();
}

MovementDirection getMovementDirection(const Eigen::Vector3d& initialPosition, const Eigen::Vector3d& finalPosition,
                                       const Eigen::Matrix<double, 8, 1>& jointConfiguration) {
   double thetaInitial = std::atan2(initialPosition(1), initialPosition(0));
   double thetaFinal = std::atan2(finalPosition(1), finalPosition(0));

   double delta = thetaFinal - thetaInitial;

   delta = std::atan2(std::sin(delta), std::cos(delta));
   double finalAngle = jointConfiguration(0) + delta;
   // check if the final angle is in the range [-pi, pi] in order to avoid actuators limits problems
   if (finalAngle > M_PI) {
      return MovementDirection_::COUNTERCLOCKWISE;
   } else if (finalAngle < -M_PI) {
      return MovementDirection_::CLOCKWISE;
   }
   // set the movement direction based on the delta angle
   if (delta >= 0) {
      if (delta <= M_PI) {
         return MovementDirection_::CLOCKWISE;
      } else {
         return MovementDirection_::COUNTERCLOCKWISE;
      }
   }
   if (delta < 0) {
      if (delta >= -M_PI) {
         return MovementDirection_::COUNTERCLOCKWISE;
      } else {
         return MovementDirection_::CLOCKWISE;
      }
   }
   return MovementDirection_::NONE;
}

constexpr const Eigen::Matrix<double, N_SEGMENTS, 3> getNPointsOnCircle() {
   Eigen::Matrix<double, N_SEGMENTS, 3> points;
   for (double k = 0.0; k < N_SEGMENTS; k += 1.0) {
      // get N equally spaced points on a circle using complex numbers
      std::complex<double> zk = RADIUS_CIRCLE * std::exp(std::complex<double>(0.0, 2.0 * M_PI * (k / N_SEGMENTS)));
      Eigen::Vector3d point(zk.real(), zk.imag(), STD_HEIGHT);
      points.row(k) = point;
   }
   return points;
}

Trajectory computeCircularTrajectory(const Eigen::Vector3d& initialPosition, const Eigen::Vector3d& finalPosition,
                                     const Eigen::Matrix<double, 8, 1>& jointConfiguration) {
   // get the two intersections points between the circle and the line passing through the origin and the initial
   // position
   Eigen::Vector3d p1(
       (initialPosition(0) * RADIUS_CIRCLE) / (sqrt(pow(initialPosition(0), 2) + pow(initialPosition(1), 2))),
       (initialPosition(1) * RADIUS_CIRCLE) / (sqrt(pow(initialPosition(0), 2) + pow(initialPosition(1), 2))),
       STD_HEIGHT);
   Eigen::Vector3d p2(
       (-initialPosition(0) * RADIUS_CIRCLE) / (sqrt(pow(initialPosition(0), 2) + pow(initialPosition(1), 2))),
       (-initialPosition(1) * RADIUS_CIRCLE) / (sqrt(pow(initialPosition(0), 2) + pow(initialPosition(1), 2))),
       STD_HEIGHT);
   // get the closest point to the initial position
   Eigen::Vector3d initialPosOnCircle =
       (distanceBetweenPoints(initialPosition, p1) < distanceBetweenPoints(initialPosition, p2)) ? p1 : p2;
   // get the two intersections points between the circle and the line passing through the origin and the final position
   Eigen::Vector3d p3((finalPosition(0) * RADIUS_CIRCLE) / (sqrt(pow(finalPosition(0), 2) + pow(finalPosition(1), 2))),
                      (finalPosition(1) * RADIUS_CIRCLE) / (sqrt(pow(finalPosition(0), 2) + pow(finalPosition(1), 2))),
                      STD_HEIGHT);
   Eigen::Vector3d p4((-finalPosition(0) * RADIUS_CIRCLE) / (sqrt(pow(finalPosition(0), 2) + pow(finalPosition(1), 2))),
                      (-finalPosition(1) * RADIUS_CIRCLE) / (sqrt(pow(finalPosition(0), 2) + pow(finalPosition(1), 2))),
                      STD_HEIGHT);
   // get the closest point to the final position
   Eigen::Vector3d finalPosOnCircle =
       (distanceBetweenPoints(finalPosition, p3) < distanceBetweenPoints(finalPosition, p4)) ? p3 : p4;
   // get the movement direction based on the initial and final positions
   MovementDirection direction = getMovementDirection(initialPosOnCircle, finalPosOnCircle, jointConfiguration);
   if (direction == MovementDirection_::NONE) {
      return Trajectory();
   }
   // std::cout << "Initial position on circle: " << initialPosOnCircle << std::endl;
   // std::cout << "Final position on circle: " << finalPosOnCircle << std::endl;
   // std::cout << "Direction: " << direction << std::endl;

   // get the N points on the circle
   auto pointsOnCircle = getNPointsOnCircle();

   double minDistanceInitial = 1000.0;
   double minDistanceFinal = 1000.0;
   int indexInitial = 0;
   int indexFinal = 0;
   // iterate over the N points on the circle to get the index of the closest points to the initial and final positions
   for (int i = 0; i < N_SEGMENTS; i++) {
      double distanceInitial = distanceBetweenPoints(initialPosOnCircle, pointsOnCircle.row(i));
      double distanceFinal = distanceBetweenPoints(finalPosOnCircle, pointsOnCircle.row(i));
      if (distanceInitial < minDistanceInitial) {
         minDistanceInitial = distanceInitial;
         indexInitial = i;
      }
      if (distanceFinal < minDistanceFinal) {
         minDistanceFinal = distanceFinal;
         indexFinal = i;
      }
   }
   int increment = 0;
   // check if the initial and final points are in the same direction of the movement
   if (direction == MovementDirection_::CLOCKWISE) {
      increment = 1;
      if (getMovementDirection(initialPosOnCircle, pointsOnCircle.row(indexInitial), jointConfiguration) ==
          MovementDirection_::COUNTERCLOCKWISE) {
         indexInitial = (indexInitial + increment) % N_SEGMENTS;
      }
      if (getMovementDirection(finalPosOnCircle, pointsOnCircle.row(indexFinal), jointConfiguration) ==
          MovementDirection_::COUNTERCLOCKWISE) {
         indexFinal = (indexFinal + increment) % N_SEGMENTS;
      }
   } else if (direction == MovementDirection_::COUNTERCLOCKWISE) {
      increment = -1;
      if (getMovementDirection(initialPosOnCircle, pointsOnCircle.row(indexInitial), jointConfiguration) ==
          MovementDirection_::CLOCKWISE) {
         indexInitial = (indexInitial + increment) % N_SEGMENTS;
      }
      if (getMovementDirection(finalPosOnCircle, pointsOnCircle.row(indexFinal), jointConfiguration) ==
          MovementDirection_::CLOCKWISE) {
         indexFinal = (indexFinal + increment) % N_SEGMENTS;
      }
   }
   Trajectory trajectory;
   insertTrajectory(trajectory, initialPosOnCircle);
   // iterate over the points on the circle to set the trajectory
   for (int i = indexInitial; i != indexFinal; i = (i + increment) % N_SEGMENTS) {
      if (i < 0) {
         i = N_SEGMENTS - 1;
      }
      insertTrajectory(trajectory, pointsOnCircle.row(i));
   }
   insertTrajectory(trajectory, finalPosOnCircle);

   return trajectory;
}

Path toggleGripper(const Eigen::Matrix<double, 8, 1>& jointConfiguration, const GripperState& state,
                   const std::string& blockId) {
   Eigen::Matrix<double, 8, 1> jointConfigurationTmp = jointConfiguration;
   Path path;
   double toggleGripper = 0.0;
   double gripperRight = 0.0;
   double gripperLeft = 0.0;
   // set the angle of the gripper based on the block id and the state
   switch (state) {
      case GripperState_::CLOSE:
         // ROS_INFO("CLOSING GRIPPER");
         if (blockId == X2_Y2_Z2_ || blockId == X2_Y2_Z2_FILLET_) {
            toggleGripper = CLOSE_GRIPPER_ANGLE_THICK;
         } else {
            toggleGripper = CLOSE_GRIPPER_ANGLE_THIN;
         }
         break;
      case GripperState_::OPEN:
         // ROS_INFO("OPENING GRIPPER");
         toggleGripper = OPEN_GRIPPER_ANGLE;
         break;
      default:
         ROS_WARN("UNKNOWN GRIPPER STATE");
         break;
   }

   static int steps = 20;
   // interpolate the gripper angle
   for (int i = 1; i <= steps; i++) {
      gripperRight = jointConfigurationTmp(6) + (((toggleGripper - jointConfigurationTmp(6)) * i) / steps);
      gripperLeft = jointConfigurationTmp(7) + (((toggleGripper - jointConfigurationTmp(7)) * i) / steps);
      jointConfigurationTmp(6) = gripperRight;
      jointConfigurationTmp(7) = gripperLeft;
      insertPath(path, jointConfigurationTmp);
   }

   return path;
}

Path moveRobot(const Eigen::Matrix<double, 8, 1>& jointConfiguration, const Eigen::Vector3d& finalPosition,
               const Eigen::Matrix3d& finalRotationMatrix, const double maxTime) {
   auto jointState = getJointState(jointConfiguration);
   auto [pe, Re, transformationMatrix] = directKinematics(jointState);

   Eigen::Quaterniond initialQuaternion(Re);
   Eigen::Quaterniond finalQuaternion(finalRotationMatrix);

   return differentialKinematicsQuaternion(jointConfiguration, pe, initialQuaternion, finalPosition, finalQuaternion,
                                           maxTime);
}

Path moveRobot(const Eigen::Matrix<double, 8, 1>& jointConfiguration, const Eigen::Vector3d& finalPosition,
               const Eigen::Quaterniond& finalQuaternion, const double maxTime) {
   auto jointState = getJointState(jointConfiguration);
   auto [pe, Re, transformationMatrix] = directKinematics(jointState);
   Eigen::Quaterniond initialQuaternion(Re);

   return differentialKinematicsQuaternion(jointConfiguration, pe, initialQuaternion, finalPosition, finalQuaternion,
                                           maxTime);
}

double calculateDeterminantJJT(const Eigen::Matrix<double, 6, 6>& jacobian) {
   Eigen::Matrix<double, 6, 6> JJT = jacobian * jacobian.transpose();
   return std::sqrt(abs(JJT.determinant()));
}

double calculateDampingFactor(double w, double wt, double lambda0) {
   // check if the determinant is less than the threshold
   if (w < wt) {
      return lambda0 * std::pow(1.0 - (w / wt), 2);
   } else {
      return 0.0;
   }
}
Eigen::Matrix<double, 6, 6> calculateDampedPseudoInverse(const Eigen::Matrix<double, 6, 6>& jacobian, double lambda) {
   return jacobian.transpose() *
          (jacobian * jacobian.transpose() + std::pow(lambda, 2) * Eigen::Matrix<double, 6, 6>::Identity()).inverse();
}

Path differentialKinematicsQuaternion(const Eigen::Matrix<double, 8, 1>& jointConfiguration,
                                      const Eigen::Vector3d& initialPosition,
                                      const Eigen::Quaterniond& initialQuaternion, const Eigen::Vector3d& finalPosition,
                                      const Eigen::Quaterniond& finalQuaternion, const double maxTime) {
   Eigen::Matrix<double, 6, 6> jacobian_instantK, pseudoInverseJacobian_instantK;

   Eigen::Vector2d gripperState{jointConfiguration(6), jointConfiguration(7)};

   Eigen::Matrix<double, 6, 1> jointState_instantK, jointStateDot_instantK;
   Path path;

   Eigen::Quaterniond quaternion_instantK;
   Eigen::Matrix<double, 6, 1> velocities_instantK;

   // gains for the position and orientation
   Eigen::Matrix3d Kp = Eigen::Matrix3d::Identity() * 10;
   Eigen::Matrix3d Kq = Eigen::Matrix3d::Identity() * 1;
   jointState_instantK = jointConfiguration.head(6);
   insertPath(path, jointConfiguration);

   static const double tolerance = 0.005;

   for (double instantK = 0.0; instantK < maxTime; instantK += TIME_STEP) {
      // compute direct kinematics for the current joint state
      auto [pe_instantK, Re_instantK, transformationMatrix_instantK] = directKinematics(jointState_instantK);
      quaternion_instantK = Eigen::Quaterniond{Re_instantK};
      // compute the positional and angular velocities using lerp and slerp
      Eigen::Vector3d positionalVelocity_instantK =
          (lerp(initialPosition, finalPosition, instantK, maxTime) -
           lerp(initialPosition, finalPosition, instantK - TIME_STEP, maxTime)) /
          TIME_STEP;
      Eigen::Quaterniond quaternionVelocity_instantK =
          slerp(initialQuaternion, finalQuaternion, instantK + TIME_STEP, maxTime) *
          slerp(initialQuaternion, finalQuaternion, instantK, maxTime).conjugate();
      Eigen::Vector3d angularVelocity_instantK = (quaternionVelocity_instantK.vec() * 2.0) / TIME_STEP;

      jacobian_instantK = getJacobian(jointState_instantK);
      // set the maximum damping factor
      static double lambda0 = 1.0e-8;
      // set the threshold for the determinant
      static double wt = 0.24;
      // compute the determinant of JJT
      double w = calculateDeterminantJJT(jacobian_instantK);
      // compute the damping factor
      double lambda = calculateDampingFactor(w, wt, lambda0);
      // compute the damped pseudo inverse of the jacobian
      pseudoInverseJacobian_instantK = calculateDampedPseudoInverse(jacobian_instantK, lambda);
      // check for the singularities
      if (abs(jacobian_instantK.determinant()) < 1.0e-5) {
         ROS_WARN("NEAR SINGULARITY");
      }
      // compute the position and orientation errors
      Eigen::Vector3d positionError_instantK = lerp(initialPosition, finalPosition, instantK, maxTime) - pe_instantK;
      Eigen::Quaterniond quaternionError_instantK =
          slerp(initialQuaternion, finalQuaternion, instantK, maxTime) * quaternion_instantK.conjugate();
      // compute the desired velocities
      velocities_instantK << positionalVelocity_instantK + (Kp * positionError_instantK),
          angularVelocity_instantK + (Kq * quaternionError_instantK.vec());
      // compute the qdot0
      auto qdot0 = computeQdot0(jointState_instantK);
      // compute the joint state dot
      jointStateDot_instantK =
          pseudoInverseJacobian_instantK * velocities_instantK +
          (Eigen::Matrix<double, 6, 6>::Identity() - pseudoInverseJacobian_instantK * jacobian_instantK) * qdot0;
      jointState_instantK += jointStateDot_instantK * TIME_STEP;
      Eigen::Matrix<double, 8, 1> path_instantK;
      path_instantK << jointState_instantK, gripperState;
      // insert the joint configuration in the path
      insertPath(path, path_instantK);
      // check if the final position is reached with a tolerance
      if ((finalPosition - pe_instantK).norm() < tolerance) {
         ROS_INFO("REACHED %f", instantK);
         break;
      }
   }

   return path;
}

Eigen::Matrix<double, 6, 1> computeQdot0(const Eigen::Matrix<double, 6, 1>& jointsState) {
   static double k0 = 200;
   Eigen::Matrix<double, 6, 1> qdot0;
   qdot0 = -k0 * jointsState;
   return qdot0;
}

Eigen::Matrix<double, 1, 8> optimizeParamDiffKinQuat(const Eigen::Matrix<double, 8, 1>& jointConfiguration,
                                                     const Eigen::Vector3d& initialPosition,
                                                     const Eigen::Quaterniond& initialQuaternion,
                                                     const Eigen::Vector3d& finalPosition,
                                                     const Eigen::Quaterniond& finalQuaternion, const double maxTime,
                                                     const double& wt, const double& lambda0, int& singularities) {
   Eigen::Matrix<double, 6, 6> jacobian_instantK, pseudoInverseJacobian_instantK;

   Eigen::Vector2d gripperState{jointConfiguration(6), jointConfiguration(7)};

   Eigen::Matrix<double, 6, 1> jointState_instantK, jointStateDot_instantK;
   Eigen::Matrix<double, 1, 8> path;

   Eigen::Quaterniond quaternion_instantK;
   Eigen::Matrix<double, 6, 1> velocities_instantK;

   Eigen::Matrix3d Kp = Eigen::Matrix3d::Identity() * 10;
   Eigen::Matrix3d Kq = Eigen::Matrix3d::Identity() * 1;
   jointState_instantK = jointConfiguration.head(6);
   for (double instantK = 0.0; instantK < TOTAL_TIME; instantK += TIME_STEP) {
      auto [pe_instantK, Re_instantK, transformationMatrix_instantK] = directKinematics(jointState_instantK);
      quaternion_instantK = Eigen::Quaterniond{Re_instantK};

      Eigen::Vector3d positionalVelocity_instantK =
          (lerp(initialPosition, finalPosition, instantK, maxTime) -
           lerp(initialPosition, finalPosition, instantK - TIME_STEP, maxTime)) /
          TIME_STEP;
      Eigen::Quaterniond quaternionVelocity_instantK =
          slerp(initialQuaternion, finalQuaternion, instantK + TIME_STEP, maxTime) *
          slerp(initialQuaternion, finalQuaternion, instantK, maxTime).conjugate();
      Eigen::Vector3d angularVelocity_instantK = (quaternionVelocity_instantK.vec() * 2.0) / TIME_STEP;

      jacobian_instantK = getJacobian(jointState_instantK);
      double w = calculateDeterminantJJT(jacobian_instantK);
      double lambda = calculateDampingFactor(w, wt, lambda0);
      pseudoInverseJacobian_instantK = calculateDampedPseudoInverse(jacobian_instantK, lambda);
      if (abs(jacobian_instantK.determinant()) < 1.0e-5) {
         singularities++;
         ROS_WARN("NEAR SINGULARITY wt: %f lambda0: %f", wt, lambda0);
      }

      Eigen::Vector3d positionError_instantK = lerp(initialPosition, finalPosition, instantK, maxTime) - pe_instantK;
      Eigen::Quaterniond quaternionError_instantK =
          slerp(initialQuaternion, finalQuaternion, instantK, maxTime) * quaternion_instantK.conjugate();

      velocities_instantK << positionalVelocity_instantK + (Kp * positionError_instantK),
          angularVelocity_instantK + (Kq * quaternionError_instantK.vec());

      auto qdot0 = computeQdot0(jointState_instantK);

      jointStateDot_instantK =
          pseudoInverseJacobian_instantK * velocities_instantK +
          (Eigen::Matrix<double, 6, 6>::Identity() - pseudoInverseJacobian_instantK * jacobian_instantK) * qdot0;
      jointState_instantK += jointStateDot_instantK * TIME_STEP;
      path = Eigen::Matrix<double, 1, 8>{jointState_instantK(0), jointState_instantK(1), jointState_instantK(2),
                                         jointState_instantK(3), jointState_instantK(4), jointState_instantK(5),
                                         gripperState(0),        gripperState(1)};
   }

   return path;
}

PathRow moveRobotOptimization(const Eigen::Matrix<double, 8, 1>& jointConfiguration,
                              const Eigen::Vector3d& finalPosition, const Eigen::Quaterniond& finalQuaternion,
                              const double& lambda0, const double& wt, int& singularities, const double maxTime) {
   auto jointState = getJointState(jointConfiguration);
   auto [pe, Re, transformationMatrix] = directKinematics(jointState);
   Eigen::Quaterniond initialQuaternion(Re);
   return optimizeParamDiffKinQuat(jointConfiguration, pe, initialQuaternion, finalPosition, finalQuaternion, maxTime,
                                   wt, lambda0, singularities);
}