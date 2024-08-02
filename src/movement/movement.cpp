/* Header */
#include "robotics_project/movement/movement.h"

#include <iostream>

#include "ros/ros.h"

int main(int argc, char **argv) {
   /* Initialize ROS node's name */

   ros::init(argc, argv, "robotics_project_movement_handler");
   std::cout << "MOVEMENT HANDLER Process: STARTED" << std::endl;

   /* Define service server */
   ros::NodeHandle n;
   ros::ServiceServer service = n.advertiseService("movement_handler", movHandler);
   // and start it
   ros::spin();

   std::cout << "MOVEMENT HANDLER Process: ENDED" << std::endl;
   return 0;
}

/* Service function */
bool movHandler(robotics_project::MovementHandler::Request &req, robotics_project::MovementHandler::Response &res) {
   std::cout << "MOVEMENT HANDLER Process: Service CALLED" << std::endl;

   Eigen::Vector3d brickPosition(req.pose.position.x, req.pose.position.y, req.pose.position.z);
   Eigen::Quaterniond rotationQuaternion(req.pose.orientation.w, req.pose.orientation.x, req.pose.orientation.y,
                                         req.pose.orientation.z);
   brickPosition = worldToBaseCoordinates(brickPosition);
   std::string blockId = req.block_id;
   Path movements = getPath(brickPosition, rotationQuaternion, blockId);

   res = getResponse(movements);

   return true;
}

Path getPath(const Eigen::Vector3d &brickPosition, const Eigen::Quaterniond &brickOrientation,
             const std::string &blockId) {
   Eigen::Vector3d finalPosition = getFinalPosition(blockId);
   if (finalPosition == Eigen::Vector3d::Zero()) {
      ROS_ERROR("UNKNOWN BLOCK ID");
      ROS_ERROR("STOPPING MOVEMENT");
      return Path();
   }
   ROS_INFO("BEGIN moveRobot");
   // MOVEMENT FROM INITIAL POSITION TO INITIAL STANDARD HEIGHT POSITION
   auto [jointState0, jointConfiguration0] = getJointConfiguration();

   auto [initialPosition, Re, transformation_matrix] = directKinematics(jointState0);
   Eigen::Vector3d initialPositionStdHeight(initialPosition(0), initialPosition(1), STD_HEIGHT);
   Eigen::Vector3d brickPositionStdHeight(brickPosition(0), brickPosition(1), STD_HEIGHT);
   std::cout << "initialPosition: " << initialPosition.transpose() << std::endl;
   std::cout << "initialPositionStdHeight: " << initialPositionStdHeight.transpose() << std::endl;
   std::cout << "brickPosition: " << brickPosition.transpose() << std::endl;
   std::cout << "brickPositionStdHeight: " << brickPositionStdHeight.transpose() << std::endl;
   Path path = moveRobot(jointConfiguration0, initialPositionStdHeight, Re);
   ROS_INFO("FINISH move to initial standard height");

   // MOVEMENT FROM INITIAL STANDARD HEIGHT POSITION TO BRICK STANDARD HEIGHT POSITION
   Trajectory trajectoryInitialToBrick = computeCircularTrajectory(initialPositionStdHeight, brickPositionStdHeight);
   std::cout << "trajectoryInitialToBrick: " << std::endl;
   std::cout << trajectoryInitialToBrick << std::endl;
   for (int i = 0; i < trajectoryInitialToBrick.rows(); i++) {
      auto jointConfiguration_i = path.row(path.rows() - 1);
      insertPath(path, moveRobot(jointConfiguration_i, trajectoryInitialToBrick.row(i), Re, 1.0));
   }
   ROS_INFO("FINISH move to brick standard height");
   // OPEN GRIPPER
   insertPath(path,
              moveRobot(toggleGripper(path.row(path.rows() - 1), GripperState_::OPEN), brickPositionStdHeight, Re));
   // MOVEMENT FROM BRICK STANDARD HEIGHT POSITION TO BRICK POSITION
   double graspingAngle = brickOrientation.z() + M_PI / 2;
   Eigen::Matrix3d graspingRotationMatrix = rotationMatrixAroundZ(graspingAngle);
   Eigen::Quaterniond graspingOrientation(graspingRotationMatrix);
   insertPath(path, moveRobot(path.row(path.rows() - 1), brickPosition, graspingOrientation));
   // CLOSE GRIPPER
   insertPath(path, moveRobot(toggleGripper(path.row(path.rows() - 1), GripperState_::CLOSE), brickPosition,
                              graspingOrientation));

   // MOVEMENT FROM BRICK POSITION TO BRICK STANDARD HEIGHT POSITION
   insertPath(path, moveRobot(path.row(path.rows() - 1), brickPositionStdHeight, graspingOrientation));

   // COMPUTE FINAL POSITION
   Eigen::Vector3d finalPositionStdHeight(finalPosition(0), finalPosition(1), STD_HEIGHT);
   Eigen::Matrix3d finalRe{{-1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}};
   Eigen::Quaterniond finalRotationQuaternion(finalRe);

   std::cout << "finalPosition: " << finalPosition.transpose() << std::endl;
   std::cout << "finalPositionStdHeight: " << finalPositionStdHeight.transpose() << std::endl;

   // MOVEMENT FROM BRICK STANDARD HEIGHT POSITION TO FINAL STANDARD HEIGHT POSITION
   Trajectory trajectoryBrickToFinal = computeCircularTrajectory(brickPositionStdHeight, finalPositionStdHeight);
   std::cout << "trajectoryBrickToFinal: " << std::endl;
   std::cout << trajectoryBrickToFinal << std::endl;
   for (int i = 0; i < trajectoryBrickToFinal.rows(); i++) {
      auto jointConfiguration_i = path.row(path.rows() - 1);
      insertPath(path, moveRobot(jointConfiguration_i, trajectoryBrickToFinal.row(i), Re, 1.0));
   }
   ROS_INFO("FINISH move to final standard height");

   // MOVEMENT FROM FINAL STANDARD HEIGHT POSITION TO FINAL POSITION
   insertPath(path, moveRobot(path.row(path.rows() - 1), finalPosition, finalRotationQuaternion));

   // OPEN GRIPPER
   insertPath(path, moveRobot(toggleGripper(path.row(path.rows() - 1), GripperState_::OPEN), finalPositionStdHeight,
                              finalRotationQuaternion));
   ROS_INFO("FINISH moveRobot");

   // THIS IS ONLY FOR DEBUGGING
   Eigen::Matrix<double, 8, 1> jointConfiguration = path.row(path.rows() - 1);
   Eigen::Matrix<double, 6, 1> joint{jointConfiguration(0), jointConfiguration(1), jointConfiguration(2),
                                     jointConfiguration(3), jointConfiguration(4), jointConfiguration(5)};

   const auto &[peFinal, ReFinal, transformation_matrixFinal] = directKinematics(joint);

   std::cout << "\nReFinal" << std::endl;
   for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
         std::cout << ReFinal.row(i).col(j) << " ";
      }
      std::cout << std::endl;
   }
   std::cout << "\npeFinal" << std::endl;
   for (auto val : peFinal) {
      std::cout << val << " ";
   }
   //////////////////////////////////////
   ROS_INFO("END getPath");
   return path;
}

robotics_project::MovementHandler::Response getResponse(Path movements) {
   robotics_project::MovementHandler::Response res;
   res.path.movements.resize(movements.rows());

   // Assigning, in a safe manner, each row of the Path to the response's counterpart
   for (int i = 0; i < movements.rows(); i++)
      // For clarification: res.path.movements[i] = movements.row(i)
      res.path.movements[i].data.assign(movements.row(i).begin(), movements.row(i).end());

   return res;
}