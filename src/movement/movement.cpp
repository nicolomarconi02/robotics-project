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
   ros::ServiceServer service = n.advertiseService("movement_handler", mov_handler);
   // and start it
   ros::spin();

   std::cout << "MOVEMENT HANDLER Process: ENDED" << std::endl;
   return 0;
}

/* Service function */
bool mov_handler(robotics_project::MovementHandler::Request &req, robotics_project::MovementHandler::Response &res) {
   std::cout << "MOVEMENT HANDLER Process: Service CALLED" << std::endl;

   Eigen::Vector3d brickPosition(req.pose.position.x, req.pose.position.y, req.pose.position.z);
   Eigen::Quaterniond rotation_quaternion(req.pose.orientation.w, req.pose.orientation.x, req.pose.orientation.y,
                                          req.pose.orientation.z);
   brickPosition = worldToBaseCoordinates(brickPosition);
   Path movements = get_path(brickPosition, rotation_quaternion);

   res = get_response(movements);

   return true;
}

Path get_path(const Eigen::Vector3d &brickPosition, const Eigen::Quaterniond &brickOrientation) {
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
   Trajectory trajectory = computeCircularTrajectory(initialPositionStdHeight, brickPositionStdHeight);
   std::cout << "trajectory: " << std::endl;
   std::cout << trajectory << std::endl;
   for (int i = 0; i < trajectory.rows(); i++) {
      auto jointConfiguration_i = path.row(path.rows() - 1);
      insertPath(path, moveRobot(jointConfiguration_i, trajectory.row(i), Re, 1.0));
   }
   ROS_INFO("FINISH move to brick standard height");

   // MOVEMENT FROM BRICK STANDARD HEIGHT POSITION TO BRICK POSITION
   auto jointConfiguration_BRICKSTDHEIGHT = path.row(path.rows() - 1);
   insertPath(path, moveRobot(jointConfiguration_BRICKSTDHEIGHT, brickPosition, brickOrientation));
   ROS_INFO("FINISH moveRobot");

   Eigen::Matrix<double, 8, 1> joint_configuration = path.row(path.rows() - 1);
   Eigen::Matrix<double, 6, 1> joint{joint_configuration(0), joint_configuration(1), joint_configuration(2),
                                     joint_configuration(3), joint_configuration(4), joint_configuration(5)};

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

   ROS_INFO("END get_path");
   return path;
}

robotics_project::MovementHandler::Response get_response(Path movements) {
   robotics_project::MovementHandler::Response res;
   res.path.movements.resize(movements.rows());

   // Assigning, in a safe manner, each row of the Path to the response's counterpart
   for (int i = 0; i < movements.rows(); i++)
      // For clarification: res.path.movements[i] = movements.row(i)
      res.path.movements[i].data.assign(movements.row(i).begin(), movements.row(i).end());

   return res;
}