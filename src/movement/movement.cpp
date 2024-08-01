/* Header */
#include "robotics_project/movement/movement.h"

#include <iostream>

#include "ros/ros.h"

int main(int argc, char **argv) {
   /* Initialize ROS node's name */

   // Eigen::Vector3d world_point1(0.636, 0.5, 1.109);
   // Eigen::Vector3d world_point2(0.218, 0.649, 1.170);
   // Eigen::Vector3d world_point3(0.326, 0.307, 1.170);
   // Eigen::Vector3d base_point1 = worldToBaseCoordinates(world_point1);
   // Eigen::Vector3d base_point2 = worldToBaseCoordinates(world_point2);
   // Eigen::Vector3d base_point3 = worldToBaseCoordinates(world_point3);
   // std::cout << "world_point1: " << world_point1 << std::endl;
   // std::cout << "world_point2: " << world_point2 << std::endl;
   // std::cout << "world_point3: " << world_point3 << std::endl;
   // std::cout << "base_point1: " << base_point1 << std::endl;
   // std::cout << "base_point2: " << base_point2 << std::endl;
   // std::cout << "base_point3: " << base_point3 << std::endl;

   // auto trajectory = computeCircularTrajectory(base_point1, base_point2);
   // std::cout << trajectory << std::endl;

   // Eigen::Matrix3d rotation_matrix{{-1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}};
   // runOptimization(base_point, rotation_matrix);

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

   // Eigen::Vector3d world_point1(0.636, 0.5, 1.109);
   // Eigen::Vector3d world_point2(0.218, 0.649, 1.170);
   Eigen::Vector3d world_point3(0.326, 0.307, 1.170);
   // Eigen::Vector3d base_point1 = worldToBaseCoordinates(world_point1);
   // Eigen::Vector3d base_point2 = worldToBaseCoordinates(world_point2);
   Eigen::Vector3d base_point3 = worldToBaseCoordinates(world_point3);
   // std::cout << "world_point1: " << world_point1 << std::endl;
   // std::cout << "world_point2: " << world_point2 << std::endl;
   std::cout << "world_point3: " << world_point3.transpose() << std::endl;
   // std::cout << "base_point1: " << base_point1 << std::endl;
   // std::cout << "base_point2: " << base_point2 << std::endl;
   std::cout << "base_point3: " << base_point3.transpose() << std::endl;

   Eigen::Matrix3d rotation_matrix{{-1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}};
   Eigen::Quaterniond rotation_quaternion(rotation_matrix);

   Path movements = get_path(base_point3, rotation_quaternion);

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

   // Eigen::Matrix3d rotation_matrix{{-1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}};

   // Path p = moveRobot(base_point1, rotation_matrix);
   // Path p2 = moveRobot(base_point2, rotation_matrix);
   // // Path p3 = moveRobot(base_point3, rotation_matrix);
   // insertPath(p, p2);
   // insertPath(p, p3);
   ROS_INFO("FINISH moveRobot");

   Eigen::Matrix<double, 8, 1> joint_configuration = path.row(path.rows() - 1);
   Eigen::Matrix<double, 6, 1> joint{joint_configuration(0), joint_configuration(1), joint_configuration(2),
                                     joint_configuration(3), joint_configuration(4), joint_configuration(5)};

   const auto &[peFinal, ReFinal, transformation_matrixFinal] = directKinematics(joint);
   std::cout << "joint_configuration" << std::endl;
   for (auto val : joint_configuration) {
      std::cout << val << " ";
   }
   std::cout << "\njoint" << std::endl;
   for (auto val : joint) {
      std::cout << val << " ";
   }
   std::cout << "\ntransformation matrix" << std::endl;
   for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
         std::cout << transformation_matrixFinal.row(i).col(j) << " ";
      }
      std::cout << std::endl;
   }
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

   std::cout << "\n\njacobian" << std::endl;
   auto jacobian = getJacobian(joint);
   std::cout << jacobian << std::endl;
   std::cout.flush();

   // for (int i = 0; i < path.rows(); i++) {
   //    std::cout << "path[" << i << "]: ";
   //    Eigen::Matrix<double, 8, 1> joint_configuration_i = path.row(i);
   //    Eigen::Matrix<double, 6, 1> joint_i{joint_configuration_i(0), joint_configuration_i(1),
   //    joint_configuration_i(2),
   //                                        joint_configuration_i(3), joint_configuration_i(4),
   //                                        joint_configuration_i(5)};
   //    auto [pei, Rei, transformation_matrixi] = directKinematics(joint_i);
   //    std::cout << pei.transpose() << std::endl;
   // }
   // // insertPath(p, initial_configuration);

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