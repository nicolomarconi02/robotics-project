/* Header */
#include "robotics_project/movement/movement.h"

#include <iostream>

#include "ros/ros.h"

Path get_path();

int main(int argc, char **argv) {
   /* Initialize ROS node's name */

   // Eigen::Vector3d world_point(0.218, 0.649, 1.17);
   // Eigen::Vector3d base_point = worldToBaseCoordinates(world_point);
   // std::cout << "world_point: " << world_point << std::endl;
   // std::cout << "base_point: " << base_point << std::endl;

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
   Path movements = get_path();

   res = get_response(movements);

   return true;
}

/***
 * Dummy function
 */
Path get_path() {
   Eigen::Matrix<double, 8, 1> initial_configuration = getJointConfiguration();
   for (auto val : initial_configuration) {
      std::cout << val << " ";
   }

   Eigen::Vector3d world_point1(0.636, 0.5, 1.109);
   Eigen::Vector3d world_point2(0.218, 0.649, 1.170);
   // Eigen::Vector3d world_point3(0.326, 0.307, 1.170);
   Eigen::Vector3d base_point1 = worldToBaseCoordinates(world_point1);
   Eigen::Vector3d base_point2 = worldToBaseCoordinates(world_point2);
   // Eigen::Vector3d base_point3 = worldToBaseCoordinates(world_point3);
   std::cout << "world_point1: " << world_point1 << std::endl;
   std::cout << "world_point2: " << world_point2 << std::endl;
   // std::cout << "world_point3: " << world_point3 << std::endl;
   std::cout << "base_point1: " << base_point1 << std::endl;
   std::cout << "base_point2: " << base_point2 << std::endl;
   // std::cout << "base_point3: " << base_point3 << std::endl;

   Eigen::Matrix3d rotation_matrix{{-1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}};

   ROS_INFO("BEGIN moveRobot");
   Path p = moveRobot(base_point1, rotation_matrix);
   Path p2 = moveRobot(base_point2, rotation_matrix);
   // Path p3 = moveRobot(base_point3, rotation_matrix);
   insertPath(p, p2);
   // insertPath(p, p3);
   ROS_INFO("FINISH moveRobot");

   Eigen::Matrix<double, 8, 1> joint_configuration = p.row(p.rows() - 1);
   Eigen::Matrix<double, 6, 1> joint{joint_configuration(0), joint_configuration(1), joint_configuration(2),
                                     joint_configuration(3), joint_configuration(4), joint_configuration(5)};

   const auto &[pe, Re, transformation_matrix] = directKinematics(joint);
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
         std::cout << transformation_matrix.row(i).col(j) << " ";
      }
      std::cout << std::endl;
   }
   std::cout << "\nRe" << std::endl;
   for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
         std::cout << Re.row(i).col(j) << " ";
      }
      std::cout << std::endl;
   }
   std::cout << "\npe" << std::endl;
   for (auto val : pe) {
      std::cout << val << " ";
   }

   std::cout << "\n\njacobian" << std::endl;
   auto jacobian = getJacobian(joint);
   std::cout << jacobian << std::endl;
   std::cout.flush();

   // insertPath(p, initial_configuration);

   ROS_INFO("END get_path");
   return p;
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