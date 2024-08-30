/*!
    @file movement.cpp
    @brief Functions definition for the service movement_handler
    @author Nicolo' Marconi
*/
#include "robotics_project/movement/movement.h"

#include <fstream>
#include <iostream>

#include "ros/ros.h"

int main(int argc, char **argv) {
   /* Initialize ROS node's name */
   std::cout << "MOVEMENT HANDLER Process: STARTED" << std::endl;

   // Uncomment to run optimization
   // std::cout << "OPTIMIZATION FOR DIFFERENTIAL KINEMATICS" << std::endl;
   // runOptimization();
   ////////////////////////////////////////7

   ros::init(argc, argv, "robotics_project_movement_handler");

   // Define service server
   ros::NodeHandle n;
   ros::ServiceServer service = n.advertiseService("movement_handler", movHandler);
   // and start it
   ros::spin();

   std::cout << "MOVEMENT HANDLER Process: ENDED" << std::endl;
   return 0;
}

bool movHandler(robotics_project::MovementHandler::Request &req, robotics_project::MovementHandler::Response &res) {
   std::cout << "MOVEMENT HANDLER Process: Service CALLED" << std::endl;

   Eigen::Vector3d brickPosition(req.pose.position.x, req.pose.position.y, req.pose.position.z);
   Eigen::Quaterniond rotationQuaternion(req.pose.orientation.w, req.pose.orientation.x, req.pose.orientation.y,
                                         req.pose.orientation.z);
   // Convert the brick position from world to base coordinates
   brickPosition = worldToBaseCoordinates(brickPosition);
   // Subtract the tool size from the z coordinate
   brickPosition -= Eigen::Vector3d{0.0, 0.0, TOOL_SIZE};
   std::string blockId = req.block_id;
   // Compute the path of the robot's movement
   Path movements = getPath(brickPosition, rotationQuaternion, blockId);
   // Fill the response of the service movement_handler
   res = getResponse(movements);

   return true;
}

Path getPath(const Eigen::Vector3d &brickPosition, const Eigen::Quaterniond &brickOrientation,
             const std::string &blockId) {
   ROS_INFO("%s", blockId.c_str());
   // Get the final position of the block on the elevated part of the table
   Eigen::Vector3d finalPosition = getFinalPosition(blockId);
   // Check if the block id is valid
   if (finalPosition.norm() < 1e-3) {
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
   Trajectory trajectoryInitialToBrick =
       computeCircularTrajectory(initialPositionStdHeight, brickPositionStdHeight, path.row(path.rows() - 1));
   std::cout << "trajectoryInitialToBrick: " << std::endl;
   std::cout << trajectoryInitialToBrick << std::endl;
   insertPath(path, moveRobot(path.row(path.rows() - 1), trajectoryInitialToBrick.row(0), Re, TOTAL_TIME));
   for (int i = 1; i < trajectoryInitialToBrick.rows(); i++) {
      auto jointConfiguration_i = path.row(path.rows() - 1);
      insertPath(path, moveRobot(jointConfiguration_i, trajectoryInitialToBrick.row(i), Re, 1.0));
   }
   insertPath(path, moveRobot(path.row(path.rows() - 1), brickPositionStdHeight, Re, TOTAL_TIME));
   ROS_INFO("FINISH move to brick standard height");
   // OPEN GRIPPER
   insertPath(path, toggleGripper(path.row(path.rows() - 1), GripperState_::OPEN, blockId));
   // MOVEMENT FROM BRICK STANDARD HEIGHT POSITION TO BRICK POSITION

   // Compute radiant angle
   double angle = std::acos(brickOrientation.z()) * 2;

   double graspingAngle = angle + M_PI / 2;

   if (graspingAngle > M_PI) {
      graspingAngle -= 2 * M_PI;
   } else if (graspingAngle < -M_PI) {
      graspingAngle += 2 * M_PI;
   }

   Eigen::Matrix3d graspingRotationMatrix = rotationMatrixAroundZ(graspingAngle);
   Eigen::Quaterniond graspingOrientation(graspingRotationMatrix);
   insertPath(path, moveRobot(path.row(path.rows() - 1), brickPosition, graspingOrientation));
   // CLOSE GRIPPER
   insertPath(path, toggleGripper(path.row(path.rows() - 1), GripperState_::CLOSE, blockId));

   // MOVEMENT FROM BRICK POSITION TO BRICK STANDARD HEIGHT POSITION
   insertPath(path, moveRobot(path.row(path.rows() - 1), brickPositionStdHeight, graspingOrientation));

   // COMPUTE FINAL POSITION
   Eigen::Vector3d finalPositionStdHeight(finalPosition(0), finalPosition(1), STD_HEIGHT);
   Eigen::Matrix3d finalRe{{-1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}};
   Eigen::Quaterniond finalRotationQuaternion(finalRe);

   std::cout << "finalPosition: " << finalPosition.transpose() << std::endl;
   std::cout << "finalPositionStdHeight: " << finalPositionStdHeight.transpose() << std::endl;

   // MOVEMENT FROM BRICK STANDARD HEIGHT POSITION TO FINAL STANDARD HEIGHT POSITION
   Trajectory trajectoryBrickToFinal =
       computeCircularTrajectory(brickPositionStdHeight, finalPositionStdHeight, path.row(path.rows() - 1));
   std::cout << "trajectoryBrickToFinal: " << std::endl;
   std::cout << trajectoryBrickToFinal << std::endl;
   insertPath(path, moveRobot(path.row(path.rows() - 1), trajectoryBrickToFinal.row(0), Re, TOTAL_TIME));
   for (int i = 1; i < trajectoryBrickToFinal.rows(); i++) {
      auto jointConfiguration_i = path.row(path.rows() - 1);
      insertPath(path, moveRobot(jointConfiguration_i, trajectoryBrickToFinal.row(i), graspingOrientation, 1.0));
   }
   insertPath(path, moveRobot(path.row(path.rows() - 1), finalPositionStdHeight, graspingOrientation, TOTAL_TIME));
   ROS_INFO("FINISH move to final standard height");

   // MOVEMENT FROM FINAL STANDARD HEIGHT POSITION TO FINAL POSITION
   insertPath(path, moveRobot(path.row(path.rows() - 1), finalPosition, finalRotationQuaternion));

   // OPEN GRIPPER
   insertPath(path, toggleGripper(path.row(path.rows() - 1), GripperState_::OPEN, blockId));
   insertPath(path, moveRobot(path.row(path.rows() - 1), finalPositionStdHeight, finalRotationQuaternion));
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
   std::cout << std::endl;
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

void runOptimization() {
   // Open the file to write the results of the optimization
   std::ofstream file("optimization_results.txt", std::ios::app);
   if (!file.is_open()) {
      std::cerr << "Error opening file" << std::endl;
      return;
   }
   // Homing position of the robot
   Eigen::Matrix<double, 8, 1> initialJointConfiguration = Eigen::Matrix<double, 8, 1>{
       -0.320096, -0.780249, -2.56081, -1.63051, -1.5705, 3.4911, -4.38614e-05, 7.69203e-06};
   // World coordinates of the blocks
   Eigen::Matrix<double, 9, 3> world_points{{1.0, 0.8, 1.025},     {1.0, 0.15, 1.225},     {0.0, 0.15, 1.225},
                                            {0.0, 0.8, 1.025},     {-0.001, 0.055, 1.225}, {0.549, 0.089, 1.225},
                                            {0.218, 0.649, 1.070}, {0.636, 0.5, 1.009},    {0.326, 0.307, 1.070}};
   int n_blocks = world_points.rows();
   // Orientation of the blocks
   Eigen::Matrix3d rotation_matrix{{-1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}};
   Eigen::Quaterniond rotation_quaternion(rotation_matrix);
   // Block ids
   std::vector<std::string> blocks_id{
       "X1-Y1-Z2", "X1-Y2-Z1",        "X1-Y2-Z2", "X1-Y2-Z2-CHAMFER", "X1-Y2-Z2-TWINFILLET",
       "X1-Y3-Z2", "X1-Y3-Z2-FILLET", "X1-Y4-Z1", "X1-Y4-Z2"};
   // Data structure to store the results of the optimization
   struct movement_data {
      double lamda0;
      double wt;
      double deltaPos;
      int singularities;
   };

   std::vector<movement_data> movements_data;
   // Iterate over the blocks
   for (int i = 0; i < n_blocks; i++) {
      std::cout << "i: " << i << std::endl;
      std::cout << "BLOCK ID: " << blocks_id[i] << std::endl;
      Eigen::Vector3d brickPosition = world_points.row(i);
      std::string blockId = blocks_id[i];
      brickPosition = worldToBaseCoordinates(brickPosition);
      Eigen::Vector3d finalPosition = getFinalPosition(blockId);
      if (finalPosition.norm() < 1e-3) {
         ROS_ERROR("UNKNOWN BLOCK ID");
         ROS_ERROR("STOPPING MOVEMENT");
         continue;
      }
      Eigen::Matrix<double, 6, 1> initialJointState = initialJointConfiguration.head(6);
      auto [initialPosition, Re, transformation_matrix] = directKinematics(initialJointState);
      Eigen::Quaterniond initialOrientation(Re);
      Eigen::Vector3d initialPositionStdHeight(initialPosition(0), initialPosition(1), STD_HEIGHT);
      Eigen::Vector3d brickPositionStdHeight(brickPosition(0), brickPosition(1), STD_HEIGHT);
      Eigen::Vector3d finalPositionStdHeight(finalPosition(0), finalPosition(1), STD_HEIGHT);
      Eigen::Matrix3d finalRe{{-1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}};
      Eigen::Quaterniond finalRotationQuaternion(finalRe);
      // Starting values for the optimization
      double currWt = 0.001;
      double currLambda0 = 1e-8;
      Eigen::Vector3d minDelta(1000.0, 1000.0, 1000.0);
      double min = 1000.0;
      int currSingularities = 0;
      // Iterate over the values of lambda0 and wt
      for (double lambda0 = 1e-8; lambda0 < 0.1e-4; lambda0 *= 10) {
         for (double wt = 0.01; wt < 1.0; wt += 0.01) {
            int singularities = 0;
            std::cout << "lambda0: " << lambda0 << " wt: " << wt << std::endl;
            PathRow path = moveRobotOptimization(initialJointConfiguration, initialPositionStdHeight,
                                                 initialOrientation, lambda0, wt, singularities);
            // MOVEMENT FROM INITIAL STANDARD HEIGHT POSITION TO BRICK STANDARD HEIGHT POSITION
            Trajectory trajectoryInitialToBrick =
                computeCircularTrajectory(initialPositionStdHeight, brickPositionStdHeight, path.row(path.rows() - 1));
            path = moveRobotOptimization(path.row(path.rows() - 1), trajectoryInitialToBrick.row(0), initialOrientation,
                                         lambda0, wt, singularities, TOTAL_TIME);
            for (int i = 1; i < trajectoryInitialToBrick.rows(); i++) {
               auto jointConfiguration_i = path.row(path.rows() - 1);
               path = moveRobotOptimization(jointConfiguration_i, trajectoryInitialToBrick.row(i), initialOrientation,
                                            lambda0, wt, singularities, 1.0);
            }
            path = moveRobotOptimization(path.row(path.rows() - 1), brickPositionStdHeight, initialOrientation, lambda0,
                                         wt, singularities);
            // OPEN GRIPPER
            path = toggleGripper(path.row(path.rows() - 1), GripperState_::OPEN, blockId);
            path = moveRobotOptimization(path.row(path.rows() - 1), brickPositionStdHeight, initialOrientation, lambda0,
                                         wt, singularities);
            // MOVEMENT FROM BRICK STANDARD HEIGHT POSITION TO BRICK POSITION
            double graspingAngle = finalRotationQuaternion.z() + M_PI / 2;
            Eigen::Matrix3d graspingRotationMatrix = rotationMatrixAroundZ(graspingAngle);
            Eigen::Quaterniond graspingOrientation(graspingRotationMatrix);
            path = moveRobotOptimization(path.row(path.rows() - 1), brickPosition, graspingOrientation, lambda0, wt,
                                         singularities);
            // CLOSE GRIPPER
            path = toggleGripper(path.row(path.rows() - 1), GripperState_::CLOSE, blockId);
            path = moveRobotOptimization(path.row(path.rows() - 1), brickPosition, graspingOrientation, lambda0, wt,
                                         singularities);

            // MOVEMENT FROM BRICK POSITION TO BRICK STANDARD HEIGHT POSITION
            path = moveRobotOptimization(path.row(path.rows() - 1), brickPositionStdHeight, graspingOrientation,
                                         lambda0, wt, singularities);

            // MOVEMENT FROM BRICK STANDARD HEIGHT POSITION TO FINAL STANDARD HEIGHT POSITION
            Trajectory trajectoryBrickToFinal =
                computeCircularTrajectory(brickPositionStdHeight, finalPositionStdHeight, path.row(path.rows() - 1));
            path = moveRobotOptimization(path.row(path.rows() - 1), trajectoryBrickToFinal.row(0), initialOrientation,
                                         lambda0, wt, singularities, TOTAL_TIME);
            for (int i = 1; i < trajectoryBrickToFinal.rows(); i++) {
               auto jointConfiguration_i = path.row(path.rows() - 1);
               path = moveRobotOptimization(jointConfiguration_i, trajectoryBrickToFinal.row(i), initialOrientation,
                                            lambda0, wt, singularities, 1.0);
            }
            path = moveRobotOptimization(path.row(path.rows() - 1), finalPositionStdHeight, initialOrientation, lambda0,
                                         wt, singularities);
            // MOVEMENT FROM FINAL STANDARD HEIGHT POSITION TO FINAL POSITION
            path = moveRobotOptimization(path.row(path.rows() - 1), finalPosition, finalRotationQuaternion, lambda0, wt,
                                         singularities);
            // update the minimum distance
            Eigen::Matrix<double, 6, 1> js = path.head(6);
            auto [pe_i, Re_i, transformationMatrix_i] = directKinematics(js);
            double tmp = (finalPosition - pe_i).norm();
            if (tmp < min) {
               min = tmp;
               minDelta = finalPosition - pe_i;
               currWt = wt;
               currLambda0 = lambda0;
               currSingularities = singularities;
            }

            // OPEN GRIPPER
            path = toggleGripper(path.row(path.rows() - 1), GripperState_::OPEN, blockId);
            path = moveRobotOptimization(path.row(path.rows() - 1), finalPositionStdHeight, finalRotationQuaternion,
                                         lambda0, wt, singularities);
         }
      }
      // Store the results of the optimization
      movements_data.push_back({currLambda0, currWt, minDelta.norm(), currSingularities});
   }
   // Write the results of the optimization in the file
   for (auto data : movements_data) {
      std::cout << "lambda0: " << data.lamda0 << " wt: " << data.wt << " deltaPos: " << data.deltaPos
                << " singularities: " << data.singularities << std::endl;
      file << "lambda0: " << data.lamda0 << " wt: " << data.wt << " deltaPos: " << data.deltaPos
           << " singularities: " << data.singularities << std::endl;
   }
   file.close();
}
