/*!
    @file main.cpp
    @brief Functions definition of the client service that communicates with the vision node and the movement_handler
    @author Nicolo' Marconi and Nizar Nadif
*/
#include "robotics_project/main.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

#include "ros/ros.h"

int main(int argc, char **argv) {
   /* Initialize ROS node's name */
   ros::init(argc, argv, "robotics_project_main");
   std::cout << "CLIENT Process: STARTED" << std::endl;

   // ROS handler
   ros::NodeHandle n;

   // Vision Service
   ros::ServiceClient vision_client = n.serviceClient<robotics_project::GetBlocks>("vision");
   robotics_project::GetBlocks vision_srv;

   // Movement Handler Service
   ros::ServiceClient service_client = n.serviceClient<robotics_project::MovementHandler>("movement_handler");
   robotics_project::MovementHandler srv;

   ros::NodeHandle node_handler;
   // Publisher to the robot
   ros::Publisher pub =
       node_handler.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10);
   Path movements;
   Eigen::Vector3d world_point;
   Eigen::Quaterniond world_orientation;
   std::string block_id;
   int n_blocks = 0;

   static bool hasVisionFinished = false;
   // Continue until the vision module has finished
   while (!hasVisionFinished) {
      // Call the vision module
      if (vision_client.call(vision_srv)) {
         n_blocks = vision_srv.response.n_blocks;
         std::cout << "Successfully called the VISION module: " << n_blocks << " block" << (n_blocks != 1 ? "s" : "")
                   << " received" << std::endl;
         hasVisionFinished = vision_srv.response.finished;
         // Iterate over the recognized blocks
         for (int i = 0; i < n_blocks; i++) {
            world_point << vision_srv.response.poses[i].position.x, vision_srv.response.poses[i].position.y,
                vision_srv.response.poses[i].position.z;
            world_orientation = Eigen::Quaterniond(
                vision_srv.response.poses[i].orientation.w, vision_srv.response.poses[i].orientation.x,
                vision_srv.response.poses[i].orientation.y, vision_srv.response.poses[i].orientation.z);
            block_id = vision_srv.response.blocks_id[i];

            float yaw = std::acos(vision_srv.response.poses[i].orientation.z) * 2;
            if (yaw > M_PI) {
               yaw -= 2 * M_PI;
            } else if (yaw < -M_PI) {
               yaw += 2 * M_PI;
            }

            float angle = yaw * 180 / M_PI;

            std::cout << "Block " << i + 1 << ": " << block_id << " in (" << std::setprecision(3) << world_point[0]
                      << ", " << world_point[1] << ", " << world_point[2] << "), angle = " << (int)angle
                      << "° = " << yaw << " rad" << std::endl;

            // Block transmission to movement module
            srv.request.pose.position.x = world_point[0];
            srv.request.pose.position.y = world_point[1];
            srv.request.pose.position.z = world_point[2];
            srv.request.pose.orientation.x = world_orientation.x();
            srv.request.pose.orientation.y = world_orientation.y();
            srv.request.pose.orientation.z = world_orientation.z();
            srv.request.pose.orientation.w = world_orientation.w();
            srv.request.block_id = block_id;
            // Call the movement handler module
            if (service_client.call(srv)) {
               // Get the movements from the response
               movements = get_movements(srv.response);
               if (movements.rows() <= 0) {
                  std::cerr << "No movements to be made | block " << i + 1 << std::endl;
                  continue;
               }
               std::cout << "Transmitting movements | block " << i + 1 << std::endl;
               // Send the movements to the robot
               move(pub, movements);
            } else {
               std::cerr << "Failed to call the MOVEMENT HANDLER module | block " << i + 1 << std::endl;
               continue;
            }
         }
      } else {
         std::cerr << "Failed to call the VISION module" << std::endl;
         // Wait for 5 seconds before trying again if the vision module is not ready
         std::this_thread::sleep_for(std::chrono::seconds(5));
         continue;
      }
   }

   std::cout << "CLIENT Process: ENDED" << std::endl;
   return 0;
}

void move(ros::Publisher &pub, Path path) {
   // Iterating over the rows of the path
   for (int i = 0; i < path.rows(); i++) {
      move_row(pub, path.row(i));
   }
}

void move_row(ros::Publisher &pub, PathRow vals) {
   // Defining the frequency in Hz
   ros::Rate movement_rate(120);

   // Standard message that the robot receives
   std_msgs::Float64MultiArray joint_statement;
   joint_statement.data.assign(vals.begin(), vals.end());

   // publish the command to the channel, the robot will intercept it
   pub.publish(joint_statement);
   // and then let's go to sleep
   movement_rate.sleep();
}

Path get_movements(robotics_project::MovementHandler::Response &res) {
   Path movements;

   // Increasing the number of rows
   movements.conservativeResize(res.path.movements.size(), movements.cols());

   // Digesting the data from the response
   for (int i = 0; i < movements.rows(); i++)
      for (int j = 0; j < movements.cols(); j++) movements.row(i).col(j) << res.path.movements[i].data[j];

   return movements;
}