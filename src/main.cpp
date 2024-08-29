/* Header */
#include "robotics_project/main.h"

#include <chrono>
#include <iostream>
#include <thread>

#include "ros/ros.h"

int main(int argc, char **argv) {
   /* Initialize ROS node's name */
   ros::init(argc, argv, "robotics_project_main");
   std::cout << "MAIN Process: STARTED" << std::endl;

   // ROS handler
   ros::NodeHandle n;

   // Vision Service
   ros::ServiceClient vision_client = n.serviceClient<robotics_project::GetBlocks>("vision");
   robotics_project::GetBlocks vision_srv;

   // Movement Handler Service
   ros::ServiceClient service_client = n.serviceClient<robotics_project::MovementHandler>("movement_handler");
   robotics_project::MovementHandler srv;

   ros::NodeHandle node_handler;
   ros::Publisher pub =
       node_handler.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10);
   Path movements;
   Eigen::Vector3d world_point;
   Eigen::Quaterniond world_orientation;
   std::string block_id;
   int n_blocks = N_BLOCKS;

   Eigen::Matrix<double, 9, 3> world_points{{1.0, 0.8, 1.025},     {1.0, 0.15, 1.225},     {0.0, 0.15, 1.225},
                                            {0.0, 0.8, 1.025},     {-0.001, 0.055, 1.225}, {0.549, 0.089, 1.225},
                                            {0.218, 0.649, 1.070}, {0.636, 0.5, 1.009},    {0.326, 0.307, 1.070}};
   n_blocks = world_points.rows();
   Eigen::Matrix3d rotation_matrix{{-1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}};
   Eigen::Quaterniond rotation_quaternion(rotation_matrix);
   world_orientation = rotation_quaternion;
   std::vector<std::string> blocks_id{
       "X1-Y1-Z2", "X1-Y2-Z1",        "X1-Y2-Z2", "X1-Y2-Z2-CHAMFER", "X1-Y2-Z2-TWINFILLET",
       "X1-Y3-Z2", "X1-Y3-Z2-FILLET", "X1-Y4-Z1", "X1-Y4-Z2"};
   // for-each block, move

   static bool hasVisionFinished = false;
   static int8_t moved = 0;
   // for (int8_t moved = 0; !hasVisionFinished; moved++) {
   while (!hasVisionFinished) {
      // when the vision module is ready, uncomment this block
      vision_srv.request.n_moved_blocks = moved;
      if (vision_client.call(vision_srv)) {
         n_blocks = vision_srv.response.n_blocks;
         hasVisionFinished = vision_srv.response.finished;
         for (int i = 0; i < n_blocks; i++) {
            world_point << vision_srv.response.poses[i].position.x, vision_srv.response.poses[i].position.y,
                vision_srv.response.poses[i].position.z;
            world_orientation = Eigen::Quaterniond(
                vision_srv.response.poses[i].orientation.w, vision_srv.response.poses[i].orientation.x,
                vision_srv.response.poses[i].orientation.y, vision_srv.response.poses[i].orientation.z);
            block_id = vision_srv.response.blocks_id[i];
            float yaw = std::acos(vision_srv.response.poses[i].orientation.z) * 2;
            float angle = yaw * 180 / M_PI;

            std::cout << "MAIN Process: Transmitting block " << i << " of type " << block_id << " and centered in ("
                      << world_point[0] << ", " << world_point[1] << ", " << world_point[2] << ") with an angle of "
                      << angle << "°" << std::endl;

            // Block transmission to movement module
            srv.request.pose.position.x = world_point[0];
            srv.request.pose.position.y = world_point[1];
            srv.request.pose.position.z = world_point[2];
            srv.request.pose.orientation.x = world_orientation.x();
            srv.request.pose.orientation.y = world_orientation.y();
            srv.request.pose.orientation.z = world_orientation.z();
            srv.request.pose.orientation.w = world_orientation.w();
            srv.request.block_id = block_id;
            /* Import the required movements from the module */
            if (service_client.call(srv)) {
               movements = get_movements(srv.response);
               if (movements.rows() <= 0) {
                  std::cerr << "MAIN Process: No movements to be made | block " << i << std::endl;
                  continue;
               }
               std::cout << "MAIN Process: Transmitting movements | block " << i << std::endl;
               moved++;
               move(pub, movements);
            } else {
               std::cerr << "MAIN Process: Failed to call the MOVEMENT HANDLER module | block " << i << std::endl;
               continue;
            }
         }
      } else {
         std::cerr << "MAIN Process: Failed to call the VISION module" << std::endl;
         std::this_thread::sleep_for(std::chrono::seconds(5));
         continue;
      }
      //////////////////////////////////
      // when the vision module is ready, comment this block
      // world_point = world_points.row(i);
      // block_id = blocks_id[i];
      // hasVisionFinished = true;
      //////////////////////////////////
   }

   std::cout << "MAIN Process: ENDED" << std::endl;
   return 0;
}

void move(ros::Publisher &pub, Path path) {
   for (int i = 0; i < path.rows(); i++) {
      move_row(pub, path.row(i));
   }
}

void move_row(ros::Publisher &pub, PathRow vals) {
   // Defining the frequency in Hz
   ros::Rate movement_rate(120);

   /* Standard message that the robot receives */
   std_msgs::Float64MultiArray joint_statement;
   joint_statement.data.assign(vals.begin(), vals.end());

   /* publish the command to the channel, the robot will intercept it */
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