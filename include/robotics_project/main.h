#include "Eigen/Dense"
#include "ros/ros.h"

#include "robotics_project/robotics_project.h"
#include "robotics_project/MovementHandler.h"

/* Number of blocks to be moved */
#define N_BLOCKS 1
// This data should be deducted from the vision's response

/* MAIN functions */
void move(ros::Publisher &pub, Path path);
void move_row(ros::Publisher &pub, PathRow vals);

/* Interactions with movement module */
Path get_movements(robotics_project::MovementHandler::Response &res);