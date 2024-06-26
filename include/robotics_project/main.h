#ifndef MAIN_H
#define MAIN_H

#include "robotics_project/GetBlocks.h"
#include "robotics_project/MovementHandler.h"
#include "robotics_project/utils.h"
#include "ros/ros.h"

/* Number of blocks to be moved */
#define N_BLOCKS 1
// This data should be deducted from the vision's response

/* MAIN functions */
void move(ros::Publisher &pub, Path path);
void move_row(ros::Publisher &pub, PathRow vals);

/* Interactions with movement module */
Path get_movements(robotics_project::MovementHandler::Response &res);

#endif