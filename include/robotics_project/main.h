/*!
    @file main.h
    @brief Functions declaration of the client service that communicates with the vision node and the movement_handler
    @author Nicolo' Marconi and Nizar Nadif
*/
#ifndef MAIN_H
#define MAIN_H

#include "robotics_project/GetBlocks.h"
#include "robotics_project/MovementHandler.h"
#include "robotics_project/utils.h"
#include "ros/ros.h"

/*!
    @brief Function to iterate over the joint configurations of the robot
    @param[in] pub: publisher to the topic /ur5/joint_group_pos_controller/command to send the joints configurations to
   the robot
    @param[in] path: path matrix containing the joints configurations of the movement
*/
void move(ros::Publisher &pub, Path path);
/*!
    @brief Function to send the joint configurations to the robot
    @param[in] pub: publisher to the topic /ur5/joint_group_pos_controller/command to send the joints configurations to
   the robot
    @param[in] vals: path row containing the joints configurations of the movement
*/
void move_row(ros::Publisher &pub, PathRow vals);

/*!
    @brief Function to convert the response of the service movement_handler to a path matrix
    @param[in] res: response of the service movement_handler containing the joints configurations of the movement
   the robot
    @return Path: path matrix containing the joints configurations of the movement
*/
Path get_movements(robotics_project::MovementHandler::Response &res);

#endif