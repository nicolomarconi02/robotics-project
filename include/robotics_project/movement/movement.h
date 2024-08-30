/*!
    @file movement.h
    @brief Functions declaration for the service movement_handler
    @author Nicolo' Marconi
*/
#ifndef MOVEMENT_HANDLER_H
#define MOVEMENT_HANDLER_H

#include "robotics_project/MovementHandler.h"
#include "robotics_project/utils.h"

/*!
    @brief Function of the service movement_handler
    @param[in] req: request value
    @param[in] res: response value
    @return bool: true
*/
bool movHandler(robotics_project::MovementHandler::Request &req, robotics_project::MovementHandler::Response &res);
/*!
    @brief Function to get the path of the robot's movement
    @param[in] brickPosition: position of the brick to be moved
    @param[in] brickOrientation: orientation of the brick to be moved
    @param[in] blockId: id string of the block to be moved
    @return Path: path matrix containing the joints configurations of the movement
*/
Path getPath(const Eigen::Vector3d &brickPosition, const Eigen::Quaterniond &brickOrientation,
             const std::string &blockId);
/*!
    @brief Function to fill the response of the service movement_handler
    @param[in] movements: path matrix containing the joints configurations of the movement
    @return robotics_project::MovementHandler::Response: response containing the joints configurations of the movement
*/
robotics_project::MovementHandler::Response getResponse(Path movements);
/*!
    @brief Function to run the optimization of the parameters for the differential kinematics
*/
void runOptimization();

#endif