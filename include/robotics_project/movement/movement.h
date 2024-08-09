#ifndef MOVEMENT_HANDLER_H
#define MOVEMENT_HANDLER_H

#include "robotics_project/MovementHandler.h"
#include "robotics_project/utils.h"

/* Number of moves inside a path */
#define N_MOVEMENTS 5
// This is a dummy data for simulations of a functioning MOVEMENT HANDLER module

bool movHandler(robotics_project::MovementHandler::Request &req, robotics_project::MovementHandler::Response &res);

Path getPath(const Eigen::Vector3d &brickPosition, const Eigen::Quaterniond &brickOrientation,
             const std::string &blockId);

/* Interactions with main */
robotics_project::MovementHandler::Response getResponse(Path movements);

void runOptimization();

#endif