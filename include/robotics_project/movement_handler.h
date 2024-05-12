#include "robotics_project/robotics_project.h"
#include "robotics_project/MovementHandler.h"

/* Number of moves inside a path */
#define N_MOVEMENTS 5
// This is a dummy data for simulations of a functioning MOVEMENT HANDLER module


bool mov_handler(robotics_project::MovementHandler::Request &req, robotics_project::MovementHandler::Response &res);

/* Interactions with main */
robotics_project::MovementHandler::Response get_response(Path movements);