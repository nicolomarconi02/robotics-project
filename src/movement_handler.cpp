#include "ros/ros.h"
#include "robotics_project/robotics_project.h"
#include "robotics_project/MovementHandler.h"
#include "robotics_project/movement_handler.h"
#include <eigen3/Eigen/Dense>


Path get_path();

int main(int argc, char **argv) {

    /* Initialize ROS node's name */
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
    Path p;
    p.conservativeResize(N_MOVEMENTS, p.cols());

    for (int i = 4; i < N_MOVEMENTS; i++) {
        double dummy = (i+1) * 10;
        for (int j=0; j<8; j++) p.row(i).col(j) << dummy;
    }

    return p;
}


robotics_project::MovementHandler::Response get_response(Path movements) {
    robotics_project::MovementHandler::Response res;
    res.path.movements.resize(5);

    // Assigning, in a safe manner, each row of the Path to the response's counterpart
    for (int i=0; i<movements.rows(); i++)
        // For clarification: res.path.movements[i] = movements.row(i)
        res.path.movements[i].data.assign(movements.row(i).begin(), movements.row(i).end());

    return res;
}