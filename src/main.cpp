/* Header */
#include <iostream>
#include "robotics_project/main.h"
#include "ros/ros.h"


int main(int argc, char **argv) {

    /* Initialize ROS node's name */
    ros::init(argc, argv, "robotics_project_main");
    std::cout << "MAIN Process: STARTED" << std::endl;


    ros::NodeHandle n;
    ros::ServiceClient service_client = n.serviceClient<robotics_project::MovementHandler>("movement_handler");
    robotics_project::MovementHandler srv;


    ros::NodeHandle node_handler;
    ros::Publisher pub = node_handler.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10);
    Path movements;

    // for-each block, move
    for (int i=0; i<N_BLOCKS; i++) {

        /* Import the required movements from the module */
        if (service_client.call(srv)) {
            
            movements = get_movements(srv.response);

            std::cout << "MAIN Process: Transmissitting movements | block " << i << std::endl;
            move(pub, movements);
        }
        else {
            std::cerr << "MAIN Process: Failed to call the MOVEMENT HANDLER module | block " << i << std::endl;
            continue;
        }

    }

    std::cout << "MAIN Process: ENDED" << std::endl;
    return 0;
}

void move(ros::Publisher &pub, Path path) {
    for (int i=0; i<path.rows(); i++)
        move_row(pub, path.row(i));
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
    for (int i=0; i<movements.rows(); i++)
        for (int j=0; j<movements.cols(); j++)
            movements.row(i).col(j) << res.path.movements[i].data[j];

    return movements;
}