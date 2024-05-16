#include "robotics_project/robotics_project.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "ros/ros.h"

Eigen::Matrix4d generalTransformationMatrix(double theta, double alpha, double d, double a){
    return Eigen::Matrix4d {
        {cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)},
        {sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)},
        {0, sin(alpha), cos(alpha), d},
        {0, 0, 0, 1}
    };
}

std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix4d> directKinematics(Eigen::Matrix<double, 6, 1> joints){
    Eigen::Matrix4d transformation_matrix = generalTransformationMatrix(joints(0), alpha_DH(0), d_DH(0), a_DH(0));
    for(int i = 1; i < 6; i++){
        transformation_matrix *= generalTransformationMatrix(joints(i), alpha_DH(i), d_DH(i), a_DH(i));
    }
    
    // pe: position end effector -> position wrist3_links
    Eigen::Vector3d pe(transformation_matrix.block(0,3,3,1));
    Eigen::Matrix3d Re(transformation_matrix.block(0,0,3,3));
    return std::make_tuple(pe, Re, transformation_matrix);
}   

Eigen::Matrix<double, 8, 1> getJointConfiguration(){
    // index from 0 to 7
    // Shoulder pan joint           index:4
    // Shoulder lift joint          index:3
    // Elbow joint                  index:0
    // Wrist1 joint                 index:5
    // Wrist2 joint                 index:6
    // Wrist3 joint                 index:7
    // Hand1 joint                  index:1
    // Hand2 joint                  index:2
    boost::shared_ptr<sensor_msgs::JointState const> joint_configuration = ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states");
    return  Eigen::Matrix<double, 8, 1>{joint_configuration->position[4],joint_configuration->position[3],joint_configuration->position[0],joint_configuration->position[5],
                                        joint_configuration->position[6], joint_configuration->position[7], joint_configuration->position[1], joint_configuration->position[2]};
}