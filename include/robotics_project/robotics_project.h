#ifndef ROBOTICS_PROJECT_H
#define ROBOTICS_PROJECT_H

#include <cstdlib>

#include "Eigen/Dense"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#include "robotics_project/MovementHandler.h"

/* Typedefs in order to be more clear */
typedef Eigen::Matrix<double, Eigen::Dynamic, 8> Path;  // same as MatrixX8d
typedef Eigen::Matrix<double, 1, 8> PathRow;            // same as RowVector8d
    // docs reference: https://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html

#endif