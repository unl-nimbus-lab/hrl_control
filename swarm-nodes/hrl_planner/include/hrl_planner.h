#ifndef HRL_PLANNER_H
#define HRL_PLANNER_H

#define ENABLE_PUBLISH 1
#define DISABLE_PUBLISH 2

#include "ros/ros.h"
#include <iostream>
#include <Eigen/Dense>

//ROS message includes
#include "geographic_msgs/GeoPoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "swarm_comms/allAgentsPosStamped.h"
#include "custom_libs/geo_converter.hpp"
#include "mavros_msgs/DebugValue.h"
#include "geometry_msgs/Inertia.h"


Eigen::MatrixXd A(6,6), B(6,2), K(2,6), X(6,1), T_X(6,1), U(2,1), I(6,1), DX(6,1);

double movement_threshold = 3.0;
double max_input = 100.0;
geometry_msgs::Inertia state_message;

void parseCommand(std::string message, int *cluster, int *command) {
    //Process a command of the form:
    //CXXX$XXX Where the "XXX" after the "C" identifies the cluster and
    //the "XXX" after the "$" identifies the command

     std::string delimiter = "$";
     // remove 'C', 'S', or 'A'
     std::string short_message = message.substr(1);
     // now find the '$'
     int split_idx = short_message.find(delimiter);
     // check for error and std::string::npos
     std::string cluster_str = short_message.substr(0, split_idx);
     // remove cluster and delimiter
     short_message.erase(0, split_idx+delimiter.length());
     std::string command_str = short_message;
     // convert the command to an int
     *cluster = atoi(cluster_str.c_str());
     *command = atoi(command_str.c_str());
}

double input_saturtion_check (double input, double stauration_value_threshold) {
    if ( abs(input) > stauration_value_threshold ) {
        //the requested input is too high
        double sign = input/input;
        return sign * stauration_value_threshold;

    } else {
        return input;
    }
}

#endif