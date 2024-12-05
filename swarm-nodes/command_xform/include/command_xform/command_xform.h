#ifndef COMMAND_XFORM_H
#define COMMAND_XFORM_H

#include "ros/ros.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

//Class to convert 3d vecotrs to roll, pitch, yaw, and thrust commands

class CommandXform {

    public:
        CommandXform();

        ros::NodeHandle nh;

        ros::Publisher att_pub;

        ros::Subscriber vector_sub;
        void vectorCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
        
        mavros_msgs::AttitudeTarget att_target;
        tf2::Quaternion q;

};

#endif