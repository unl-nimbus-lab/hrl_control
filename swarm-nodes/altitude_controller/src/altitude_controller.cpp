#include <iostream>
#include <Eigen/Dense>
#include "ros/ros.h"
#include "geometry_msgs/AccelStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

//Fast and dirty ROS node that outputs a desired force in the z direction at 20Hz

float z_target = 400.0; //MSL altitude

//Initialize the state and gain matricies
Eigen::MatrixXd x_z(2,1);
Eigen::MatrixXd k_z(1,2);
Eigen::MatrixXd aug_state(2,1);
double F_z = 0.0;

void stateCallback(const geometry_msgs::AccelStamped::ConstPtr& msg) {
    // Update the "z" portion of the state when a agent state is available
    x_z(0,0) = msg->accel.linear.z;
    x_z(1,0) = msg->accel.angular.z;
}


int main(int argc, char **argv) {
    // Main node

    ros::init(argc, argv, "altitude_controller");
    ros::NodeHandle nh;
    
    ros::Publisher zForce_pub = nh.advertise<geometry_msgs::Vector3Stamped>("vector", 2);
    ros::Subscriber state_sub = nh.subscribe("/agent_state", 2, stateCallback);

    // Initilie starting state and constants
    x_z << 0.0, 0.0;
    k_z << 20.0, 18.0;
    aug_state << z_target, 0.0;

    ros::Rate loop_rate(20);

    while (ros::ok()) {

        // Compute "raw" desired force
        auto output_raw = -1.0  * k_z * (x_z - aug_state);
        F_z = output_raw(0,0);

        // bound the output between 0 and 20 N
        F_z = std::max(0.0, F_z);
        F_z = std::min(20.0, F_z);

        // Pack the F_z into a message and publish
        geometry_msgs::Vector3Stamped fz_msg;
        fz_msg.vector.z = F_z;
        //std::cout << "F_z: " << typeid(F_z).name() << std::endl;
        zForce_pub.publish(fz_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}