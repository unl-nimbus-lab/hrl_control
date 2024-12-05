#include <iostream>
#include "ros/ros.h"
#include <Eigen/Dense>
#include "geometry_msgs/AccelStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "mavros_msgs/SetMode.h"
#include <custom_libs/geo_converter.hpp>
#include "geometry_msgs/PointStamped.h"

//Ros node that outpus a xyz control vector at 20Hz

double x_target = 30.0;
double y_target = -10.0;
double z_target = 400.0; //MSL altitude

double thrust_lim = 0.1;
double thrust_lim_z = 5.0;

double out_mod = 0.0; //this is bad, but it works for now

//Control gains, no touch
double kp = 0.80;
double ki = 0.24;
double kd = 0.375;

double error_z = 0.0;
double error_z_sum = 0.0;
double dz_error = 0.0;
double dt = 0.04;
double z_error_prev = 0.0;
double output_raw_z = 0.0;

int control_mode = 0;

int rate = 25;

//For the XY plane
Eigen::MatrixXd x(4,1); 
Eigen::MatrixXd k(2,4); 
Eigen::MatrixXd zero_mat(2,2);


//For the Z dimension
Eigen::MatrixXd x_z(3,1); //For the Z dimension
Eigen::MatrixXd k_z(1,3);

//The aug_states
Eigen::MatrixXd aug_state(4,1);
Eigen::MatrixXd aug_state_z(2,1);

ros::ServiceClient set_mode;
mavros_msgs::SetMode guided_msg;

void stateCallback(const geometry_msgs::AccelStamped::ConstPtr& msg) {
    x(0,0) = msg->accel.linear.x;
    x(1,0) = msg->accel.linear.y;
    x(2,0) = msg->accel.angular.x;
    x(3,0) = msg->accel.angular.y;

    x_z(0,0) = msg->accel.linear.z;
    x_z(1,0) = msg->accel.angular.z;
}

void augStateCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    aug_state(0,0) = msg->point.x;
    aug_state(1,0) = msg->point.y;
    set_mode.call(guided_msg);
}


int main(int argc, char **argv){ 

    ros::init(argc, argv, "lqr_control");
    ros::NodeHandle nh;
    
    //Initialize the state and gain matricies
    x << 0.0, 0.0, 0.0, 0.0;
    k << 0.9879, 0.0000, 1.814, 0.0000,
        0.0000, 0.9879, 0.0000, 1.8140;
    zero_mat << 0.0, 0.0, 0.0, 0.0;

    x_z << 0.0, 0.0, 0.0;
    k_z << 2.8613, 3.3252, 0.8608;

    aug_state << x_target, y_target, 0.0, 0.0;
    aug_state_z << z_target, 0.0;

    ros::Publisher control_pub = nh.advertise<geometry_msgs::Vector3Stamped>("vector", 2);

    ros::Subscriber state_sub = nh.subscribe("/agent_state", 2, stateCallback);
    ros::Subscriber aug_state_sub = nh.subscribe("/agent_1/aug_state", 2, augStateCallback);

    set_mode = nh.serviceClient<mavros_msgs::SetMode>("/agent_1/mavros/set_mode");

    geometry_msgs::Vector3Stamped control_msg;

    mavros_msgs::SetMode brake_msg;
    brake_msg.request.custom_mode = "BRAKE";

    
    guided_msg.request.custom_mode = "GUIDED";

    ros::Rate loop_rate(rate);

    double fx = 0;
    double fy = 0;
    double fz = 0;

    while(ros::ok()) {
        
        //PID control stuff
        error_z = z_target - x_z(0,0);
        
        //Set the control mode based of the error, if there is too much altitude error, use a P controller with no fx, fy
        if ( abs(error_z ) > 3.0) {
            control_mode = 0;
        } else {
            control_mode = 1;
        } 

        //0.5 marginally stable
        //0.1P, 0.01I works close to target ~37m

        auto output_raw = -1.0 * k * (x - aug_state);
        auto error = x - aug_state;


        switch (control_mode) {
            case 0:
                //If the error is too large, use a P controller for Z and no lateral effort
                out_mod = 0.0;
                output_raw_z = kp*error_z;
                error_z_sum = 0.0; //reset the integral
                ROS_WARN("Altitude error too large, Adjusting height only");
                break;
            case 1:
                //If the error is small, use PID control for Z and LQR for lateral control
                out_mod = 1.0;
                error_z_sum += error_z * dt;
                dz_error = (error_z - z_error_prev) / dt;
                z_error_prev = error_z;
                output_raw_z = kp*error_z + ki*error_z_sum + kd*dz_error;
                break;
        }
        
        //ROS_INFO("Output: %f, %f, %f zerror: %f, xy_error: %f", output_raw(0,0), output_raw(1,0), output_raw_z, error_z_sum, error.norm());

        fx = std::max( output_raw(0,0), -1 * thrust_lim);
        fx = std::min( fx, thrust_lim);

        fy = std::max( output_raw(1,0), -1 * thrust_lim);
        fy = std::min( fy, thrust_lim);

        fz = std::max( output_raw_z, 0.15 );
        fz = std::min( fz, thrust_lim_z);
        
        control_msg.vector.x = out_mod*fx;
        control_msg.vector.y = out_mod*fy;


        if (error.norm() < 5.0) {
            ROS_WARN("Close to target, stopping");
            set_mode.call(brake_msg);
        }

        control_msg.vector.z = fz;
        control_pub.publish(control_msg);
        ros::spinOnce();
        loop_rate.sleep();

    }

    
    
    return 0;
}