#ifndef HRL_COMMAND_INTERPRET_H
#define HRL_COMMAND_INTERPRET_H

#include "ros/ros.h"
#include "mavros_msgs/DebugValue.h"
#include "custom_libs/geo_converter.hpp"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Bool.h"

class HRLCommandInterpret {
    public:
        HRLCommandInterpret();

        ros::NodeHandle nh;

        ros::Publisher aug_state_pub;
        ros::Publisher publish_state_pub;

        ros::Subscriber debug_vector_sub;
        void debugVecCallback(const mavros_msgs::DebugValue::ConstPtr& msg);

        ros::Subscriber debug_int_sub;
        void debugIntCallback(const mavros_msgs::DebugValue::ConstPtr& msg);

        void updateAugState(float x_target, float y_target, int cluster_id);

        geo_converter converter;

        struct {
            double lat;
            double lon;
        } global_origin;

        int* msg_tgt_cluster = new int;
        int* msg_command = new int;

        //int SYSID_THISMAV = 1;
        //int CLUSTER_ID = 1;

        geometry_msgs::PointStamped aug_state;

        double lat = 0.0;
        double lon = 0.0;
        double x_target = 0.0;
        double y_target = 0.0;
        double x_sign = 0.0;
        double y_sign = 0.0;

};

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

double sign(double input) {
    //Deterine the sign of a double
    if (input < 0) {
        return -1.0;
    } else {
        return 1.0;
    }
}

#endif