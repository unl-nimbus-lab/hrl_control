//This node reads the agent's gps location and converts it to a local XY frame
//centered at a pre-determined orgin (RMF for instance)

#include "ros/ros.h"
#include "swarm_comms/allAgentsPosStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "mavros_msgs/SwarmGPS.h"
#include "custom_libs/geo_converter.hpp"
#include "geometry_msgs/PointStamped.h"
#include <string>
#include <cstring>

/*
Here is the frame we are using:

NORTH (X)
^
|
|
|
|
|
|
|
|
|
|
|---------------------------------> EAST (Y)

*/

void position_update_callback(const mavros_msgs::SwarmGPS::ConstPtr& msg);
double sign(double input);

ros::Publisher agent_location_pub;
ros::Subscriber global_position_sub;

//global declared variables
double agent_global_x;
double agent_global_y;
geo_converter convObject;
geometry_msgs::PointStamped agent_pos;
int SYSID_THISMAV;
std::string string_id;

//global initialized variables, default to RMF
double ORIGIN_LAT = 40.8467784;
double ORIGIN_LON = -96.4719180;

int main(int argc, char **argv) {

    ORIGIN_LAT = std::stod(argv[2]);
    ORIGIN_LON = std::stod(argv[3]);

    SYSID_THISMAV = atoi(argv[1]);
    std::cout << "my sysid is: " << SYSID_THISMAV << std::endl;
    
    ros::init(argc, argv, "state_conditioning");
    ros::NodeHandle nh;
    global_position_sub = nh.subscribe("/mavros/global_position/swarm",1, position_update_callback);
    agent_location_pub = nh.advertise<geometry_msgs::PointStamped>("/agent_position_xy",1,true);

    ros::spin();
}

void position_update_callback(const mavros_msgs::SwarmGPS::ConstPtr& msg) {
    double latitude = msg->latitude;
    double longitude = msg->longitude;
    string_id = std::to_string(msg->system_id);
    
    //Assumption: we are in the northern hemisphere, this will affect the sign of the local frame
    double x_sign = sign(latitude - ORIGIN_LAT);
    double y_sign = sign(longitude - ORIGIN_LON);

    //Calculate the x position, hold the longitude constant
    agent_global_x = convObject.gps_to_meters(ORIGIN_LAT,ORIGIN_LON,latitude,ORIGIN_LON);
    agent_global_x = x_sign * agent_global_x;
    
    //Calculate the y position, hold the latitude constant
    agent_global_y = convObject.gps_to_meters(ORIGIN_LAT,ORIGIN_LON,ORIGIN_LAT,longitude);
    agent_global_y = y_sign * agent_global_y;

    agent_pos.point.x = agent_global_x;
    agent_pos.point.y = agent_global_y;
    agent_pos.point.z = msg->altitude;
    agent_pos.header.stamp = ros::Time::now();
    agent_pos.header.frame_id = string_id;

    agent_location_pub.publish(agent_pos);
}

double sign(double input) {
    //Deterine the sign of a double
    if (input < 0) {
        return -1.0;
    } else {
        return 1.0;
    }
}