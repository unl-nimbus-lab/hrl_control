#include <stdio.h>
#include "ros/ros.h"
#include "mavros_msgs/SwarmGPS.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/DebugValue.h"


int myID = 1;
bool avoidanceEnabled = false;
int clusterID = 1;
float myLat = 0.0;
float myLon = 0.0;
float latTol = 0.0005;
float lonTol = 0.0005;

int workingID;
float workingLat;
float workingLon;

float upperLat = 0.0;
float lowerLat = 0.0;

float upperLon = 0.0;
float lowerLon = 0.0;

mavros_msgs::SetMode mode_msg;

ros::Subscriber global_position_sub, debug_vector_sub;
ros::ServiceClient set_mode;

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

void command_cb(const mavros_msgs::DebugValue::ConstPtr& msg) {
    //Pointers to store message info
    int *targetEntity = new int;
    int *command = new int;
    parseCommand(msg->name, targetEntity, command);
    char entity = msg->name[0];  //[S]warm, [C]luster, or [A]gent

    switch(entity) {
        case 'A':
            if (*targetEntity == myID) {
                switch(*command) {
                    case 16:
                        std::cout << "Enabiling Collision Avoidance" << std::endl;
                        avoidanceEnabled = true;
                        break;
                    case 17:
                        std::cout << "Disabling Collision Avoidance" << std::endl;
                        avoidanceEnabled = false;
                        break;
                }
            }
        case 'C':
            if (*targetEntity == clusterID) {
                switch(*command) {
                    case 16:
                        std::cout << "Enabiling Collision Avoidance" << std::endl;
                        avoidanceEnabled = true;
                        break;
                    case 17:
                        std::cout << "Disabling Collision Avoidance" << std::endl;
                        avoidanceEnabled = false;
                        break;
                }
            }
        break;
    }
}


void GPS_cb(const mavros_msgs::SwarmGPS::ConstPtr& msg) {
    workingID = msg->system_id;
    //If the message is from the host, update the host
    if(workingID == myID) {
        myLat = msg->latitude;
        myLon = msg->longitude;

        upperLat = myLat + latTol;
        lowerLat = myLat - latTol;

        upperLon = myLon + lonTol;
        lowerLon = myLon - lonTol;
    }
    //If the message is not from the host, figure out if it is in the deadzone
    else {
        workingLat = msg->latitude;
        workingLon = msg->longitude;

        if(workingLat < upperLat && workingLat > lowerLat) {
            if (workingLon < upperLon && workingLon > lowerLon) {
                //This vehicle is in the airspace, altert the user
                if (avoidanceEnabled) {
                    std::cout << "Agent_" << workingID << " is in my airspace, Calling Brake" << std::endl;
                    mode_msg.request.custom_mode = "BRAKE";
                    set_mode.call(mode_msg);
                }
            }
        }
    }
}

int main(int argc, char **argv) {

    myID = std::stod(argv[1]);
    clusterID = std::stod(argv[2]);

    ros::init(argc, argv, "observer");
    ros::NodeHandle nh;
    global_position_sub = nh.subscribe("/mavros/global_position/swarm", 1, GPS_cb);
    debug_vector_sub = nh.subscribe("/mavros/debug_value/debug_vector", 5, command_cb);
    set_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    ros::spin();

}

