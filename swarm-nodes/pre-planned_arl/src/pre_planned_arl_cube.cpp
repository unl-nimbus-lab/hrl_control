#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mavros_msgs/DebugValue.h"
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "custom_libs/geo_converter.hpp"
#include "geographic_msgs/GeoPoseStamped.h"

//Debug commands
#define ENABLE_PUBLISH 1
#define DISABLE_PUBLISH 2
#define BEGIN_TRAJECTORY_FOLLOW 3
#define RESET_TRAJECTORY_FOLLOW 4

//Planner States
#define PRE_FOLLOW 1
#define FOLLOWING 2


bool initialPositionFlag = true;
bool finalPositionFlag = true;
ros::Time init_time;
ros::Duration length;
float sysTime;
int columnIndex = 0;
int numberOfColumns = 1001;
int numberOfRows = 2;

std::string file_var;

int *cluster = new int;
int *command = new int;
bool publishFlag = false;
geographic_msgs::GeoPoseStamped pos_msg;
mavros_msgs::DebugValue hrl_command;
int mode = 1;
double heightFixed;
geo_converter convObject;
double GLOBAL_ORIGIN_LAT;
double GLOBAL_ORIGIN_LON;


// void stateBroadcast_cb(const mavros_msgs::DebugValue::ConstPtr& msg) {
//     //update the state
//     modeBroadcast = *msg;
//     mode = modeBroadcast.value_int;
//     //ROS_INFO("[%i",mode);
// }

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

void HRLReference_cb(const mavros_msgs::DebugValue::ConstPtr& msg) {
    //update the reference
    parseCommand(msg->name, cluster, command);
    ROS_WARN("COMMAND PARSED");

    switch(*command) {
        case ENABLE_PUBLISH:
            publishFlag = true;
            break;
        case DISABLE_PUBLISH:
            publishFlag = false;
            break;
        case BEGIN_TRAJECTORY_FOLLOW:
            mode = FOLLOWING;
            break;
        case RESET_TRAJECTORY_FOLLOW:
            mode = PRE_FOLLOW;
            break;
    }
}

int main(int argc, char **argv)
{

    file_var = argv[1];     //Read in the file name as a command line argument
    heightFixed = std::stod(argv[2]) + std::stod(argv[3]);  //Read in the altitude as a command line argument
    GLOBAL_ORIGIN_LAT = std::stod(argv[4]);
    GLOBAL_ORIGIN_LON = std::stod(argv[5]);
    std::cout << "home Lat: " << GLOBAL_ORIGIN_LAT << std::endl;
    std::cout << "home Lon: " << GLOBAL_ORIGIN_LON << std::endl;



    ros::init(argc, argv, "prePlanned_cube");
    ros::NodeHandle nh;


    ////////// BEGIN FILE READ //////////
    using namespace std;
    string line;
    ifstream myfile(file_var);
    float freyjaTrajectory[numberOfRows][numberOfColumns];
    int i = 0;
    int j = 0;
    string temp;

    if (myfile.is_open()) {
        while (myfile && getline(myfile, line)) //split each line from the file into "line"
        {
            stringstream ss(line); // substream takes a string and begins to read it like ifstream does to a file. Creates a stream that we can use
            j = 0;
            while (ss && ss >> temp) { // goes as long as the stream from ss has stuff to read in
                freyjaTrajectory[i][j] = stof(temp);
                j++;
            }
            i++;
        }
        myfile.close();
        ROS_WARN("Trajectory successfuly loaded");
    }

    else {
        ROS_ERROR("Unable to load trajectory");
        return 1;
    }
    ////////// END FILE READ //////////


//Set initial conditions 
float initialX = freyjaTrajectory[1][0];
float initialY = freyjaTrajectory[2][0];
float finalX = freyjaTrajectory[1][1000];
float finalY = freyjaTrajectory[2][1000];
float yaw = 0.0;

std::cout << "initial X: " << initialX << std::endl;
std::cout << "initial Y: " << initialY << std::endl;

ros::Subscriber liveHRLReference_sub = nh.subscribe("/mavros/debug_value/debug_vector", 1, HRLReference_cb);

ros::Publisher wp_pub = nh.advertise<geographic_msgs::GeoPoseStamped> ( "/mavros/setpoint_position/global", 1, true );
while (ros::ok()) {
    switch(mode) {

        case PRE_FOLLOW:
            pos_msg.pose.position.latitude = convObject.meters_to_lat(GLOBAL_ORIGIN_LAT,GLOBAL_ORIGIN_LON, initialX);
            pos_msg.pose.position.longitude = convObject.meters_to_lat(GLOBAL_ORIGIN_LAT,GLOBAL_ORIGIN_LON, initialY);
            pos_msg.pose.position.altitude = heightFixed;
            break;
        case FOLLOWING:
            pos_msg.pose.position.latitude = convObject.meters_to_lat(GLOBAL_ORIGIN_LAT,GLOBAL_ORIGIN_LON, finalX);
            pos_msg.pose.position.longitude = convObject.meters_to_lat(GLOBAL_ORIGIN_LAT,GLOBAL_ORIGIN_LON, finalY);
            pos_msg.pose.position.altitude = heightFixed;
            break;
    }
    if (publishFlag == true) {
        pos_msg.header.stamp = ros::Time::now();
        wp_pub.publish( pos_msg );
    }
    
    ros::Duration(0.5).sleep();
    ros::spinOnce();
}
return 0;
}