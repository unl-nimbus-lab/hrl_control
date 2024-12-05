// This node does two three things
// 1. Receives agent info from mavlink node
// 2. Estimate the correct info
// 3. Publishes the info to the hrl controller node

#include <ros/ros.h>
#include <thread>
#include <string>
#include <mutex>
#include <chrono>
#include "std_msgs/String.h"
#include <math.h>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <custom_libs/geo_converter.hpp>

#include <swarm_msgs/state.h>


#define ROS_NODE_NAME "estimator"
#define PUBLISH_RATE 8 // TODO : change 8Hz to the desired publish rate

#define NN 2 // TODO: change this to a parameter to be passed from a parameter file

void mavlinkCallback(swarm_msgs::state new_state){
	// This callback receives other agent info info from the python node that listens to mavlink directly


}

void publisherToHRL(){



}


int main( int argc, char** argv )
{
        ros::init(argc, argv, ROS_NODE_NAME); //Initialize the node


        ros::NodeHandle eh;
	// TODO replace the topic_name with the correct topic name from the python node
        ros::Subscriber mavlink_callback = eh.subscribe("topic_name", 1, mavlinkCallback);


        // Path to the json file
        std::string path_to_json = "state-matrix.json";

        //Read the json file
//        Sys ss = readJSON(path_to_json);


        // Create seperate threads for processes with specific Hz requirements
        std::thread publish_to_hrl (publisherToHRL); //Sperate thread for the publisher


        // join the threads at the end
        publish_to_hrl.join() ;

        return 0;
}
