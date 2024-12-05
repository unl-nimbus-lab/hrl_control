#ifndef CLUSTERING_H
#define CLUSTERING_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mavros_msgs/DebugValue.h"
#include "mavros_msgs/SwarmGPS.h"
#include "mavros_msgs/CommandLong.h"
#include "mavros_msgs/SetMode.h"
#include "geographic_msgs/GeoPoseStamped.h"
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <eigen3/Eigen/Dense>
#include "custom_libs/geo_converter.hpp"
#include "geometry_msgs/PointStamped.h"
#include "custom_libs/json_reader.hpp"
#include "swarm_comms/allAgentsPosStamped.h"
#include "swarm_comms/agentPos.h"
#include "std_msgs/Float64MultiArray.h"

//Universal constants:
#define PI 3.14159265

//MAVROS message types:
#define NAMED_VALUE_VECT 1
#define NAMED_VALUE_FLOAT 3
#define NAMED_VALUE_INT 4

//HRL Flightmodes
#define CLUSTER_GUIDED_LOCAL_MODE 1
#define CLUSTER_RALLY_MODE 2
#define CLUSTER_GUIDED_GLOBAL_MODE 3
#define ALTITUDE_SEP_MODE 4
#define FOLLOW_THE_LEADER_MODE 5
#define FORCE_COLLISION 6
#define AGENT_GUIDED_LOCAL 7
#define AGENT_GUIDED_GLOBAL 8
#define HRL_NAVIGATE 9

//Cluster Formation types
#define CIRCLE_FORMATION 1
#define LINEAR_FORMATION 2

//Cluster Commands
#define PING 0
#define START_CLUSTER_GUIDED 1
#define STOP_CLUSTER_GUIDED 2
#define CHANGE_FLIGHT_MODE 3
#define SET_CLUSTER_LOCAL_POSITION 4
#define SET_CLUSTER_RADIUS 5
#define SET_CLUSTER_TYPE 6
#define SET_HOME_POSITION 7
#define SET_LOCAL_RALLY 8
#define SET_CLUSTER_GLOBAL_POSITION 9
#define SET_FTL_SYSID 10
#define ENABLE_CIRCLE 11
#define DISABLE_CIRCLE 12
#define SET_CIRCLE_RATE 13
#define SET_CLUSTER_OFFSETZ 14
#define SET_BRAKE 15
#define UPDATE_TARGET_REL 16
#define UPDATE_TARGET_GBL 17

//Agent Commands

#define CHANGE_CLUSTER_ID 2
#define CHANGE_SYSID 3
#define CHANGE_CLUSTER_ORDER 4
#define CHANGE_CLUSTER_SIZE 5
#define CHANGE_AGENT_ALTITUDE 6
#define SET_AGENT_LOCAL_TARGET 7
#define SET_AGENT_GLOBAL_TAGET 8

// Agent restraints
#define MAX_ALT 121.92 // 400 feet (converted to meters)

using json = nlohmann::json;
using namespace std;
using namespace Eigen;

//Some objects that are used:
ros::Publisher ak_pub;
ros::Publisher wp_pub;
ros::Publisher debug_pub;
ros::ServiceClient client;
ros::ServiceClient set_mode;
mavros_msgs::DebugValue modeBroadcast;
mavros_msgs::DebugValue agentAcknowledgement;
mavros_msgs::DebugValue bootAcknowledgeMsg;
mavros_msgs::CommandLong alt_sep;
mavros_msgs::SetMode mode_msg;
geo_converter convObject;
int tick_count = 0;
float hrl_trajectory[2][1001];
double distanceThreshold = 0.0;
double GLOBAL_POSITION_X;
double GLOBAL_POSITION_Y;

// //New globals for HRL planner
MatrixXd hrl_state(100, 1);
MatrixXd raw_hrl_state(100,1);
MatrixXd targets_rel; //<< TODO: Un-hardcode this one (2 clusters)
MatrixXd aug_state;
MatrixXd index_encodings;
int numberOfHRLStates;

double hrl_sampling_time = 0.2; //s
double convergent_dist = 0.1; //m
double waypoint_dist_req = 10.0; // meters
double final_dist_req = 4.0; // meters

std_msgs::Float64MultiArray debugMsg;

//Function prototypes:
void parseCommand(std::string message, int *cluster, int *command);
void handleCommand(int *cluster, int *command);
void HRLReference_cb(const mavros_msgs::DebugValue::ConstPtr& msg);
void brake_cb(const mavros_msgs::DebugValue::ConstPtr& msg);
void GPS_cb(const mavros_msgs::SwarmGPS::ConstPtr& msg);
void acknowledge(int agentID,int* command);
void parseCommandUnitTest();
int updateTrajectoryTarget(int previousIndex);
void stateUpdate_cb(const swarm_comms::allAgentsPosStamped::ConstPtr& msg);
void hrlStateUpdate(MatrixXd swarm_comp, MatrixXd index_encodings, MatrixXd raw_state);
int getHRLIndex(int sysID, MatrixXd swarm_comp, MatrixXd index_encodings);
int getHRLCluster(int sysID, MatrixXd swarm_comp);
MatrixXd buildAugState(MatrixXd targets_rel, MatrixXd index_encodings, int numberOfHRLStates);
void updateTargetRel(MatrixXd &TargetsRel, int newX, int newY, int myCluster);
void updateTargetGbl(MatrixXd &TargetsRel, int newLat, int newLon, int myCluster, double ORIGIN_LAT, double ORIGIN_LON);
double sign(double input);
MatrixXd composeSwarm(MatrixXd index_encodings);
MatrixXd generateDefaultTargetsRel(int numberOfClusters);


//agent class:
class agent {
    public:
        void setSysid(double id);
        void setClusterid(double id);
        void setClusterOrder(double order);
        void setOriginalClusterOrder(double order);
        void setClusterSize(double size);
        void setClusterRadius(double radius);
        void setAgentAlt(double alt);
        void setHome(double lat, double lon, double alt);
        void setRally1(double x, double y);
        void setRally2(double x, double y);
        double getRally1X();
        double getRally1Y();
        double getRally1Z();
        double getRally2X();
        double getRally2Y();
        double getRally2Z();
        double getAgentAlt();
        void setFlightMode(int mode);
        void setClusterOffsets();
        double getClusterXOffset();
        double getClusterYOffset();
        void setClusterOffsets(int order);
        void setPositionTarget();
        void setClusterLocalTarget(double x, double y, double z);
        void setClusterType(int type);
        double getSysid();
        double getClusterid();
        double circleXOffset(int order);
        double circleYOffset(int order);
        double calculateXLineOffset();
        double calculateYLineOffset();
        int getFlightMode();
        double latOffset();
        double lonOffset();
        void setAgentLocalTarget(double x, double y, double z);
        double getAgentLocalTargetX();
        double getAgentLocalTargetY();
        double getAgentLocalTargetZ();
        void setPublish(bool logic);
        bool getPublish();
        void setClusterGlobalTarget(double lat, double lon, double alt);
        void setCurrentPos(double lat, double lon, double alt);
        void setFTLSysid(int id);
        int getFTLSysid();
        void setFTLloc(double lat, double lon, double alt);
        void enableCircle();
        void disableCircle();
        double getOriginalClusterOrder();
        double getClusterOrder();
        double getClusterSize();
        int getCircleTicks();
        void setCircleTicks(int ticks);
        void setClusterOffsetZ (double altitude);
        void setAgentGlobalTarget(double lat, double lon, double alt);
        double getAgentGlobalLat();
        double getAgentGlobalLon();
        geographic_msgs::GeoPoseStamped pos_msg;
        bool FTL_ENABLE = false;    //O-k to begin ftl
        bool CIRCLE_FLAG = false;   //Enables/Disables circling


    private:
        double SYSID;               //Unique ID of this agent within the swarm
        double CLUSTERID;           //ID of the cluster this agent belongs to 
        int HRL_FLIGHTMODE = 1;     //HRL "flight mode"
        double CLUSTER_ORDER;       //This agent's "spot" in the cluster
        double ORIGINAL_CLUSTER_ORDER; //Set this only once and never change
        double CLUSTER_SIZE;        //How many agents are in the cluster
        int CLUSTER_TYPE = 1;       //Organization of agents within the cluster 1) Circle 2) Line
        double CLUSTER_RADIUS;      //Dimension of the cluster
        double AGENT_ALT;           //Height (in meters) above "cluster target"
        bool PUBLISH = false;       //O-k to begin publishing messages to the FCU
        int FTL_SYSID = 0;          //FOLLOW THE LEADER TARGET SYSID
        int CIRCLE_TICKS = 10;      //Number of ros ticks to wait before moving to next position
        struct {
            double X;
            double Y;
            double Z;
        } CLUSTER_OFFSETS, AGENT_LOCAL_TARGET, CLUSTER_LOCAL_TARGET, CLUSTER_RALLY1, CLUSTER_RALLY2;
        //Cluster Offsets: How far (in meters this agent should be offset from the cluster center)
        //Agent Local Target: agent position in the local 'meters' frame
        //Cluster Local Target: Location (in meters) the center of the cluster should be relative to "GLOBAL ORIGIN"
        struct {
            double LAT;
            double LON;
            double ALT;
        } AGENT_RALLY, GLOBAL_ORIGIN, GPS_OFFSETS, CLUSTER_GLOBAL_TARGET, CURRENT_POS, FTL_LOC, AGENT_GLOBAL_TARGET;
        //Agent Rally: Not used
        //Cluster Rally: Locally saved point, unique to each cluster
        //Global Origin: Common world frame for the swarm
        //CLUSTER_GLOBAL_TARGET: Target lat/lon/alt for the cluster
        //CURRENT_POS: CURRENT LAT/LON OF THIS AGENT
        //FTL_LOC: CURRENT LAT/LON OF THE FTL TARGET
};

void agent::setPositionTarget() {
    //This function is used to update the global target of the agent (pos_msg) 
    //based on the hrl flightmode, this is the only function that should
    //directly manipulate the pos_msg.
    double tempx, tempy, tempz;

    switch (HRL_FLIGHTMODE) {
        case CLUSTER_GUIDED_LOCAL_MODE: //Move the cluster in the local frame
            //Update "where the agent should be in the local 'meters' frame"
            tempx = CLUSTER_LOCAL_TARGET.X + CLUSTER_OFFSETS.X;
            tempy = CLUSTER_LOCAL_TARGET.Y + CLUSTER_OFFSETS.Y;
            tempz = CLUSTER_LOCAL_TARGET.Z + CLUSTER_OFFSETS.Z;
            setAgentLocalTarget(tempx,tempy,tempz);

            //Update the pos_msg based on agent local target and global origin
            pos_msg.pose.position.latitude = convObject.meters_to_lat(GLOBAL_ORIGIN.LAT,GLOBAL_ORIGIN.LON,AGENT_LOCAL_TARGET.X);
            pos_msg.pose.position.longitude = convObject.meters_to_lon(GLOBAL_ORIGIN.LAT,GLOBAL_ORIGIN.LON,AGENT_LOCAL_TARGET.Y);
            pos_msg.pose.position.altitude = GLOBAL_ORIGIN.ALT + AGENT_LOCAL_TARGET.Z;
            break;
        case CLUSTER_RALLY_MODE: //Move the cluster in the local frame
            //Update "where the agent should be in the local frame"
            tempx = CLUSTER_RALLY1.X + CLUSTER_OFFSETS.X;
            tempy = CLUSTER_RALLY1.Y + CLUSTER_OFFSETS.Y;
            tempz = CLUSTER_RALLY1.Z + CLUSTER_OFFSETS.Z;
            setAgentLocalTarget(tempx,tempy,tempz);

            //Update the pos_msg based on agent local target and global origin
            pos_msg.pose.position.latitude = convObject.meters_to_lat(GLOBAL_ORIGIN.LAT,GLOBAL_ORIGIN.LON,AGENT_LOCAL_TARGET.X);
            pos_msg.pose.position.longitude = convObject.meters_to_lon(GLOBAL_ORIGIN.LAT,GLOBAL_ORIGIN.LON,AGENT_LOCAL_TARGET.Y);
            pos_msg.pose.position.altitude = GLOBAL_ORIGIN.ALT + AGENT_LOCAL_TARGET.Z;
            break;
        case CLUSTER_GUIDED_GLOBAL_MODE:
            //Update "where the agent should be in the global frame"
            pos_msg.pose.position.latitude = convObject.meters_to_lat(CLUSTER_GLOBAL_TARGET.LAT, CLUSTER_GLOBAL_TARGET.LON, CLUSTER_OFFSETS.X);
            pos_msg.pose.position.longitude = convObject.meters_to_lon(CLUSTER_GLOBAL_TARGET.LAT, CLUSTER_GLOBAL_TARGET.LON, CLUSTER_OFFSETS.Y);
            pos_msg.pose.position.altitude = GLOBAL_ORIGIN.ALT + AGENT_LOCAL_TARGET.Z;
            break;
        case ALTITUDE_SEP_MODE:
            //Move the agent to its altitude target at its current location
            pos_msg.pose.position.latitude = CURRENT_POS.LAT;
            pos_msg.pose.position.longitude = CURRENT_POS.LON;
            pos_msg.pose.position.altitude = GLOBAL_ORIGIN.ALT + AGENT_LOCAL_TARGET.Z;
            break;
        case FOLLOW_THE_LEADER_MODE:
            //Follow the specified agent in cluster formation
            pos_msg.pose.position.latitude = convObject.meters_to_lat(FTL_LOC.LAT, FTL_LOC.LON, CLUSTER_OFFSETS.X);
            pos_msg.pose.position.longitude = convObject.meters_to_lon(FTL_LOC.LAT, FTL_LOC.LON, CLUSTER_OFFSETS.Y);
            pos_msg.pose.position.altitude = GLOBAL_ORIGIN.ALT + AGENT_LOCAL_TARGET.Z;
            break;
        case FORCE_COLLISION:
            pos_msg.pose.position.latitude = GLOBAL_ORIGIN.LAT;
            pos_msg.pose.position.longitude = GLOBAL_ORIGIN.LON;
            pos_msg.pose.position.altitude = 425;
            break;
        case AGENT_GUIDED_LOCAL:
            // tempx = CLUSTER_LOCAL_TARGET.X + CLUSTER_OFFSETS.X;
            // tempy = CLUSTER_LOCAL_TARGET.Y + CLUSTER_OFFSETS.Y;
            // tempz = CLUSTER_LOCAL_TARGET.Z + CLUSTER_OFFSETS.Z;
            // setAgentLocalTarget(tempx,tempy,tempz);
            //Update the pos_msg based on agent local target and global origin
            pos_msg.pose.position.latitude = convObject.meters_to_lat(GLOBAL_ORIGIN.LAT,GLOBAL_ORIGIN.LON,AGENT_LOCAL_TARGET.X);
            pos_msg.pose.position.longitude = convObject.meters_to_lon(GLOBAL_ORIGIN.LAT,GLOBAL_ORIGIN.LON,AGENT_LOCAL_TARGET.Y);
            pos_msg.pose.position.altitude = GLOBAL_ORIGIN.ALT + AGENT_LOCAL_TARGET.Z;
            break;
        case AGENT_GUIDED_GLOBAL:
            tempz = CLUSTER_LOCAL_TARGET.Z + CLUSTER_OFFSETS.Z;
            setAgentLocalTarget(0.0,0.0,tempz);
            pos_msg.pose.position.latitude = AGENT_GLOBAL_TARGET.LAT;
            pos_msg.pose.position.longitude = AGENT_GLOBAL_TARGET.LON;
            pos_msg.pose.position.altitude = GLOBAL_ORIGIN.ALT + AGENT_LOCAL_TARGET.Z;
            break;
        case HRL_NAVIGATE:
            tempx = CLUSTER_LOCAL_TARGET.X;
            tempy = CLUSTER_LOCAL_TARGET.Y;
            tempz = CLUSTER_LOCAL_TARGET.Z + CLUSTER_OFFSETS.Z;
            setAgentLocalTarget(tempx,tempy,tempz);
            
            //Update the pos_msg based on agent local target and global origin
            pos_msg.pose.position.latitude = convObject.meters_to_lat(GLOBAL_ORIGIN.LAT,GLOBAL_ORIGIN.LON,AGENT_LOCAL_TARGET.X);
            pos_msg.pose.position.longitude = convObject.meters_to_lon(GLOBAL_ORIGIN.LAT,GLOBAL_ORIGIN.LON,AGENT_LOCAL_TARGET.Y);
            pos_msg.pose.position.altitude = GLOBAL_ORIGIN.ALT + AGENT_LOCAL_TARGET.Z;
            break;
        default:
            printf("INVALID FLIGHT MODE REQUESTED\n");
            break;
    };
}

void agent::setAgentLocalTarget(double x, double y, double z) {
    //where the agent should be in the local 'meters' frame
    AGENT_LOCAL_TARGET.X = x;
    AGENT_LOCAL_TARGET.Y = y;
    AGENT_LOCAL_TARGET.Z = z;
    printf("Agent Local Target Set: %f, %f, %f\n", AGENT_LOCAL_TARGET.X, AGENT_LOCAL_TARGET.Y, AGENT_LOCAL_TARGET.Z);
}

void agent::setClusterGlobalTarget(double lat, double lon, double alt) {
    CLUSTER_GLOBAL_TARGET.LAT = lat;
    CLUSTER_GLOBAL_TARGET.LON = lon;
    CLUSTER_GLOBAL_TARGET.ALT = alt;
    printf("Agent Global Target Set: %f, %f, %f\n", CLUSTER_GLOBAL_TARGET.LAT, CLUSTER_GLOBAL_TARGET.LON, CLUSTER_GLOBAL_TARGET.ALT);
}

// #define CLUSTER_GUIDED_LOCAL_MODE 1
// #define CLUSTER_RALLY_MODE 2
// #define CLUSTER_GUIDED_GLOBAL_MODE 3
// #define ALTITUDE_SEP_MODE 4
// #define FOLLOW_THE_LEADER_MODE 5

void agent::setFlightMode(int mode) {
    switch (mode) {
        case CLUSTER_GUIDED_LOCAL_MODE:
            HRL_FLIGHTMODE = mode;
            FTL_ENABLE = false;
            printf("Flight mode set to CLUSTER_GUIDED_LOCAL_MODE\n");
            break;
        case CLUSTER_RALLY_MODE:
            HRL_FLIGHTMODE = mode;
            FTL_ENABLE = false;
            printf("Flight mode set to CLUSTER_RALLY_MODE\n");
            break;
        case CLUSTER_GUIDED_GLOBAL_MODE:
            HRL_FLIGHTMODE = mode;
            FTL_ENABLE = false;
            printf("Flight mode set to CLUSTER_GUIDED_GLOBAL_MODE\n");
            break;
        case ALTITUDE_SEP_MODE:
            if (CURRENT_POS.LAT == 0)
                printf("Flighmode change to ATLTITUDE_SEP_MODE rejected due to no GPS\n");
            else {
                HRL_FLIGHTMODE = mode;
            }
            FTL_ENABLE = false;
            break;
        case FOLLOW_THE_LEADER_MODE:
            HRL_FLIGHTMODE = mode;
            FTL_ENABLE = true;
            printf("Flight mode set to Follow the leader\n");
            break;
        case FORCE_COLLISION:
            HRL_FLIGHTMODE = mode;
            FTL_ENABLE = false;
            printf("Flight mode set to FORCE_COLLISION\n");
            break;
        case AGENT_GUIDED_LOCAL:
            HRL_FLIGHTMODE = mode;
            FTL_ENABLE = false;
            printf("Flight mode set to AGENT_GUIDED_LOCAL\n");
            break;
        case AGENT_GUIDED_GLOBAL:
            HRL_FLIGHTMODE = mode;
            FTL_ENABLE = false;
            printf("Flight mode set to AGENT_GUIDED_GLOBAL\n");
            break;
        case HRL_NAVIGATE:
            HRL_FLIGHTMODE = mode;
            FTL_ENABLE = false;
            printf("Flight mode set to HRL_NAVIGATE\n");
        default: 
            printf("INVALID FLIGHTMODE SELECTION\n");
            break;
    }
}

void agent::setClusterOffsets(int order) {
    //This function is used to update the cluster offsets, ONLY CALL AFTER THE RADIUS HAS BEEN SET!!
    switch (CLUSTER_TYPE) {
        case CIRCLE_FORMATION:
            CLUSTER_OFFSETS.X = circleXOffset(order);
            CLUSTER_OFFSETS.Y = circleYOffset(order);
            CLUSTER_OFFSETS.Z = AGENT_ALT;
            break;
        case LINEAR_FORMATION: //Line cluster formation
            CLUSTER_OFFSETS.X = calculateXLineOffset();
            CLUSTER_OFFSETS.Y = calculateYLineOffset();
            CLUSTER_OFFSETS.Z = AGENT_ALT;
            break;
        default:
            printf("INVALID CLUSTER TYPE ATTEMPTED\n");
        break;
    }
}

double agent::circleXOffset(int order) {
    double separation = 360 / CLUSTER_SIZE; //Angular offset between agents in the cluster
    double n = order - 1;
    double offset = CLUSTER_RADIUS*cos(PI/180*(separation*n));
    return offset;
}

double agent::circleYOffset(int order) {
    double separation = 360 / CLUSTER_SIZE; //Angular offset between agents in the cluster
    double n = order - 1;
    double offset = CLUSTER_RADIUS*sin(PI/180*(separation*n));
    return offset;
}

void agent::setSysid(double id) {
    SYSID = id;
    printf("SYSID Set to %f\n",SYSID);
}

void agent::setClusterid(double id) {
    CLUSTERID = id;
    printf("Cluster ID Set to %f\n",CLUSTERID);
}

void agent::setClusterOrder(double order) {
    CLUSTER_ORDER = order;
    printf("Cluster Order Set to %f\n",CLUSTER_ORDER);
}

void agent::setOriginalClusterOrder(double order) {
    ORIGINAL_CLUSTER_ORDER = order;
    printf("Original Cluster Order Set to %f\n",ORIGINAL_CLUSTER_ORDER);
}

void agent::setClusterSize(double size) {
    CLUSTER_SIZE = size;
    printf("Cluster size Set to %f\n",CLUSTER_SIZE);
}

void agent::setClusterRadius(double radius) {
    CLUSTER_RADIUS = radius;
    printf("Cluster radius Set to %f\n",CLUSTER_RADIUS);
}

void agent::setAgentAlt(double alt) {
    AGENT_ALT = std::min(alt,MAX_ALT);
    printf("Agent Altitude Set to %f\n",AGENT_ALT);
}

void agent::setHome(double lat, double lon, double alt) {
    GLOBAL_ORIGIN.LAT = lat;
    GLOBAL_ORIGIN.LON = lon;
    GLOBAL_ORIGIN.ALT = alt;
    printf("Global Origin Set to %f, %f, %f\n",GLOBAL_ORIGIN.LAT, GLOBAL_ORIGIN.LON, GLOBAL_ORIGIN.ALT);
}

void agent::setRally1(double x, double y) {
    CLUSTER_RALLY1.X = x;
    CLUSTER_RALLY1.Y = y;
    CLUSTER_RALLY1.Z = AGENT_ALT;
    printf("Cluster Rally 1 Set to %f, %f, %f\n",CLUSTER_RALLY1.X, CLUSTER_RALLY1.Y, CLUSTER_RALLY1.Z);
}

double agent::getRally1X() {
    return CLUSTER_RALLY1.X;
}

double agent::getRally1Y() {
    return CLUSTER_RALLY1.Y;
}

double agent::getRally1Z() {
    return CLUSTER_RALLY1.Z;
}

void agent::setRally2(double x, double y) {
    CLUSTER_RALLY2.X = x;
    CLUSTER_RALLY2.Y = y;
    CLUSTER_RALLY2.Z = AGENT_ALT;
    printf("Cluster Rally 2 Set to %f, %f, %f\n",CLUSTER_RALLY2.X, CLUSTER_RALLY2.Y, CLUSTER_RALLY2.Z);
}

double agent::getRally2X() {
    return CLUSTER_RALLY2.X;
}

double agent::getRally2Y() {
    return CLUSTER_RALLY2.Y;
}

double agent::getRally2Z() {
    return CLUSTER_RALLY2.Z;
}

double agent::getAgentAlt() {
    return AGENT_ALT;
}

double agent::getAgentLocalTargetX() {
    return AGENT_LOCAL_TARGET.X;
}

double agent::getAgentLocalTargetY() {
    return AGENT_LOCAL_TARGET.Y;
}

double agent::getAgentLocalTargetZ() {
    return AGENT_LOCAL_TARGET.Z;
}

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

void agent::setClusterLocalTarget(double x, double y, double z) {
    //Update where the cluster should be in the local 'meters' frame
    CLUSTER_LOCAL_TARGET.X = x;
    CLUSTER_LOCAL_TARGET.Y = y;
    CLUSTER_LOCAL_TARGET.Z = z;
    printf("Cluster local target set to: %f, %f, %f\n",x,y,z);
}

void agent::setClusterType(int type) {
    CLUSTER_TYPE = type;
    printf("Cluster changed to type %d\n",type);
}

void acknowledge(int agentID,int* command) {
    agentAcknowledgement.type = NAMED_VALUE_FLOAT;
    agentAcknowledgement.header.stamp = ros::Time::now();
    
    //build and publish the acknowledgement
    char str[10];
    sprintf(str, "A%d$%d", agentID, *command);
    agentAcknowledgement.name = str; 
    ak_pub.publish(agentAcknowledgement);
}

double agent::getSysid() {
    return SYSID;
}

double agent::calculateXLineOffset() {
    double length = (CLUSTER_SIZE - 1) * CLUSTER_RADIUS;
    double offset = (CLUSTER_ORDER - 1) * length;
    offset = offset - length / 2; //Need to think about this one some more
    return offset;
}

double agent::calculateYLineOffset() {
    return 0;
}

int agent::getFlightMode() {
    return HRL_FLIGHTMODE;
}

double agent::getClusterid() {
    return CLUSTERID;
}

void agent::setPublish(bool logic) {
    PUBLISH = logic;
    printf("Publish status is: %d\n", PUBLISH);
}

bool agent::getPublish() {
    return PUBLISH;
}

void agent::setCurrentPos(double lat, double lon, double alt) {
    CURRENT_POS.LAT = lat;
    CURRENT_POS.LON = lon;
    CURRENT_POS.ALT = alt;
}

void agent::setFTLSysid(int id) {
    FTL_SYSID = id;
    printf("FTL SYSID set to %d\n", FTL_SYSID);
}

int agent::getFTLSysid() {
    return FTL_SYSID;
}

void agent::setFTLloc(double lat, double lon, double alt) {
    FTL_LOC.LAT = lat;
    FTL_LOC.LON = lon;
    FTL_LOC.ALT = alt;
}

double agent::getClusterXOffset() {
    return CLUSTER_OFFSETS.X;
}

double agent::getClusterYOffset() {
    return CLUSTER_OFFSETS.Y;
}
    
void agent::enableCircle() {
    CIRCLE_FLAG = true;
    printf("Circling Enabled\n");
}

void agent::disableCircle() {
    CIRCLE_FLAG = false;
    printf("Circling Disabled\n");
    setClusterOffsets(getOriginalClusterOrder());
    setPositionTarget();
}

double agent::getOriginalClusterOrder() {
    return ORIGINAL_CLUSTER_ORDER;
}

double agent::getClusterOrder() {
    return CLUSTER_ORDER;
}

double agent::getClusterSize() {
    return CLUSTER_SIZE;
}

int agent::getCircleTicks() {
    return CIRCLE_TICKS;
}

void agent::setCircleTicks(int ticks) {
    if (ticks < 0) {
        //If a negative value comes across, just set it to a nice, safe 10
        CIRCLE_TICKS = 10;
    }
    else {
        CIRCLE_TICKS = ticks;
    }
    printf("Circle Ticks updated to %d\n", CIRCLE_TICKS);
}

void agent::setClusterOffsetZ (double altitude) {
    CLUSTER_OFFSETS.Z = altitude - CLUSTER_LOCAL_TARGET.Z;
    //update agent alt
    AGENT_ALT = CLUSTER_OFFSETS.Z;

}

void parseCommandUnitTest()
{
     int cluster;
     int command;
     std::string msg = "C123$987";
     std::cout << "msg sent: " << msg << std::endl;
     parseCommand(msg, &cluster, &command);
     std::cout << "parseCommand results:" << std::endl;
     std::cout << "\tCluster: " << cluster << std::endl;
     std::cout << "\tCommand: " << command << std::endl;

     msg = "C1$9";
     std::cout << "\nmsg sent: " << msg << std::endl;
     parseCommand(msg, &cluster, &command);
     std::cout << "parseCommand results:" << std::endl;
     std::cout << "\tCluster: " << cluster << std::endl;
     std::cout << "\tCommand: " << command << std::endl;

     msg = "C15$9";
     std::cout << "\nmsg sent: " << msg << std::endl;
     parseCommand(msg, &cluster, &command);
     std::cout << "parseCommand results:" << std::endl;
     std::cout << "\tCluster: " << cluster << std::endl;
     std::cout << "\tCommand: " << command << std::endl;

     msg = "C1$90";
     std::cout << "\nmsg sent: " << msg << std::endl;
     parseCommand(msg, &cluster, &command);
     std::cout << "parseCommand results:" << std::endl;
     std::cout << "\tCluster: " << cluster << std::endl;
     std::cout << "\tCommand: " << command << std::endl;

     msg = "C01$02";
     std::cout << "\nmsg sent: " << msg << std::endl;
     parseCommand(msg, &cluster, &command);
     std::cout << "parseCommand results:" << std::endl;
     std::cout << "\tCluster: " << cluster << std::endl;
     std::cout << "\tCommand: " << command << std::endl;

}

void agent::setAgentGlobalTarget(double lat, double lon, double alt) {
    AGENT_GLOBAL_TARGET.LAT = lat;
    AGENT_GLOBAL_TARGET.LON = lon;
    AGENT_GLOBAL_TARGET.ALT = alt;
}

int read_integer_from_json_file(const string &filename, const string &key) {
    ifstream input_file(filename);
    if (!input_file.is_open()) {
        cerr << "Error: Couldn't open the file" << endl;
        exit(EXIT_FAILURE);
    }

    json j;
    input_file >> j;
    input_file.close();

    if (j.contains(key) && j[key].is_number_integer()) {
        return j[key];
    } else {
        cerr << "Error: Key not found or not an integer" << endl;
        exit(EXIT_FAILURE);
    }
}

void hrlStateUpdate(MatrixXd swarm_comp, MatrixXd index_encodings, MatrixXd raw_state) {
    int numberOfClusters = swarm_comp.rows();
    int maxAgents = swarm_comp.cols();

    for (int i = 0; i < numberOfClusters; i++) {
        for (int j = 0; j < maxAgents; j++) {
            if (swarm_comp(i,j) > 0) {
                int raw_state_index = (swarm_comp(i,j) - 1) * 2;
                hrl_state( index_encodings(i,j)    , 0) = raw_state(raw_state_index    , 0);
                hrl_state( index_encodings(i,j) + 1, 0) = raw_state(raw_state_index + 1, 0);
            }
        }
    }
    //std::cout << hrl_state << std::endl;
}

int getHRLIndex(int sysID, MatrixXd swarm_comp, MatrixXd index_encodings) {
    int numberOfClusters = swarm_comp.rows();
    int maxAgents = swarm_comp.cols();

    for (int i = 0; i < numberOfClusters; i++) {
        for (int j = 0; j < maxAgents; j++) {
            if (swarm_comp(i,j) == sysID) {
                std::cout << "My Index is: " << index_encodings(i,j) << std::endl;
                return(index_encodings(i,j));
            }
        }
    }
}

int getHRLCluster(int sysID, MatrixXd swarm_comp) {
    int numberOfClusters = swarm_comp.rows();
    int maxAgents = swarm_comp.cols();

    for (int i = 0; i < numberOfClusters; i++) {
        for (int j = 0; j < maxAgents; j++) {
            if (swarm_comp(i,j) == sysID) {
                std::cout << "My cluster is: " << i << std::endl;
                return i;
            }
        }
    }
}

double agent::getAgentGlobalLat() {
    return GLOBAL_ORIGIN.LAT;
}

double agent::getAgentGlobalLon() {
    return GLOBAL_ORIGIN.LON;
}


#endif