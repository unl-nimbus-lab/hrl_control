#ifndef STATE_PROVIDER_H
#define STATE_PROVIDER_H

#include "ros/ros.h"
#include "mavros_msgs/SwarmGPS.h"
#include "nav_msgs/Odometry.h"
#include "custom_libs/geo_converter.hpp"
#include "geometry_msgs/AccelStamped.h"

class StateProvider {
    public:
        StateProvider();

        ros::NodeHandle nh;

        ros::Publisher state_pub;
        
        ros::Subscriber pos_sub;
        void posCallback(const mavros_msgs::SwarmGPS::ConstPtr& msg);

        ros::Subscriber vel_sub;
        void velCallback(const nav_msgs::Odometry::ConstPtr& msg);

        ros::Timer state_pub_timer;
        void publishState(const ros::TimerEvent& event);

        geo_converter converter;

        int SYSID_THISMAV;

        geometry_msgs::AccelStamped state_msg;




    private:
        struct{
            double lat;
            double lon;
        } global_origin;

        struct{
            double x;
            double y;
            double z;
            double vx;
            double vy;
            double vz;
        } agent_state;
};
#endif