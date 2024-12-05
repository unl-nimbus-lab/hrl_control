#include "state_provider/state_provider.h"

//This node publishes the agent's state [x, y, z, vx, vy, vz] converted from GPS to local coordinates,
//relative to a global orgign (RMF is the default)

StateProvider::StateProvider() {


    nh.param<int>("SystemID", SYSID_THISMAV, 1);
    nh.param<double>("homeLat", global_origin.lat, 40.846740);
    nh.param<double>("homeLon", global_origin.lon, -96.471819);

    state_pub = nh.advertise<geometry_msgs::AccelStamped>("/agent_state", 10, true);

    pos_sub = nh.subscribe("/mavros/global_position/swarm", 10, &StateProvider::posCallback, this);
    vel_sub = nh.subscribe("/mavros/global_position/local", 10, &StateProvider::velCallback, this);

    state_pub_timer = nh.createTimer(ros::Duration(0.04), &StateProvider::publishState, this);

}

double sign(double input) {
    //Deterine the sign of a double
    if (input < 0) {
        return -1.0;
    } else {
        return 1.0;
    }
}

void StateProvider::posCallback(const mavros_msgs::SwarmGPS::ConstPtr& msg) {
    if (msg->system_id == SYSID_THISMAV) {//Check if this message is for this agent
        //This message is for this agent
        double lat = msg->latitude;
        double lon = msg->longitude;

        //Assumption: we are working in the northern hemisphere
        double y_sign = sign(lat - global_origin.lat);
        double x_sign = sign(lon - global_origin.lon);

        //See custom libs for more deatails but this is just the haversine formula
        agent_state.y = converter.gps_to_meters(global_origin.lat, global_origin.lon, lat, global_origin.lon);
        agent_state.y = y_sign * agent_state.y;

        agent_state.x = converter.gps_to_meters(global_origin.lat, global_origin.lon, global_origin.lat, lon);
        agent_state.x = x_sign * agent_state.x;

        agent_state.z = msg->altitude;
    }
}

void StateProvider::velCallback(const nav_msgs::Odometry::ConstPtr& msg) { //This is assuming we are only getting velocity from the host agent
    agent_state.vx = msg->twist.twist.linear.x;
    agent_state.vy = msg->twist.twist.linear.y;
    agent_state.vz = -1.0 * msg->twist.twist.linear.z;
}

void StateProvider::publishState(const ros::TimerEvent& event) {
    //Encode the agent's position into the linear part of the accel message
    state_msg.accel.linear.x = agent_state.x;
    state_msg.accel.linear.y = agent_state.y;
    state_msg.accel.linear.z = agent_state.z;

    //Encode the agent's velocity into the angular part of the accel message
    state_msg.accel.angular.x = agent_state.vx;
    state_msg.accel.angular.y = agent_state.vy;
    state_msg.accel.angular.z = agent_state.vz;

    state_msg.header.stamp = ros::Time::now();

    state_pub.publish(state_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "state_provider");
    StateProvider state_provider;
    ros::spin();
}
