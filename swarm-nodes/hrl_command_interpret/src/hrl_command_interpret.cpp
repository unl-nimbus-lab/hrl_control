#include "hrl_command_interpret/hrl_command_interpret.h"

//This node is used to interpret commands sent from the ground station and convert them into a form that can be used by the HRLController

HRLCommandInterpret::HRLCommandInterpret() {

    //Set the global origin from the parameter server so they can be converted to local coordinates if needed
    nh.param<double>("homeLat", global_origin.lat, 40.846740);
    nh.param<double>("homeLon", global_origin.lon, -96.471819);

    aug_state_pub = nh.advertise<geometry_msgs::PointStamped>("/aug_state", 2);
    publish_state_pub = nh.advertise<std_msgs::Bool>("/publish_state", 2);

    debug_vector_sub = nh.subscribe("/mavros/debug_value/debug_vector", 5, &HRLCommandInterpret::debugVecCallback, this);
    debug_int_sub = nh.subscribe("/mavros/debug_value/named_value_int", 5, &HRLCommandInterpret::debugIntCallback, this);
}

void HRLCommandInterpret::updateAugState(float x_target, float y_target, int cluster_id) {
    //send new aug state component to the agents (cluster target)
    aug_state.header.stamp = ros::Time::now();
    aug_state.point.x = x_target;
    aug_state.point.y = y_target;
    aug_state.point.z = cluster_id;
    aug_state_pub.publish(aug_state);
}

void HRLCommandInterpret::debugVecCallback(const mavros_msgs::DebugValue::ConstPtr& msg) {
    //Handle commands sent in the form of debug vectors
    //Command set:
    //CXXX$XXX Where the "XXX" after the "C" identifies the cluster and
    //the "XXX" after the "$" identifies the command
    
    //CXX$1: target as global position
    //CXX$2: target as local position
    parseCommand(msg->name, msg_tgt_cluster, msg_command);

    switch(*msg_command) {
        case 1: //Command type 1: tagets as global position
            lat = msg->data[0];
            lon = msg->data[1];

            //Assumption: we are working in the northern hemisphere
            y_sign = sign(lat - global_origin.lat);
            x_sign = sign(lon - global_origin.lon);

            y_target = converter.gps_to_meters(global_origin.lat, global_origin.lon, lat, global_origin.lon);
            y_target = y_sign * y_target;

            x_target = converter.gps_to_meters(global_origin.lat, global_origin.lon, global_origin.lat, lon);
            x_target = x_sign * x_target;
            //conert to local
            updateAugState(x_target, y_target, *msg_tgt_cluster);
                break;

        case 2://Command type 2: target as local position
            ROS_INFO("Received local position target command");
            updateAugState(msg->data[0], msg->data[1], *msg_tgt_cluster);
                break;

        default:
            ROS_ERROR("Invalid command received: %d", *msg_command);
                break;
    }

}

void HRLCommandInterpret::debugIntCallback(const mavros_msgs::DebugValue::ConstPtr& msg) {
    //Handle commands sent in the form of debug integers to turn on/off the publishing of the state of the HRLcontroller
    //Command set (integer value)
    //1: turn on
    //2: turn off
    std_msgs::Bool publish_state;
    switch(msg->value_int) {
        case 1:
            publish_state.data = true;
            publish_state_pub.publish(publish_state);
                break;
        case 2:
            publish_state.data = false;
            publish_state_pub.publish(publish_state);
                break;
        default:
            ROS_ERROR("Invalid INT command received");
                break;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "hrl_command_interpret");
    HRLCommandInterpret hrl_command_interpret;
    ros::spin();
    return 0;
}