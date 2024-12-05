#include "command_xform/command_xform.h"

//This node converts 3D thrust vectors to attitude commands as they are received

CommandXform::CommandXform() {
    att_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 2);
    vector_sub = nh.subscribe("vector", 2, &CommandXform::vectorCallback, this);
}

void CommandXform::vectorCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
    //Formula used for conversion:

    //roll = tan-1(Fy/Fz)
    //pitch = tan-1(-Fx/sqrt(Fy^2 + Fz^2))
    //yaw = 0 (fixed because why not)
    //thrust = sqrt(Fx^2 + Fy^2 + Fz^2)

    //Coordinate transroming
    float Fx = -1.0 * msg->vector.x;
    float Fy = -1.0 * msg->vector.y;
    float Fz = msg->vector.z;

    //Apply conversion formula
    float roll = atan2(Fy, Fz);
    float pitch = atan2(-Fx, sqrt(Fy*Fy + Fz*Fz));
    float yaw = 0.0;

    float thrust = sqrt(Fx*Fx + Fy*Fy + Fz*Fz);

    //Convert roll pitch yaw thrust to quaternion
    q.setRPY(roll, pitch, yaw);

    att_target.orientation.x = q.x();
    att_target.orientation.y = q.y();
    att_target.orientation.z = q.z();
    att_target.orientation.w = q.w();
    att_target.thrust = thrust;
    att_target.type_mask = 0b00000111 ; // bitmask to ignore roll, pitch, yaw rates
    att_target.header.stamp = ros::Time::now();
    att_pub.publish(att_target);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "command_xform");
    CommandXform command_xform;
    ros::spin();
    return 0;
}
