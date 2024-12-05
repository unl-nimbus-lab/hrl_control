#include "hrl_planner.h"

//Global Variables
float hrl_publish_rate = 0.5;
int hrl_propogation_steps = 5;
bool publish_flag = false;
float integral_state_x = 0.0;
float integral_state_y = 0.0;

ros::Publisher state_pub;

//READ IN TRANSFORM MATRICIES

//READ IN GAIN MATRIX

//STATE UPDATE CALLBACK
void state_update_callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    /* This function updates this vehicle's state based on information from
    the agent_position_xy topic */
    X << msg->point.x, msg->point.y, 0.0, 0.0, integral_state_x, integral_state_y;
    //ROS_INFO_STREAM("X: " << msg->point.x << " Y: " << msg->point.y);
    state_message.ixx = X(0,0);
}

//COMMAND PARSE CALLBACK
void debug_vector_callback(const mavros_msgs::DebugValue::ConstPtr& msg) {
    mavros_msgs::DebugValue debug_message = *msg;
    int *target_entity = new int;
    int *command = new int;
    char entity = debug_message.name[0];
    parseCommand(debug_message.name, target_entity, command);
    ROS_INFO("DEBUG COMMAND PARSED");

    switch(entity) {
        // //Agent Command
        // case 'A':
        //     switch(*command):

        //     break;
        // break;
        //Cluster Command
        case 'C':
            switch(*command) {
                case ENABLE_PUBLISH:
                    publish_flag = true;
                    break;
                case DISABLE_PUBLISH:
                    publish_flag = false;
                    break;
            }
            break;
        // //Swarm Command
        // case 'S':
        //     switch(*command):
        //         c
        //     break;
        // break;
    }
}

//Specify the setpoints
double x_target, y_target;

double RMF_ORIGIN_LAT = 40.8467784;
double RMF_ORIGIN_LON = -96.4719180;
geo_converter convObject;
geographic_msgs::GeoPoseStamped position_target_msg;

int main(int argc, char **argv) {

    //Assign Values to Variables:

    x_target = 25.0;
    y_target = 60.0;

    A << 0, 0, 1, 0,
         0, 0, 0,  1,
         0, 0, -1, 0,
         0, 0, 0, -1,

    B << 0, 0,
         0, 0,
         -1, 0,
         0, -1;

    K << -1, 0, -1, 0,
         0, -1, 0, -1;

    //I << 0, 0, 0, 0, x_target, y_target;

    //Setup the hrl_planner node
    ros::init(argc, argv, "hrl_planner");
    ros::NodeHandle nh;

    //ROS subscribers
    ros::Subscriber swarm_state_sub = nh.subscribe("/agent_position_xy",1, state_update_callback);
    ros::Subscriber debug_vector_sub = nh.subscribe("/mavros/debug_value/debug_vector", 10, debug_vector_callback);


    //ROS publishers
    ros::Publisher gps_setpoint_pub = nh.advertise<geographic_msgs::GeoPoseStamped>("/mavros/setpoint_position/global", 10, true);
    state_pub = nh.advertise<geometry_msgs::Inertia>("/vehicle_state", 10, true);

    //begin main ROS LOOP
    while (ros::ok()) {

        T_X = X;

        for (int i = 0; i < 2; i++) {
            //Calculate the control input, u = -kx
            U = K*T_X;
            U(0,0) = input_saturtion_check(U(0,0), max_input);
            U(1,0) = input_saturtion_check(U(1,0), max_input);

            ROS_INFO_STREAM("U1: " << U(0,0) << " U2: " << U(1,0));

            //std::cout << "\nStart\n" << A << "\n" << X << "\n" << I << "\n" << B << "\n" << U << std::endl;

            //Propogate the control input forward, x + 1 = Ax + Bu + Zd
            DX = A*T_X + B*U;
            ROS_INFO_STREAM("NEXT TARGET: X: " << DX(0,0) << " Y: " << DX(1,0) );
            T_X = DX;
        }

        state_message.ixx = X(0,0);
        state_message.ixy = X(1,0);
        state_message.ixz = X(2,0);
        state_message.iyy = X(3,0);

        state_pub.publish(state_message);

        if (publish_flag) {
            position_target_msg.pose.position.latitude =  convObject.meters_to_lat(RMF_ORIGIN_LAT,RMF_ORIGIN_LON, DX(1,0));
            position_target_msg.pose.position.longitude = convObject.meters_to_lon(RMF_ORIGIN_LAT,RMF_ORIGIN_LON, DX(0,0));
            position_target_msg.pose.position.altitude =  410.0;

            //publish the next setpoint to the FMU
            gps_setpoint_pub.publish(position_target_msg);
        }

        ros::Duration(hrl_publish_rate).sleep();
        ros::spinOnce();
    }

    return 0;
}
