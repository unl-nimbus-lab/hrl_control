#include "hrl_control/hrl_control.h"

//This node uses state feedback to compute control inputs, its behavior can be modified by 
//sending debug vectors (see hrl_command_interpret for more information)
 
HRLController::HRLController() {
    //ROS Publishes, Subscribers, and Services
    control_pub = nh.advertise<geometry_msgs::Vector3Stamped>("vector", 2);
    
    state_sub = nh.subscribe("/agent_state", 2, &HRLController::stateCallback, this);
    aug_state_sub = nh.subscribe("/aug_state", 1, &HRLController::augStateCallback, this);
    publish_state_sub = nh.subscribe("/publish_state", 1, &HRLController::publishStateCallback, this);

    set_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    guided_msg.request.custom_mode = "GUIDED";
    brake_msg.request.custom_mode = "BRAKE";

    //Timer that tiggers the main control loop
    controller_timer = nh.createTimer(ros::Duration(controller_period), &HRLController::computeControl, this);

    //Be sure to use absolute paths to the system matricies
    system_matricies = reader.readJSON("/home/catkin_ws/src/hrl_control/src/HRL_Sys.json");
    
    //Resize and initialize the current state of the swarm to zeros since we may not have full feedback yet
    int numberOfHRLStates = system_matricies.ZD.size();
    int numberOfAgents = system_matricies.p.sum();
    ROS_WARN("Number of agents is %d", numberOfAgents);

    nh.param<int>("SystemID", SYSID_THISMAV, 1);
    ROS_WARN("Set SYSID_THISMAV to %d", SYSID_THISMAV);

    nh.param<int>("ClusterID", CLSID_THISMAV, 1);
    ROS_WARN("Set CLUSTERID_THISMAV to %d", CLSID_THISMAV);

    nh.param<double>("agentAlt", target_z, 400);
    ROS_WARN("Set target_z to %f", target_z);

    nh.param<double>("thrustScalar",thrust_scalar, 1.0);
    ROS_WARN("Set thrust_scalar to %f", thrust_scalar);

    //Its helpful to have the state indices in a vector of IDXs
    IDXs.resize(numberOfAgents,1);
    IDXs = vectorIdXs(system_matricies.state_idxs_C);

    //Reshape and initialize the state vector,
    X.resize(numberOfHRLStates, 1);
    X.setZero();

    //Initialize the AUG_STATE (cluster position targets)
    //Helpful to have the aug state indexes for each cluster, that is,
    //tell AUG_STATE_IDXs your cluster, and it will tell you the index of the AUG_STATE
    AUG_STATE_IDXs.resize(system_matricies.p.rows(), 1);
    AUG_STATE_IDXs = generateAugStateIDXs(system_matricies.p);

    AUG_STATE.resize(numberOfHRLStates, 1);
    AUG_STATE.setZero();

    int numberOfClusters = system_matricies.p.rows();

    //Generate default augmented states (this is only hard coded for 2 clusters for now)
    for (int i = 0; i < numberOfClusters; i++) {
        float angle = 2*3.14/numberOfClusters * i;
        AUG_STATE( AUG_STATE_IDXs(i)     ,0) = 100.0*cos(angle); //Cluster X
        AUG_STATE( AUG_STATE_IDXs(i) + 1 ,0) = 100.0*sin(angle); //Cluster Y
    }
    ROS_INFO("DEFAULT AUG_STATE: ");
    std::cout << "AUG_STATE: " << std::endl << AUG_STATE << std::endl;
    ROS_INFO("===============");
    // AUG_STATE( AUG_STATE_IDXs(0)     ,0) =     0.0; //Cluster 1 X
    // AUG_STATE( AUG_STATE_IDXs(0) + 1 ,0) =   100.0; //Cluster 1 Y
    // AUG_STATE( AUG_STATE_IDXs(1)     ,0) =     0.0; //Cluster 2 X
    // AUG_STATE( AUG_STATE_IDXs(1) + 1 ,0) =  -100.0; //Cluster 2 Y


    //Create vector of integral state indices

    for (int i = 0; i < numberOfClusters; i++) {
        int tempAgents = system_matricies.p(i,0);
        numberOfIntStates = numberOfIntStates + 2*(tempAgents - 1) + 2;
    }
    INT_STATE_IDXs.resize(numberOfIntStates, 1);
    INT_STATE_IDXs = generateIntStateIDXs(system_matricies.p);

    //This should be number of agents x 2
    OUTPUT_RAW.resize(numberOfAgents * 2, 1);
    OUTPUT_RAW.setZero();

    //Inverse of transformation matrix
    IT = system_matricies.T.inverse();

    //Error vector
    ERROR.resize( 2, 1);
    ERROR.setZero();

    //This agent's output index from the LQR controller
    OUTIDX = (SYSID_THISMAV - 1) * 2;

    x_z << 0.0, 0.0, 0.0;

    //Test the helper functions
    //vectorIdXsTest();
    //generateAugStateIDXsTest();
    //generateIntStateIDXsTest();
}

void HRLController::stateCallback(const geometry_msgs::AccelStamped::ConstPtr& msg) {
    //Update the system's state from the message
    x_z(0,0) = msg->accel.linear.z;
    x_z(1,0) = msg->accel.angular.z;

    int i = IDXs(SYSID_THISMAV - 1,0); //Get the idx of this agent

    X = IT*X; //Un-transform X

    //Add in this agents state to the vector
    X(i,0) = msg->accel.linear.x;
    X(i+1,0) = msg->accel.linear.y;
    X(i+2,0) = msg->accel.angular.x;
    X(i+3,0) = msg->accel.angular.y;
    X = system_matricies.T*X; //Transform X

}

void HRLController::augStateCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    //Update the augmented state from GCS the set to GUIDED mode
    int cluster_id = msg->point.z;

    if (cluster_id > system_matricies.p.rows() || cluster_id < 1) {
        ROS_ERROR("Cluster ID is out of range");
        return;
    }

    AUG_STATE( AUG_STATE_IDXs(cluster_id - 1)     ,0) = msg->point.x;
    AUG_STATE( AUG_STATE_IDXs(cluster_id - 1) + 1 ,0) = msg->point.y;
    ROS_WARN("Cluster AUG_STATE %d updated to %f, %f", cluster_id, msg->point.x, msg->point.y);
    std::cout << "AUG_STATE: " << std::endl << AUG_STATE << std::endl;

    if (cluster_id == CLSID_THISMAV) {
        publish_flag = true;
        set_mode.call(guided_msg);
    }
}

void HRLController::publishStateCallback(const std_msgs::Bool::ConstPtr& msg) {
    //Update the flag to publish the state
    publish_flag = msg->data;
}

Eigen::MatrixXd HRLController::vectorIdXs(Eigen::MatrixXd input) {
    //This function takes a matrix of indices and reshapes it into a vector
    int count = 0;
    Eigen::MatrixXd output(input.rows()*input.cols(),1);
    for (int i = 0; i < input.rows(); i++) {
        for (int j = 0; j < input.cols(); j++) {
            output(count,0) = input(i,j);
            count++;
        }
    }
    return output;
}

Eigen::MatrixXd HRLController::generateAugStateIDXs(Eigen::MatrixXd swarmComp) {
    //This function takes a swarm composition and returns the indices of the aug_states for each cluster
    //swarmComp looks like [2 4]' for a swar with 2 clusters, one with 2 agents and one with 4 agents

    //The calculation here is in each cluster, there are 4 states per agent, 2(n-1) relative integral states
    //and 2 cluster-centroid integral states
    int numberOfClusters = swarmComp.rows();
    //std::cout << "number of clusters: " << numberOfClusters << std::endl;
    int tempIdx = 0;
    int outIdx = 0;
    Eigen::MatrixXd output(numberOfClusters,1);

    for (int i = 0; i < numberOfClusters; i++) {
        int numberOfAgents = swarmComp(i,0);
        tempIdx = tempIdx + 4*(numberOfAgents - 1);
        output(outIdx++,0) = tempIdx;
        tempIdx = tempIdx + 2 + 2*(numberOfAgents - 1) + 4;
    }
    //std::cout << "aug state IDXs: " << std::endl << output << std::endl;
    return output;

}

Eigen::MatrixXd HRLController::generateIntStateIDXs(Eigen::MatrixXd swarmComp) {
    //This function takes a swarm composition and returns the indices of the integral states for each cluster
    //swarmComp looks like [2 4]' for a swar with 2 clusters, one with 2 agents and one with 4 agents

    //The calculation here is in each cluster, there are 4 states per agent, 2(n-1) relative integral states
    //and 2 cluster-centroid integral states
    int numberOfClusters = swarmComp.rows();
    //First determine how many integral states there are
    int numberOfIntStates = 0;
    for (int i = 0; i < numberOfClusters; i++) {
        int tempAgents = swarmComp(i,0);
        numberOfIntStates = numberOfIntStates + 2*(tempAgents - 1) + 2;
    }
    //std::cout << "number of integral states: " << numberOfIntStates << std::endl;
    
    int tempIdx = 0;
    int outIdx = 0;
    Eigen::MatrixXd output(numberOfIntStates,1);

    for (int i = 0; i < numberOfClusters; i++) {
        int numberOfAgents = swarmComp(i,0);
        tempIdx = tempIdx  + 4*(numberOfAgents);
        for (int j = 0; j < 2*(numberOfAgents - 1); j++) {
            output(outIdx++,0) = tempIdx++;
        }
        output(outIdx++,0) = tempIdx++;
        output(outIdx++,0) = tempIdx++;
    }
    //std::cout << "integral state IDXs: " << std::endl << output << std::endl;
    return output;

}

void HRLController::vectorIdXsTest() {
    //Test the vectorIdxs function
    Eigen::MatrixXd testMat1(2,2);
    testMat1 << 1, 2, 3, 4;
    std::cout << "Test Matrix: " << std::endl << testMat1 << std::endl;
    Eigen::MatrixXd testVec1 = vectorIdXs(testMat1);
    std::cout << "Test Vector: " << std::endl << testVec1 << std::endl;

    Eigen::MatrixXd testMat2(3,3);
    testMat2 << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    std::cout << "Test Matrix: " << std::endl << testMat2 << std::endl;
    Eigen::MatrixXd testVec2 = vectorIdXs(testMat2);
    std::cout << "Test Vector: " << std::endl << testVec2 << std::endl;

    Eigen::MatrixXd testMat3(4,4);
    testMat3 << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16;
    std::cout << "Test Matrix: " << std::endl << testMat3 << std::endl;
    Eigen::MatrixXd testVec3 = vectorIdXs(testMat3);
    std::cout << "Test Vector: " << std::endl << testVec3 << std::endl;

    Eigen::MatrixXd testMat4(2,3);
    testMat4 << 1, 2, 3, 4, 5, 6;
    std::cout << "Test Matrix: " << std::endl << testMat4 << std::endl;
    Eigen::MatrixXd testVec4 = vectorIdXs(testMat4);
    std::cout << "Test Vector: " << std::endl << testVec4 << std::endl;

    Eigen::MatrixXd testMat5(3,2);
    testMat5 << 1, 2, 3, 4, 5, 6;
    std::cout << "Test Matrix: " << std::endl << testMat5 << std::endl;
    Eigen::MatrixXd testVec5 = vectorIdXs(testMat5);
    std::cout << "Test Vector: " << std::endl << testVec5 << std::endl;

}

void HRLController::generateAugStateIDXsTest() {
    //Test the generateAugStateIDXs function
    bool test1 = false;
    bool test2 = false;
    bool test3 = false;
    bool test4 = false;
    bool test5 = false;

    Eigen::MatrixXd testMat1(2,1);
    testMat1 << 2, 2;
    //std::cout << "Test Matrix: " << std::endl << testMat1 << std::endl;
    Eigen::MatrixXd testVec1 = generateAugStateIDXs(testMat1);
    //std::cout << "Test Vector: " << std::endl << testVec1 << std::endl;
    Eigen::MatrixXd truth1(2,1);
    truth1 << 4, 16;
    if (truth1 == testVec1) {
        test1 = true;
    }
    //4 16

    Eigen::MatrixXd testMat2(2,1);
    testMat2 << 4, 4;
    //std::cout << "Test Matrix: " << std::endl << testMat2 << std::endl;
    Eigen::MatrixXd testVec2 = generateAugStateIDXs(testMat2);
    //std::cout << "Test Vector: " << std::endl << testVec2 << std::endl;
    Eigen::MatrixXd truth2(2,1);
    truth2 << 12, 36;
    if (truth2 == testVec2) {
        test2 = true;
    }
    //12 36

    Eigen::MatrixXd testMat3(3,1);
    testMat3 << 4, 4, 4;
    //td::cout << "Test Matrix: " << std::endl << testMat3 << std::endl;
    Eigen::MatrixXd testVec3 = generateAugStateIDXs(testMat3);
    //std::cout << "Test Vector: " << std::endl << testVec3 << std::endl;
    Eigen::MatrixXd truth3(3,1);
    truth3 << 12, 36, 60;
    if (truth3 == testVec3) {
        test3 = true;
    }
    //12 36 60

    Eigen::MatrixXd testMat4(3,1);
    testMat4 << 2, 3, 2;
    //std::cout << "Test Matrix: " << std::endl << testMat4 << std::endl;
    Eigen::MatrixXd testVec4 = generateAugStateIDXs(testMat4);
    //std::cout << "Test Vector: " << std::endl << testVec4 << std::endl;
    Eigen::MatrixXd truth4(3,1);
    truth4 << 4, 20, 34;
    if (truth4 == testVec4) {
        test4 = true;
    }
    //4 20 34

    Eigen::MatrixXd testMat5(2,1);
    testMat5 << 2, 3;
    //std::cout << "Test Matrix: " << std::endl << testMat5 << std::endl;
    Eigen::MatrixXd testVec5 = generateAugStateIDXs(testMat5);
    //std::cout << "Test Vector: " << std::endl << testVec5 << std::endl;
    Eigen::MatrixXd truth5(2,1);
    truth5 << 4, 20;
    if (truth5 == testVec5) {
        test5 = true;
    }
    //4 20

    if (test1 && test2 && test3 && test4 && test5) {
        ROS_WARN("All Augmented State generation tests passed");
    } else {
        ROS_ERROR("One or more tests failed");
    }

}

void HRLController::generateIntStateIDXsTest() {
    bool test1 = false;
    bool test2 = false;
    bool test3 = false;

    //Test the generateIntStateIDXs function
    Eigen::MatrixXd testMat1(2,1);
    testMat1 << 2, 2;
    //std::cout << "Test Matrix: " << std::endl << testMat1 << std::endl;
    Eigen::MatrixXd testVec1 = generateIntStateIDXs(testMat1);
    //std::cout << "Test Vector: " << std::endl << testVec1 << std::endl;
    Eigen::MatrixXd truth1(8,1);
    truth1 << 8, 9, 10, 11, 20, 21, 22, 23;
    if (truth1 == testVec1) {
        test1 = true;
    }

    Eigen::MatrixXd testMat2(2,1);
    testMat2 << 4, 4;
    //std::cout << "Test Matrix: " << std::endl << testMat2 << std::endl;
    Eigen::MatrixXd testVec2 = generateIntStateIDXs(testMat2);
    //std::cout << "Test Vector: " << std::endl << testVec2 << std::endl;
    Eigen::MatrixXd truth2(16,1);
    truth2 << 16, 17, 18, 19, 20, 21, 22, 23, 40, 41, 42, 43, 44, 45, 46, 47;
    if (truth2 == testVec2) {
        test2 = true;
    }

    Eigen::MatrixXd testMat3(3,1);
    testMat3 << 4, 4, 4;
    //std::cout << "Test Matrix: " << std::endl << testMat3 << std::endl;
    Eigen::MatrixXd testVec3 = generateIntStateIDXs(testMat3);
    //std::cout << "Test Vector: " << std::endl << testVec3 << std:each:endl;
    Eigen::MatrixXd truth3(24,1);
    truth3 << 16, 17, 18, 19, 20, 21, 22, 23, 40, 41, 42, 43, 44, 45, 46, 47, 64, 65, 66, 67, 68, 69, 70, 71;
    if (truth3 == testVec3) {
        test3 = true;
    }
    //16-23, 40-47, 64-71

    if (test1 && test2 && test3) {
        ROS_WARN("All Integral State generation tests passed");
    } else {
        ROS_ERROR("One or more tests failed");
    }
}

void HRLController::computeControl( const ros::TimerEvent& event ) {

    if (publish_flag) {//Only publish the state if the flag is set

        //First the altitude PID controller
        error_z = target_z - x_z(0,0); //compute the error

        //If the altitude error is too large, only adjust the altitude
        if ( abs( error_z ) > 3.0 ) {
            control_mod  = 0.0; //turn off the XY controller
            raw_output_z = kp*error_z;//Only going to use P control to avoid large windup
            error_z_integral = 0.0;
            ROS_INFO("Altitude error too large, Adjusting height only");
        } else {
            control_mod = 1.0;//thrust_scalar; //turn on the XY controller and scale the thrust
            //Compute the PID control signal for the altitude
            error_z_integral += error_z * controller_period;
            error_z_derivative = (error_z - error_z_previous) / controller_period;
            error_z_previous = error_z;
            raw_output_z = kp*error_z + ki*error_z_integral + kd*error_z_derivative;
        }

        //Limits on the integral states since these can explode
        for (int i = 0; i < numberOfIntStates; i++) {
            int intStateIdx = INT_STATE_IDXs(i,0);
            X(intStateIdx , 0) = std::max( X(intStateIdx ,0), -50.0);
            X(intStateIdx , 0) = std::min( X(intStateIdx ,0), 50.0);
        }

        OUTPUT_RAW = -1.0 * system_matricies.DKL * (X - AUG_STATE);

        // std::cout << "OUTPUT_RAW: " << std::endl << OUTPUT_RAW(OUTIDX    ,0) << std::endl;
        // std::cout << "OUTPUT_RAW: " << std::endl << OUTPUT_RAW(OUTIDX + 1,0) << std::endl;
        // std::cout << std::endl;

        OUTPUT_RAW( OUTIDX, 0)      = thrust_scalar*OUTPUT_RAW( OUTIDX, 0);
        OUTPUT_RAW( OUTIDX + 1, 0)  = thrust_scalar*OUTPUT_RAW( OUTIDX + 1, 0);

        //Gaurd rails on the control signal, ensures that we don't command extreme angles to the UAV
        OUTPUT_RAW( OUTIDX, 0) = std::max( OUTPUT_RAW( OUTIDX, 0), -1 * thrust_lim);
        OUTPUT_RAW( OUTIDX, 0) = std::min( OUTPUT_RAW( OUTIDX, 0),      thrust_lim);

        OUTPUT_RAW( OUTIDX + 1, 0) = std::max( OUTPUT_RAW( OUTIDX + 1, 0), -1 * thrust_lim);
        OUTPUT_RAW( OUTIDX + 1, 0) = std::min( OUTPUT_RAW( OUTIDX + 1, 0),      thrust_lim);

        fx = OUTPUT_RAW(OUTIDX,     0);
        fy = OUTPUT_RAW(OUTIDX + 1, 0);

        fz = std::max( raw_output_z, thrust_min );//Note, should always have a non-zero thrust for Z
        fz = std::min( fz, thrust_lim_z);
        
        //pack the control_msg, the control mod values are set based on the altitude error
        control_msg.vector.x = control_mod*fx;
        control_msg.vector.y = control_mod*fy;
        control_msg.vector.z = fz;

        auto untransformedX = IT*X;

        //Compute a local error between this agent and the target
        ERROR(0,0) = untransformedX( IDXs( SYSID_THISMAV - 1 )    , 0) - AUG_STATE( AUG_STATE_IDXs (CLSID_THISMAV - 1)    , 0);
        ERROR(1,0) = untransformedX( IDXs( SYSID_THISMAV - 1 ) + 1, 0) - AUG_STATE( AUG_STATE_IDXs (CLSID_THISMAV - 1) + 1, 0);

        //std::cout << "ERROR: " << std::endl << ERROR << std::endl;

        // To avoid "dancing" around the target, switch to BRAKE mode if close, uploading a new
        // waypoint will automatically switch back to GUIDED mode when it is recieved.
        if (ERROR.norm() < 5.0) {
           ROS_WARN("Close to target, stopping");
           set_mode.call(brake_msg);
           publish_flag = false;
        }

        control_pub.publish(control_msg);

        //Propogate the state forward for estimation in case the state feedback is lost
        X = system_matricies.DTA * (X - AUG_STATE) + system_matricies.DTB * OUTPUT_RAW - system_matricies.ZD * controller_period + AUG_STATE;
        
        //auto X_T = IT*X;
        //std::cout << "X: " << std::endl << X_T << std::endl; 

        //update new controller rate
        //float controller_period = 1.0/controller_rate_;
        //controller_timer_.setPeriod( ros::Duration(controller_period), true);
    }

}

int main( int argc, char** argv ) {

    ros::init(argc, argv, "hrl_control");
    HRLController hrl_control;
    ros::spin();
    return 0;
}