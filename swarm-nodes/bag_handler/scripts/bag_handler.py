#!/usr/bin/env python3

import sys
import os
import rospy
import rosbag
import subprocess
import signal
from mavros_msgs.msg import State

#Hacked together node that starts a new rosbag when the drones is armed
#and gracefully stops the recording when the drone is disarmed to prevent 
#corrupt bag files. This is a temporary solution until a better one is found.

old_state = False
rosbag_process = None
bag_dir = None

def start_recording():
    global rosbag_process
    global bag_dir
    rosbag_process = subprocess.Popen(["rosbag", "record", "-o", bag_dir + "/bag", "-a"])

def stop_recording():
    global rosbag_process
    rosbag_process.send_signal(signal.SIGINT)

def mavros_state_callback(msg):
    # Process the mavros state message here
    global old_state
    if msg.armed == True and old_state == False:
        #The drone was not armed, now it is
        rospy.logwarn("DETECTED ARMED, STARTING RECORDING")
        old_state = True
        start_recording()

    elif msg.armed == False and old_state == True:
        #The drone was armed, now it is not
        rospy.logwarn("DETECTED DISARMED, STOPPING RECORDING")
        old_state = False
        stop_recording()
    else: 
        #No change, do nothing
        pass

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('mavros_state_subscriber')

    bag_dir = rospy.get_param('bagDir')
    rospy.loginfo("Bag directory: " + bag_dir)

    # Subscribe to the mavros state topic
    rospy.Subscriber('/mavros/state', State, mavros_state_callback)

    # Spin the ROS node
    rospy.spin()