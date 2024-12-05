#!/usr/bin/env python3
import sys
import serial
from digi.xbee.devices import *
from digi.xbee.util import utils
from digi.xbee.exception import *
import time
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PointStamped
from swarm_comms.msg import allAgentsPosStamped
from swarm_comms.msg import agentPos
from mavros_msgs.msg import SwarmGPS
#Jack's agent class here:
from agent import Agent

class TheRadio(object):
    '''Class to get xbee data and publish to ros topic'''

    #class vars to transmit position data
    x = '0.0'
    y = '0.0'
    z = '0.0'
    sys_id = '0'
    vx = '0.0'
    vy = '0.0'
    vz = '0.0'
    ix = '0.0'
    iy = '0.0'
    iz = '0.0'
    agent_list = []
    node_list = []
    simming = False


    def millis(self):
        return round(time.time() * 1000)

    def process_command(self, data):
        if (data=='g'):
            transmit_default_data()

    #The data_rcvd_callback is triggered when serial data is received
    def data_rcvd_callback(self,xb_msg):
        time_received = self.millis()
        #print(xb_msg.data.decode('utf8')+'  at: '+ str(time_received))
        #address = xb_msg.remote_device.get_64bit_addr()
        #node_id = self.xnet.get_device_by_64(address).get_node_id()
        data = xb_msg.data.decode('utf8')
        #print(data + ' rcvd at: ' + str(time_received))
        #print("%s: %s" % (node_id, data))
        #add other stuff in here later as needed,
        #such as parsing commands, lat/longs, etc...

        #parse incoming string...
        #print("rcvd:%s" % data)
        #commands recvd from the ground station begin with 'C'
        if (data[0] == 'C'):
            self.process_command(data[1])
        else:
            #self.parse_remote_agent_location(data)
            self.store_remote_agent_data(data, time_received)
            #now publish remote agent data to the agent_pos topic
            allAgents = allAgentsPosStamped()
            for agent in self.agent_list:
                a = agentPos()
                a.id = agent.node_id
                a.x = agent.x
                a.y = agent.y
                a.z = agent.z
                a.vx = agent.vx
                a.vy = agent.vy
                a.vz = agent.vz
                a.ix = agent.ix
                a.iy = agent.iy
                a.iz = agent.iz
                a.time_since_update = agent.time_between_locations
                allAgents.agents.append(a)

            #add time to header
            allAgents.header.stamp = rospy.Time.now()
                        
            #publish to topic
            self.agent_pos_pub.publish(allAgents)


    def parse_remote_agent_location(self,data):

        #agent strings start with 'a'
        if (data[0] == 'a'):
            #print('Agent string rcvd: ' + data)

            idx = 1
            temp_str = ''
            
            #grab the id number
            while (data[idx] != ':'):
                temp_str += data[idx]
                idx += 1
            id_num = int(temp_str)
            temp_str = ''

            #increment idx so we go past the 'x' in the 
            #received string
            idx += 2
            while (data[idx] != 'y'):
                temp_str += data[idx]
                idx += 1
            px = float(temp_str)
            temp_str = ''
            idx+=1
            
            while (data[idx] != 'z'):
                temp_str += data[idx]
                idx += 1
            py = float(temp_str)
            temp_str = ''
            idx+=1
            
            #alt
            while (data[idx] !='v'):
                temp_str += data[idx]
                idx += 1
            pz = 0.0 #float(temp_str)
            temp_str = ''
            idx+=2

            #vx
            while (data[idx] !='v'):
                temp_str += data[idx]
                idx += 1
            vx = float(temp_str)
            temp_str = ''
            idx+=2

            #vy
            while (data[idx] != 'v'):
                temp_str += data[idx]
                idx += 1
            vy = float(temp_str)
            temp_str = ''
            idx+= 2

            #vz
            while (idx < len(data)):
                temp_str += data[idx]
                idx += 1
            vz = float(temp_str)
            temp_str = ''
            idx+= 2

            #ix
            #while (data[idx] != 'i'):
            #    temp_str += data[idx]
            #    idx += 1
            ix = 0.0 #float(temp_str)
            #temp_str = ''
            #idx += 2

            #iy
            #while (data[idx] != 'i'):
            #    temp_str += data[idx]
            #    idx+= 1
            iy = 0.0 #float(temp_str)
            #temp_str = ''
            #idx += 2

            #iz
            #while(idx < len(data)):
            #    temp_str += data[idx]
            #    idx += 1
            iz = 0.0 # float(temp_str)

            return id_num, px, py, pz, vx, vy, vz, ix, iy, iz

    def store_remote_agent_data(self, data, time_received):
        id_num, px, py, pz, vx, vy, vz, ix, iy, iz = self.parse_remote_agent_location(data)

        #For simulation purposes, we exit out here if the data is our own
        #if (id_num == int(self.sys_id)):
        #    return None

        remote_agent_message = (id_num, data, time_received)

        if (id_num not in self.node_list):
            self.node_list.append(id_num)
            new_agent = Agent(id_num, [remote_agent_message])
            new_agent.x = px
            new_agent.y = py
            new_agent.z = pz
            new_agent.vx = vx
            new_agent.vy = vy
            new_agent.vz = vz
            new_agent.ix = ix
            new_agent.iy = iy
            new_agent.iz = iz
            new_agent.last_received = time_received #new_agent.current_received
            new_agent.current_received = time_received
            new_agent.time_between_locations = new_agent.current_received - new_agent.last_received
            self.agent_list.append(new_agent)
            print("Adding Agent " + str(new_agent.node_id) + ".")
        else:
            self.update_agent_location(id_num, px, py, pz, vx, vy, vz, ix, iy, iz, time_received, remote_agent_message)

    def update_agent_location(self, id_num, px, py, pz, vx, vy, vz, ix, iy, iz, time_received, remote_agent_message):
        for agent in self.agent_list:
            if (id_num == agent.node_id):
                agent.x = px
                agent.y = py
                agent.z = pz
                agent.vx = vx
                agent.vy = vy
                agent.vz = vz
                agent.ix = ix
                agent.iy = iy
                agent.iz = iz
                agent.last_received = agent.current_received
                agent.current_received = time_received
                agent.time_between_locations = agent.current_received - agent.last_received
                agent.responses.append(remote_agent_message)
                break

    #The net_modified_callback is triggered when the network is modified
    def net_modified_callback(self, event_type, reason, node):
        print("  >>> Network event:")
        print("              Type: %s (%d)" % (event_type.description, event_type.code))
        print("              Reason: %s (%d)" % (reason.description, reason.code))

        if not node:
            return

        print("              Node: %s" % node)


    def gps_callback(self, data):
        if (data.header.frame_id == self.sys_id):
            self.x = str(round(data.point.x,2))
            self.y = str(round(data.point.y,2))
            self.z = '0.0' #str(round(data.point.z,2))

    
    def vel_callback(self, data):
        self.vx = '0.0' # str(data.twist.linear.x)
        self.vy = '0.0' #str(data.twist.linear.y)
        self.vz = '0.0' #str(data.twist.linear.z)

    def radio_callback(self, xb_msg):
        #mimic the data_rcvd_callback for the radio here
        time_received = self.millis()
        #store the string from the topic 
        data = xb_msg.data

        #TEMPORARY DEBUG ONLY!!!
        #print(self.sys_id + 'rcvd: ' + data)

        if (data[0]=='C'):
            self.process_command(data[1])
        else:
            self.store_remote_agent_data(data, time_received)
            allAgents = allAgentsPosStamped()
            for agent in self.agent_list:
                a = agentPos()
                a.id = agent.node_id
                a.x = agent.x
                a.y = agent.y
                a.z = 0.0 #agent.z --AP setting to zero for bandwidth
                a.vx = agent.vx
                a.vy = agent.vy
                a.vz = agent.vz
                a.ix = agent.ix
                a.iy = agent.iy
                a.iz = agent.iz
                a.time_since_update = agent.time_between_locations
                allAgents.agents.append(a)

            #add time to header
            allAgents.header.stamp = rospy.Time.now()
            
            self.agent_pos_pub.publish(allAgents)

    def __init__(self):
        rospy.init_node('mesh_radio')
        
        # grab the system id as the first command line arg
        if (len(sys.argv) > 1):
            self.sys_id = sys.argv[1]
            print(f"Setting sys_id to {self.sys_id}.")

        if (len(sys.argv) > 2):
            if (sys.argv[2] == 'sim'):
                self.simming = True
        print(f"Simming is set to {self.simming}.")

        #initialize the actual radio hardware if not simming
        if (self.simming == False):
            radio_port = '/dev/ttyXBEE'
            radio_baud = 57600

        #self.position_pub = rospy.Publisher('position_info', String, queue_size=10)
        self.agent_pos_pub = rospy.Publisher('agent_pos', allAgentsPosStamped, queue_size = 10)

        rospy.Subscriber("agent_position_xy", PointStamped, self.gps_callback)
        print("Subscribing to mavros/local_position/velocity_local for velocity information.")
        rospy.Subscriber("mavros/local_position/velocity_local", TwistStamped, self.vel_callback)

        if (self.simming==True):
            print(f"Setting up sim publisher/subscriber for agent {self.sys_id} ...")
            self.radio_pub = rospy.Publisher('/radio_traffic', String, queue_size = 10)
            print("Publishing to radio_traffic.")
            rospy.Subscriber("/radio_traffic", String, self.radio_callback)
            print("Subscribed to radio_traffic.")

        if (self.simming == False):
            self.radio = XBeeDevice(radio_port, radio_baud)
            print("Opening radio...")
            self.radio.open()

            #Add radio data reception callback
            print("Activating serial data callback...")
            self.radio.add_data_received_callback(self.data_rcvd_callback)


            print("Polling network...")
            self.xnet = self.radio.get_network()
            self.xnet.start_discovery_process()
            while self.xnet.is_discovery_running():
                time.sleep(0.1)
            self.nodes = self.xnet.get_devices()
            print("Found the following radios:")
            for node in self.nodes:
                print(node.get_node_id())
    
            #Add network modification callback
            print("Activating network modification callback...")
            self.xnet.add_network_modified_callback(self.net_modified_callback)

    def main_loop(self):
        print("Swarm Comms node is running.")

        #restrict transmission of pos to 10 Hz
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            #change this to publish the gps coords at 10Hz.
            #Use the serial callback to update the swarm agent
            #data structure

            #configure string position info
            radio_str = 'a'+self.sys_id+':x'+self.x+'y'+self.y+'z'+self.z+'vx'+self.vx+'vy'+self.vy+'vz'+self.vz
            
            #store our own data here by calling self.store_remote_agent_data() and using radio_str
            time_received = self.millis()
            self.store_remote_agent_data(radio_str, time_received)

            #print("Sending: " + radio_str)
            #print(f"The length of radio_str is: {len(radio_str)}")
            if (self.simming == False):
                #broadcast on radio if not simming
                try:
                    self.radio.send_data_broadcast(radio_str)
                except TransmitException:
                    print("Data block too long.")
                #print(radio_str)
            else:
                #publish to topic
                s = String()
                s.data = radio_str;
                self.radio_pub.publish(s)

            #debug print
            #print(radio_str)

            #check to see if more than 5 seconds have passed on any one
            #agent's update.  If so, remove that agent from the list
            x = 0
            for agent in self.agent_list:
                if (self.millis() - agent.last_received > 5000):
                    print("Update delay > 5 seconds. Removing Agent " +str(agent.node_id) + ".")
                    #remove the agent from the list
                    self.agent_list.remove(agent)        
                    #remove node from list
                    self.node_list.remove(agent.node_id)

            r.sleep()

if __name__ == '__main__':
    r = TheRadio()
    r.main_loop()


