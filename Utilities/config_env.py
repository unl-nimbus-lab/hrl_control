#!/usr/bin/env python3

# Example output
# PORT=/dev/ttyS0
# BAUD=921600
# SYS_ID=5 # unique agent ID
# COMP_ID=1 # always 1
# CLUSTER_ID=1 # which cluster it is
# CLUSTER_POSITION=1 # position 1-CLUSTER_SIZE, ordered in clockwise order
# CLUSTER_SIZE=2 # number of agents in cluster
# CLUSTER_RADIUS=10 # distance (m) from center of circle
# AGENT_ALT=5.0 # separation (m)
# HOME_LAT=40.8467784 # absolute
# HOME_LON=-96.4719180
# HOME_ALT=400 
# RALLY1LAT=0 # relative
# RALLY1LON=30
# RALLY2LAT=30 # relative
# RALLY2LON=0

# For writing all the configuration files
import sys
import os
import subprocess
import pkg_resources

# download the required packages if not installed
required = {'paramiko', 'scp'}
installed = {pkg.key for pkg in pkg_resources.working_set}
missing = required - installed
if missing:
    python = sys.executable
    subprocess.check_call([python, '-m', 'pip', 'install', *missing], stdout=subprocess.DEVNULL)
# now import the right modules
from paramiko import SSHClient
from scp import SCPClient

######################################################
# Change these global vars as needed
######################################################
to_stdout = 0 # 0 for printing to file and upload to each agent (assumes agents are on and connected on network)
# parameters for where to put the .env file on the agents
path_to_env = "arl-swarm/docker/drone"
ag_username = "ubuntu"

# globals
PORT = "/dev/ttyUSB0"
BAUD = "921600"
COMP_ID = "1" # always 1
# Roger Memorial Farm
HOME_LAT = "40.8467784" # absolute
HOME_LON = "-96.4719180"
HOME_ALT = "400"  # (m)
RALLY1LAT = "0" # relative (m)
RALLY1LON = "30" # relative (m)
RALLY2LAT = "30" # relative (m)
RALLY2LON = "0" # relative (m)

# changed below according to input, but not on a per cluster basis
CLUSTER_RADIUS = "10" # (m)

        
######################################################
# Classes
######################################################

class Agent:

    # Class variables that will change
    cluster_ID = "0"
    cluster_size = "0"
    sys_id = "0" # unique agent ID (will change below)
    cluster_pos = "0" # position 1-CLUSTER_SIZE, ordered in clockwise order
    agent_alt = 0
    
    # constructor
    def __init__(self, ag_id, ag_alt, cl_id, cl_size, cl_pos):
        self.sys_id = ag_id
        self.agent_alt = round(ag_alt, 1)
        self.cluster_ID = cl_id
        self.cluster_size = cl_size
        self.cluster_pos = cl_pos
        
    def print(self):
        print("PORT="+PORT)
        print("BAUD="+BAUD)
        print("PORT2=/dev/ttyUSB1")
        print("SYS_ID="+str(self.sys_id))
        print("COMP_ID=1") # always 1
        print("CLUSTER_ID="+str(self.cluster_ID)) # which cluster it is
        print("CLUSTER_POSITION="+str(self.cluster_pos)) # position 1-CLUSTER_SIZE, ordered in clockwise order
        print("CLUSTER_SIZE="+str(self.cluster_size)) # number of agents in cluster
        print("CLUSTER_RADIUS="+str(CLUSTER_RADIUS)) # distance (m) from center of circle
        print("AGENT_ALT="+str(self.agent_alt)) # separation (m)
        print("HOME_LAT="+HOME_LAT) # absolute
        print("HOME_LON="+HOME_LON)
        print("HOME_ALT="+HOME_ALT)
        print("RALLY1LAT="+RALLY1LAT) # relative
        print("RALLY1LON="+RALLY1LON)
        print("RALLY2LAT="+RALLY2LAT) # relative
        print("RALLY2LON="+RALLY2LON)

######################################################
# Main code
######################################################
       
cluster_agent = input("Agents in cluster form (e.g., [1 2 6][3 5 4])? ")
CLUSTER_RADIUS = input("Cluster radius (m)? ")
alt_sep = float(input("Desired altitude separation (m)? "))
alt_base = float(input("Desired base altitude (m) for lowest agent? "))


# Process data
# now split into clusters
clusters_str = cluster_agent.split(']')
# remove empty entry in list from split command
clusters_str.pop()
agents = []
# create the agents
for c_idx, c in enumerate(clusters_str):
    # remove "[" and any stray "]"
    agents_str = c.replace('[', '')
    agents_str = agents_str.replace(']', '')
    agents_str = agents_str.strip() # remove whitespace and end and beg
    agents_list = agents_str.split() # split on whitespace
            
    clust_idx = c_idx + 1 # can't have a 0 cluster
    for a_idx, a in enumerate(agents_list):
            ag_alt = alt_base + a_idx * alt_sep # altitude positions
            clust_pos = a_idx + 1 # starts at 1
            agents.append(Agent(a, ag_alt, clust_idx, len(agents_list), clust_pos))


# Just print to stdout
if to_stdout==1:
    for agent in agents:
        agent.print()
        print("\n")
else:     # Print to file, upload to each agent
    # Setup the ssh connection and SCP the file over to the right agent
    ssh = SSHClient()
    ssh.load_system_host_keys()
    for agent in agents:
        with open('.env', 'w') as f:
            #print('Environment variables will be written to a file.')
            sys.stdout = f # Change the standard output to the file we created.
            agent.print()
            # sys.stdout = original_stdout # Reset the standard output to its original value
            f.close()
            # now SCP this over to the agent in question
            ag_hostname = "agent-"+str(agent.sys_id)
            ssh.connect(ag_hostname, username=ag_username) # port 22
            # SCPCLient takes a paramiko transport as an argument
            scp = SCPClient(ssh.get_transport())
            scp.put('.env', remote_path=path_to_env)
            # remove the file
            os.remove('.env')
