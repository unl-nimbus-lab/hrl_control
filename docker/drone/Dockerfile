# syntax=docker/dockerfile:1
FROM ros:noetic-robot

# Create the image file structure
RUN mkdir /home/catkin_ws \
    && mkdir /home/catkin_ws/src \
    && mkdir /home/rosbags

# Setup the environments
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=noetic
ENV HOME=/home/
ENV ROS_WS=/home/catkin_ws/
WORKDIR ${ROS_WS}

# Install utilities/dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        iputils-ping \
        net-tools \
        netbase \
        python3 \
        python3-pip \
        git \
        nano \
        vim \
        htop \
        python3-lxml \
        python3-catkin-tools \
        python3-rosinstall-generator \
        geographiclib-tools \
        libgeographic-dev \
	&& geographiclib-get-geoids egm96-5 \
    && geographiclib-get-gravity egm96 \
    && geographiclib-get-magnetic emm2015 \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install \
    pyserial \
    pymavlink

# Install ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-control-toolbox

# Build the ROS workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd ${ROS_WS} \
    && rosdep update \
    && catkin init \
    && wstool init src

COPY Utilities/xbee-python-master /home/xbee-python-master
RUN pip3 install /home/xbee-python-master

# Install MAVROS
# Note that we perform this step as an independent layer to prevent
# requiring internet to rebuild swarm nodes
COPY swarm-nodes/mavros /home/catkin_ws/src/mavros

# Install MAVLink and configure workspace
RUN rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall \
    && wstool merge -t src /tmp/mavros.rosinstall \
    && wstool update -t src -j4 \
    && rosdep install --from-paths src --ignore-src -y

# We perform a build here to avoid triggering the ungodly long
# mavros builds when code changes are made in the field
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \ 
    && catkin build

# Copy the swarm ROS nodes into the image
COPY swarm-nodes/custom_libs /home/catkin_ws/src/custom_libs
# COPY swarm-nodes/estimator /home/catkin_ws/src/estimator
# COPY swarm-nodes/hrl_lqi /home/catkin_ws/src/hrl_lqi
COPY swarm-nodes/swarm_msgs /home/catkin_ws/src/swarm_msgs
COPY swarm-nodes/swarm_comms /home/catkin_ws/src/swarm_comms
COPY swarm-nodes/clustering_control /home/catkin_ws/src/clustering_control
COPY swarm-nodes/observer /home/catkin_ws/src/observer
# COPY swarm-nodes/live_gcs /home/catkin_ws/src/live_gcs

# Build the catkin workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && catkin build -j1

RUN chmod +x /home/catkin_ws/src/swarm_comms/scripts/swarm_comm.py

# Rosbag management
COPY docker/drone/bag_manager.sh /home/
RUN ["chmod","+x","/home/bag_manager.sh"]

# Overlay the ROS environment on top of the existing environment
COPY ../../docker/drone/ros_entrypoint.sh /
RUN ["chmod", "+x", "/ros_entrypoint.sh"]
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
