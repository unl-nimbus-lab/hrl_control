# syntax=docker/dockerfile:1
FROM ubuntu:20.04

# Setup timezone
RUN echo 'Etc/UTC' > /etc/timezone \
    && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
    && apt-get update \
    && apt-get install -q -y --no-install-recommends tzdata \
    && rm -rf /var/lib/apt/lists/*

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

# Install some helpful utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
        iputils-ping \
        net-tools \
        netbase \
        python3 \
        python3-pip \
        git \
        nano \
        vim \
        git \
        clang \
        g++ \
        rsync \
        zip \
        make \
        cmake \
        curl \
        wget \
        libopencv-dev \
        python3-opencv \
        build-essential \
        lsb-release \
        ca-certificates \
        dirmngr \
        gnupg2 \
        python3 \
        python3-pip \
        python3-dev \
        netbase \
        htop \
        nmap \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install \
    numpy \
    pandas \
    scipy \
    matplotlib \
    pyserial \
    pymavlink

# Add the ROS repo to the sources list
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Add the ROS key
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install ROS Noetic
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-rosdep \
        python3-rosinstall \
        python3-vcstools \
        ros-noetic-ros-base \
    && rosdep init \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# Install ROS dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-noetic-tf \
        ros-noetic-diagnostic-updater \
        ros-noetic-control-toolbox \
    && rm -rf /var/lib/apt/lists/*

# Install MAVROS dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-lxml \
        python3-catkin-tools \
        python3-rosinstall-generator \
        geographiclib-tools \
        libgeographic-dev \
	&& geographiclib-get-geoids egm96-5 \
    && geographiclib-get-gravity egm96 \
    && geographiclib-get-magnetic emm2015 \
    && rm -rf /var/lib/apt/lists/

# Install Python packages
RUN pip3 install \
    pyserial \
    pymavlink

# Install ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-control-toolbox

# Initialize the ROS workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd ${ROS_WS} \
    && rosdep update \
    && catkin init \
    && wstool init src

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
    && catkin build -j4

# Overlay the ROS environment on top of the existing environment
WORKDIR ${HOME}
COPY docker/development/ros_entrypoint.sh /
RUN ["chmod", "+x", "/ros_entrypoint.sh"]
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]