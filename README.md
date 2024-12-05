# hrl_control

## Introduction
This repository contains the implementation, tools, and processes that have been used by the ARL Swarm Team for control of UAV swarms.

The core control nodes are:

* `swarm-nodes/hrl_control`
* `swarm-nodes/state_provider`
* `swarm-nodes/estimator`

## Installation
Prior to installing this project, it is necessary to first install the software requirements listed below. Note that these requirements must also be met on the respective agents.

* [Docker](https://docs.docker.com/get-docker/)
* [docker-compose](https://docs.docker.com/compose/install/)

Once these software requirements have been met, it is now necessary to clone the project repository. Performing this step will provide access to the project's `docker-compose.yml` scripts and helper scripts.

```
git clone https://github.com/unl-nimbus-lab/hrl_control.git
```

After cloning the project repository, it is now possible to install and deploy the Docker images provided within this project. The provided Docker images contain all code and the respective environments so that Users may continue to develop and deploy this project.


## Getting Started
The `hrl_control` reposiotry is designed to be developed, tested and deployed using Docker containers. The `development` conatiner is to be used by mounting the `swarm-nodes` directory as a volume a runtime. This allows developers to re-compile code without haveing to rebuild the container. The `testing` container is to be used for simulation, and `deployment` container is to be run on the drone's companion computer (the only difference between the two is the base image)

### Launching an `hrl_control/development` Container

```bash
# Navigate to the correct directory
cd hrl_control/docker/development

# Build the image
docker build -t swarm_devel .

docker run -it --net=host \
 --mount type=bind,source="${HOME}/hrl_control/swarm-nodes/custom_libs",target=/home/catkin_ws/src/custom_libs/  \
 --mount type=bind,source="${HOME}/hrl_control/swarm-nodes/swarm_msgs",target=/home/catkin_ws/src/swarm_msgs/ \
 --mount type=bind,source="${HOME}/hrl_control/swarm-nodes/command_xform",target=/home/catkin_ws/src/command_xform/ \
 --mount type=bind,source="${HOME}/hrl_control/swarm-nodes/state_provider",target=/home/catkin_ws/src/state_provider/ \
 --mount type=bind,source="${HOME}/hrl_control/swarm-nodes/lqr_control",target=/home/catkin_ws/src/lqr_control/ \
 --mount type=bind,source="${HOME}/hrl_control/swarm-nodes/hrl_command_interpret",target=/home/catkin_ws/src/hrl_command_interpret/ \
 --mount type=bind,source="${HOME}/hrl_control/swarm-nodes/hrl_control",target=/home/catkin_ws/src/hrl_control/ \
 --mount type=bind,source="${HOME}/hrl_control/swarm-nodes/bag_handler",target=/home/catkin_ws/src/bag_handler/ \
 swarm_devel

```


### Launching an `hrl_control/drone` Container
Prior to launching the `hrl_control/drone` container, it is first necessary to modify the environment variables provided within the `.env` file and to source this environment file:

```bash
# Navigate to the proper directory
cd path/to/hrl_control

# build the image
docker build -f DroneDockerfile -t hrl_docker_drone
```

After setting the desired environment variable values and sourcing the environment, the `docker-compose.yml` script may be used to launch the container.

```bash
# Navigate to the proper directory
cd path/to/hrl_control/docker/drone/

# Run the docker services
docker-compose up
```