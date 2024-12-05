#!/bin/bash

####################################################################
# Start script used to launch the drone image interactively        #
# with options to launch with host networking and a bind mount     #
####################################################################

help()
{
    # Display the help menu
    echo "The start.sh script is a helper script used to \
    launch the drone image interactively with common development \
    parameters"
    echo 
    echo "Syntax: start [-h|p|e|s path/to/source|t path/to/target|v /path/to/device]"
    echo "Options:"
    echo "h     Print this help message"
    echo "p     Pull the latest version of the drone image prior to use"
    echo "e     Expose the local ports by using host networking"
    echo "s     Full path to the source directory that should be used if using a bind mount"
    echo "t     Full path to the target directory that should be used if using a bind mount"
    echo "v     Full path to the device to expose"
    echo
}


pull_latest_version()
{
    docker pull ghcr.io/unl-nimbus-lab/arl-swarm/drone:latest
}


run()
{
    docker run -it "$@" ghcr.io/unl-nimbus-lab/arl-swarm/drone:latest
}


# Flags
SOURCE_INCLUDED=false
TARGET_INCLUDED=false

# Docker argument values
HOST_NET=""
TARGET=""
SOURCE=""
BIND_MOUNT=""
DEVICE=""


while getopts ":hpes:t:v:" opt; do
    case "${opt}" in
        h)
            help
            exit
            ;;
        p)
            echo "[INFO] Pulling latest drone image version"
            pull_latest_version
            ;;
        e)
            HOST_NET="--net=host"
            ;;
        s)
            SOURCE_INCLUDED=true
            SOURCE=$OPTARG
            ;;
        t)
            TARGET_INCLUDED=true
            TARGET=$OPTARG
            ;;
        v)
            DEVICE="--device=${OPTARG}"
            ;;
        \?)
            echo "[ERROR] Invalid flag provided. Exiting."
            ;;
    esac
done

if [[ $SOURCE_INCLUDED == false && $TARGET_INCLUDED == true ]] || [[ $TARGET_INCLUDED == false && $SOURCE_INCLUDED == true ]]; then
    echo "[ERROR] The source and target flags must be used in conjunction to create a bind mount"
    exit 1
elif [[ $SOURCE_INCLUDED == true && $TARGET_INCLUDED == true ]]; then
    BIND_MOUNT="--mount type=bind,source=${SOURCE},target=${TARGET}"
fi


run $HOST_NET $DEVICE $BIND_MOUNT