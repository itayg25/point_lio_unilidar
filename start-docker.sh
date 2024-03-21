#!/bin/bash

# Default Docker image name
DOCKER_IMAGE_NAME="ros:noetic-unilidar"
RUN_IN_BACKGROUND=0

# Parse command-line options
while getopts ":i:dh" opt; do
  case ${opt} in
    i )
      DOCKER_IMAGE_NAME=$OPTARG
      ;;
    d )
      RUN_IN_BACKGROUND=1
      ;;
    h )
      echo "Usage:"
      echo "    -i  Specify the Docker image name (default is ros:noetic-unilidar)"
      echo "    -d  Run the Docker container in the background"
      echo "    Any other flags and arguments will be passed directly to the docker run command"
      exit 0
      ;;
    \? )
      echo "Invalid option: $OPTARG" 1>&2
      ;;
    : )
      echo "Invalid option: $OPTARG requires an argument" 1>&2
      ;;
  esac
done

shift $((OPTIND -1))

# Prepare X11 forwarding for GUI
XSOCK="/tmp/.X11-unix"
XAUTH="/tmp/.docker.xauth"

# Create a new .Xauth file if it doesn't exist
if [ ! -f $XAUTH ]; then
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
fi

# Generate a timestamp and create a directory for it
TIMESTAMP_DIR=$(date +"%Y-%m-%d-%H-%M-%S")
mkdir -p "./scans/$TIMESTAMP_DIR"

echo "Running Docker container $DOCKER_IMAGE_NAME with arguments $@..."

# Running the Docker container with USB access, GUI capability, and the newly created volume
DOCKER_RUN_CMD="sudo docker run"

if [ $RUN_IN_BACKGROUND -eq 1 ]; then
    DOCKER_RUN_CMD+=" -d -it --rm"
else
    DOCKER_RUN_CMD+=" -it --rm"
fi

DOCKER_RUN_CMD+=" --name=ros_unilidar_container \
  --volume=$XSOCK:$XSOCK:rw \
  --volume=$XAUTH:$XAUTH:rw \
  --volume=$(pwd)/scans/$TIMESTAMP_DIR:/pcd \
  --env=\"XAUTHORITY=${XAUTH}\" \
  --env=\"DISPLAY\" \
  --privileged \
  --net=host \
  --device=/dev/ttyUSB0 \
  $DOCKER_IMAGE_NAME"

# Append all other passed arguments
DOCKER_RUN_CMD+=" $@"

# Execute the Docker run command
eval $DOCKER_RUN_CMD
