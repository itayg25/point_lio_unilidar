#!/bin/bash

# Name of your docker image
DOCKER_IMAGE_NAME="ros:noetic-unilidar"

# Prepare X11 forwarding for GUI
XSOCK="/tmp/.X11-unix"
XAUTH="/tmp/.docker.xauth"

# Create a new .Xauth file if it doesn't exist
if [ ! -f $XAUTH ]; then
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
fi

echo "Running Docker container $DOCKER_IMAGE_NAME with arguments $@..."

# Running the Docker container with USB access and GUI capability
sudo docker run -it --rm \
  --name=ros_unilidar_container \
  --volume=$XSOCK:$XSOCK:rw \
  --volume=$XAUTH:$XAUTH:rw \
  --env="XAUTHORITY=${XAUTH}" \
  --env="DISPLAY" \
  --privileged \
  --net=host \
  --device=/dev/ttyUSB0 \
  $DOCKER_IMAGE_NAME $@

# Note: The '--device=/dev/ttyUSB0' flag grants the container access to the USB device.
# If your device has a different name, replace '/dev/ttyUSB0' with the correct device name.
# Using '--privileged' is required for full access to USB devices but use with caution.
