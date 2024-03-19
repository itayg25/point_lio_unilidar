#!/bin/bash

# Name of your docker image
DOCKER_IMAGE_NAME="ros:noetic-unilidar"

# Default docker build options
BUILD_OPTS=""

# Check for --no-cache flag in the script arguments
for arg in "$@"
do
    case $arg in
        --no-cache)
        BUILD_OPTS="$BUILD_OPTS --no-cache"
        shift # Remove --no-cache from processing
        ;;
    esac
done

# Always build or update the Docker image
echo "Building or updating Docker image..."
docker build $BUILD_OPTS -t $DOCKER_IMAGE_NAME .

echo "Docker image $DOCKER_IMAGE_NAME built or updated successfully."
