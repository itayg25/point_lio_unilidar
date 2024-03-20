
FROM ros:noetic

# Avoid prompts from apt by setting this environment variable
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies necessary for building and running ROS packages
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    git \
    ros-noetic-tf \
    ros-noetic-pcl-ros \
    ros-noetic-eigen-conversions \
    python3-catkin-tools \
    python3-osrf-pycommon \
    && rm -rf /var/lib/apt/lists/*

# Set a default locale
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Update rosdep
RUN rosdep update

# Create and initialize the catkin workspace
WORKDIR /catkin_ws
RUN mkdir src

# Install any missing dependencies for the ROS packages
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /catkin_ws; rosdep install --from-paths src --ignore-src -r -y'

RUN apt-get update

RUN apt-get install ros-noetic-rviz -y

RUN git clone https://github.com/unitreerobotics/unilidar_sdk.git

WORKDIR /catkin_ws/unilidar_sdk/unitree_lidar_ros

# Run your commands in a single RUN statement to ensure they are executed in the same shell
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash && \
                  catkin_make'


RUN mkdir -p /catkin_ws/unilidar_sdk/unitree_lidar_ros/atkin_point_lio_unilidar/src

WORKDIR /catkin_ws/unilidar_sdk/unitree_lidar_ros/atkin_point_lio_unilidar/src

COPY . .

WORKDIR /catkin_ws/unilidar_sdk/unitree_lidar_ros/atkin_point_lio_unilidar

RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash && \
                  catkin_make'

COPY run-unilidar-map.sh .

RUN chmod +x ./run-unilidar-map.sh


# Reset DEBIAN_FRONTEND for interactive use within the container
ENV DEBIAN_FRONTEND=

# Keep the container running
CMD ["sudo", "./run-unilidar-map.sh"]