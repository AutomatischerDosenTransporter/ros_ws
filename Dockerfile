########################################
# Base Image for ADT                   #
########################################
FROM ros:foxy as build_base_part
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO=${ROS_DISTRO}
LABEL storage="github_action_prune"

MAINTAINER "Daniel Nussbaum"

# Install dependencies
RUN rm /etc/apt/sources.list.d/ros2-snapshots.list
RUN apt-get update -y
#RUN RUN apt-get install -y python3-opencv
RUN apt-get install -y python3-pip ros-humble-ros2-control ros-humble-ros2-controllers
RUN pip install pyserial
FROM build_base_part as build_dependencies_part

# Create Colcon workspace with external dependencies
RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws
COPY ./src /ros_ws/src
FROM build_dependencies_part as build_source_part

# Build the base Colcon workspace, installing dependencies first.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash
RUN apt-get update -y
RUN colcon build --symlink-install
