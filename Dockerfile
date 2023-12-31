ARG ROS_DISTRO=foxy

########################################
# Base Image for TurtleBot3 Simulation #
########################################
FROM osrf/ros:${ROS_DISTRO}-desktop as build_base_part
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

MAINTAINER "Daniel Nussbaum"

# Install dependencies
RUN rm /etc/apt/sources.list.d/ros2-snapshots.list
RUN apt-get update -y
RUN apt-get install -y python3-pip python3-opencv
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
RUN rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
RUN colcon build --symlink-install