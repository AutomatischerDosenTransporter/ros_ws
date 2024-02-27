########################################
# Base Image for ADT                   #
########################################
FROM ros:foxy as build_base_part
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO=${ROS_DISTRO}
LABEL storage="github_action_prune"

MAINTAINER "Daniel Nussbaum"

# Install dependencies
RUN bash setup.bash
FROM build_base_part as build_dependencies_part

# Create Colcon workspace with external dependencies
RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws
COPY ./src /ros_ws/src
FROM build_dependencies_part as build_source_part

# Build the base Colcon workspace, installing dependencies first.
RUN bash build.bash