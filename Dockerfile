########################################
# Base Image for ADT                   #
########################################
FROM ros:humble as build_base_part
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO=${ROS_DISTRO}
LABEL storage="github_action_prune"

LABEL MAINTAINER="Daniel Nussbaum"

# Install dependencies
COPY init.bash .
RUN bash init.bash
FROM build_base_part as build_dependencies_part

# Create Colcon workspace with external dependencies
RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws
COPY ./src /ros_ws/src
FROM build_dependencies_part as build_source_part

# Build the base Colcon workspace, installing dependencies first.
COPY build.bash .
RUN bash build.bash


COPY setup.sh /ros_ws/setup.sh
RUN chmod 777 -R .
ENTRYPOINT [ "/ros_ws/setup.sh" ]