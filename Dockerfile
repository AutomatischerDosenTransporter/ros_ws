FROM osrf/ros:foxy-desktop
LABEL authors="Daniel Nussbaum"
SHELL ["/bin/bash", "-c"]

WORKDIR /ros_ws
COPY ./src ./src

RUN source /opt/ros/foxy/setup.bash \
RUN apt-get update -y \
RUN rosdep install --from-paths src --ignore-src --rosdistro foxy -y \
RUN colcon build --symlink-install