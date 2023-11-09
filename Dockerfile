FROM osrf/ros:foxy-desktop
LABEL authors="Daniel Nussbaum"
SHELL ["/bin/bash", "-c"]

WORKDIR /ros_ws
COPY ./src ./src

RUN source /opt/ros/foxy/setup.bash \
    && apt-get update -y \
    && rosdep install --from-paths src --ignore-src --rosdistro foxy -y \
    && colcon build --symlink-install \
    && source ./install/setup.bash