FROM osrf/ros:foxy-desktop AS stage1
LABEL authors="Daniel Nussbaum"
SHELL ["/bin/bash", "-c"]

WORKDIR /ros_ws
COPY ./src ./src

FROM stage1 AS stage2
LABEL authors="Daniel Nussbaum"
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/foxy/setup.bash

FROM stage2 AS stage3
LABEL authors="Daniel Nussbaum"
SHELL ["/bin/bash", "-c"]
RUN apt-get update -y

FROM stage3 AS stage4
LABEL authors="Daniel Nussbaum"
SHELL ["/bin/bash", "-c"]
RUN rosdep install --from-paths src --ignore-src --rosdistro foxy -y

FROM stage4 AS stage5
LABEL authors="Daniel Nussbaum"
SHELL ["/bin/bash", "-c"]
RUN colcon build --symlink-install

FROM stage5
LABEL authors="Daniel Nussbaum"
SHELL ["/bin/bash", "-c"]
RUN source ./install/setup.bash