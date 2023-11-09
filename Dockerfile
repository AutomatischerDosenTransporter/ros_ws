FROM osrf/ros:foxy-desktop AS stage1
LABEL authors="Daniel Nussbaum"
SHELL ["/bin/bash", "-c"]

WORKDIR /ros_ws
COPY ./src ./src

FROM stage1 AS stage2
RUN source /opt/ros/foxy/setup.bash \

FROM stage2 AS stage3
RUN apt-get update -y \

FROM stage3 AS stage4
RUN rosdep install --from-paths src --ignore-src --rosdistro foxy -y \

FROM stage4 AS stage5
RUN colcon build --symlink-install \

FROM stage5
RUN source ./install/setup.bash