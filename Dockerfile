FROM osrf/ros:foxy-desktop
LABEL authors="Daniel Nussbaum"

WORKDIR /ros_ws
COPY ./src ./src

ENTRYPOINT ["colcon","build"]