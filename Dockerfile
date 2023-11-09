FROM osrf/ros:foxy-desktop
LABEL authors="Daniel Nussbaum"

WORKDIR /ros_ws
COPY ./src ./src

RUN sudo apt install python3-serial
RUN colcon build
RUN source install/setup.bash