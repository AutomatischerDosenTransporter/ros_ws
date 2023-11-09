FROM osrf/ros:foxy-desktop
LABEL authors="Daniel Nussbaum"

WORKDIR /ros_ws
COPY ./src ./src

RUN apt install python3-pip
RUN pip3 install setuptools==58.2.0
RUN apt install python3-serial
RUN colcon build
RUN source install/setup.bash