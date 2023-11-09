FROM osrf/ros:foxy-desktop
LABEL authors="Daniel Nussbaum"

WORKDIR /ros_ws
COPY ./src ./src

#RUN rosdep update
#RUN rosdep install --from-paths src -y
RUN colcon build
RUN source install/setup.bash