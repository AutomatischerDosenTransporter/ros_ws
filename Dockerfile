FROM osrf/ros:foxy-desktop
LABEL authors="Daniel Nussbaum"
SHELL ["/bin/bash", "-c"]

WORKDIR /ros_ws
COPY ./src ./src

RUN source /opt/ros/foxy/setup.bash
RUN apt-get update -y
RUN apt-get add-apt-repository universe
RUN apt-get install python3-colcon-common-extensions
RUN apt-get install -y python3-pip

RUN pip3 install setuptools==58.2.0
RUN pip install pyserial

RUN rosdep install --from-paths src --ignore-src --rosdistro foxy -y
RUN colcon build --symlink-install
RUN echo '~/ros_ws/install/setup.bash' >> ~/.bashrc