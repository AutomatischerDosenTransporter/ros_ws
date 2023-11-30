FROM osrf/ros:foxy-desktop
LABEL authors="Daniel Nussbaum"
SHELL ["/bin/bash", "-c"]

WORKDIR /ros_ws
COPY ./src ./src

RUN apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get clean -y
RUN apt-get update -y
RUN apt-get add-apt-repository universe
RUN apt-get install python3-colcon-common-extensions
RUN apt-get install -y python3-pip

RUN pip3 install setuptools==58.2.0
RUN pip install pyserial

RUN source /opt/ros/foxy/setup.bash
RUN rosdep install --from-paths src --ignore-src --rosdistro foxy -y
RUN colcon build --symlink-install
RUN echo '~/ros_ws/install/setup.bash' >> ~/.bashrc