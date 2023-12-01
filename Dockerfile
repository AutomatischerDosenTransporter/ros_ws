FROM ros:foxy AS step1
LABEL authors="Daniel Nussbaum"
SHELL ["/bin/bash", "-c"]

RUN rm /etc/apt/sources.list.d/ros2-snapshots.list
RUN apt-get update -y
RUN apt-get install python3-colcon-common-extensions
RUN apt-get install -y python3-pip
RUN pip3 install setuptools==58.2.0

FROM step1 AS step2

FROM step2 AS step3
RUN pip install pyserial

FROM step3 AS step4
WORKDIR /ros_ws
COPY ./src /ros_ws/src

RUN source /opt/ros/foxy/setup.bash
RUN rosdep install --from-paths src --ignore-src --rosdistro foxy -y
RUN colcon build --symlink-install
RUN echo 'source /ros_ws/install/setup.bash' >> ~/.bashrc

CMD ["/bin/bash", ""]