FROM ros:foxy AS step1
LABEL authors="Daniel Nussbaum"
SHELL ["/bin/bash", "-c"]

RUN rm /etc/apt/sources.list.d/ros2-snapshots.list
RUN apt-get update -y
RUN apt-get install python3-colcon-common-extensions
RUN apt-get install -y python3-pip
RUN pip3 install setuptools==58.2.0


FROM step1 AS step2
RUN git clone https://github.com/babakhani/rplidar_ros2.git /ros_ws/src/rplidar_ros

FROM step2 AS step3
RUN pip install pyserial

FROM step3 AS step4
WORKDIR /ros_ws
COPY ./src ./src

RUN source /opt/ros/foxy/setup.bash
RUN rosdep install --from-paths src --ignore-src --rosdistro foxy -y
RUN colcon build --symlink-install; exit 0
RUN colcon build --symlink-install
RUN echo '~/ros_ws/install/setup.bash' >> ~/.bashrc