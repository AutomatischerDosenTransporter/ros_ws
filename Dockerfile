FROM osrf/ros:foxy-desktop AS step1
LABEL authors="Daniel Nussbaum"
SHELL ["/bin/bash", "-c"]

RUN apt-get install python3-colcon-common-extensions
RUN apt-get install -y python3-pip
RUN pip3 install setuptools==58.2.0

FROM step1 AS step2
RUN pip install pyserial

FROM step2 AS step3
WORKDIR /ros_ws
COPY ./src ./src

RUN source /opt/ros/foxy/setup.bash
RUN rosdep install --from-paths src --ignore-src --rosdistro foxy -y
RUN colcon build --symlink-install
RUN echo '~/ros_ws/install/setup.bash' >> ~/.bashrc