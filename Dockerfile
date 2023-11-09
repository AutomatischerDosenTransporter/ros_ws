FROM osrf/ros:foxy-desktop
LABEL authors="Daniel Nussbaum"

WORKDIR /app
COPY ../ros_ws /app

ENTRYPOINT ["/bin/bash", "./ros_ws/custom_install.bash"]