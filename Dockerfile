# Group image (ROS 2 Jazzy).
# When the teaching staff's base image is published in the common repository, just swap BASE_IMAGE:
#   docker compose build --build-arg BASE_IMAGE=<base-image-from-the-common-repo>
ARG BASE_IMAGE=ros:jazzy-ros-base
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-c"]

# Everything the app needs is installed here. If it is not in the Dockerfile, it does not exist.
# The group's sensor driver goes in this list (e.g. ros-jazzy-velodyne, ros-jazzy-usb-cam).
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-rosbag2-storage-mcap \
        ros-jazzy-foxglove-bridge \
        ros-jazzy-rviz2 \
        python3-numpy \
        python3-pytest \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 workspace: the shared package (va_planeamento) and the group's package (va_gNN)
WORKDIR /ws
COPY src/ src/
RUN source /opt/ros/jazzy/setup.bash && colcon build

# Entrypoint: loads ROS 2 and the workspace before any command
RUN printf '%s\n' '#!/bin/bash' 'set -e' \
        'source /opt/ros/jazzy/setup.bash' 'source /ws/install/setup.bash' 'exec "$@"' \
        > /entrypoint.sh && chmod +x /entrypoint.sh \
    && echo 'source /ws/install/setup.bash' >> /root/.bashrc
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
