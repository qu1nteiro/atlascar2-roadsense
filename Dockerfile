# Imagem do grupo (ROS 2 Jazzy).
# Quando a imagem base dos docentes estiver publicada no repositório comum, basta trocar o BASE_IMAGE:
#   docker compose build --build-arg BASE_IMAGE=<imagem-base-do-repositorio-comum>
ARG BASE_IMAGE=ros:jazzy-ros-base
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-c"]

# Tudo o que a app precisa é instalado aqui. Se não está no Dockerfile, não existe.
# O driver do sensor do grupo entra nesta lista (p. ex. ros-jazzy-velodyne, ros-jazzy-usb-cam).
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-rosbag2-storage-mcap \
        ros-jazzy-foxglove-bridge \
        ros-jazzy-rviz2 \
        python3-numpy \
        python3-pytest \
    && rm -rf /var/lib/apt/lists/*

# Workspace ROS 2: o pacote comum (va_planeamento) e o pacote do grupo (va_gNN)
WORKDIR /ws
COPY src/ src/
RUN source /opt/ros/jazzy/setup.bash && colcon build

# Entrypoint: carrega o ROS 2 e o workspace antes de qualquer comando
RUN printf '%s\n' '#!/bin/bash' 'set -e' \
        'source /opt/ros/jazzy/setup.bash' 'source /ws/install/setup.bash' 'exec "$@"' \
        > /entrypoint.sh && chmod +x /entrypoint.sh \
    && echo 'source /ws/install/setup.bash' >> /root/.bashrc
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
