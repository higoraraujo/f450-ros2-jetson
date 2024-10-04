# Usar uma imagem base do ROS2 Foxy com Docker
FROM ros:foxy-ros-core

# Instalar pacotes básicos e ferramentas de dependências
RUN apt-get update && apt-get install -y \
    python3-pip \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    libopencv-dev \
    ros-foxy-cv-bridge \
    ros-foxy-message-filters \
    ros-foxy-nav2-bringup \
    ros-foxy-nmea-navsat-driver \
    ros-foxy-um7 \
    && rm -rf /var/lib/apt/lists/*

# Clonar e instalar PX4 ROS2 bridge
RUN git clone https://github.com/PX4/px4_ros_com.git ~/ros2_ws/src/px4_ros_com \
    && git clone https://github.com/PX4/px4_msgs ~/ros2_ws/src/px4_msgs \
    && cd ~/ros2_ws/src/px4_ros_com/scripts && ./build_ros2_workspace.bash

# Instalar o SDK da ZED
RUN wget https://stereolabs.sfo2.digitaloceanspaces.com/zedsdk/3.5/jetson_jp45/zedsdk_3.5.0_jp45_jetson.run \
    && chmod +x zedsdk_3.5.0_jp45_jetson.run \
    && ./zedsdk_3.5.0_jp45_jetson.run --quiet --accept-terms

# Instalar o ROS wrapper da ZED
RUN git clone https://github.com/stereolabs/zed-ros2-wrapper.git ~/ros2_ws/src/zed-ros2-wrapper \
    && cd ~/ros2_ws && colcon build

# Definir o ambiente de trabalho
WORKDIR /root/ros2_ws

# Fonte do ROS2 ao iniciar o contêiner
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/foxy/setup.bash && source install/setup.bash && bash"]


B. Construir a Imagem Docker
Com o Dockerfile pronto, você pode construir a imagem Docker que conterá todo o ambiente ROS2 com suporte ao PX4, ZED, IMU, e RTK.

No diretório onde o Dockerfile está localizado, execute:

docker build -t ros2_rumble_pointcloud .
