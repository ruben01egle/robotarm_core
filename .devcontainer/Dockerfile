FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y \
    python3-pip \
    libgl1-mesa-dev \
    libxkbcommon-x11-0 \
    libxcb-cursor0 \
    libxcb-icccm4 \
    libxcb-keysyms1 \
    libxcb-shape0 \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

ENV PIP_BREAK_SYSTEM_PACKAGES=1
RUN pip3 install PyQt6 pyqtgraph numpy transitions

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /ros2_ws

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc