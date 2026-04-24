FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    python3-pip \
    libgl1-mesa-dev \
    libxkbcommon-x11-0 \
    libxcb-cursor0 \
    libxcb-icccm4 \
    libxcb-keysyms1 \
    libxcb-shape0 \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install PyQt6 pyqtgraph numpy

WORKDIR /ros2_ws

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc