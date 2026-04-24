FROM ros:humble-ros-base

# Installation von PyQt6 und X11-Support
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-pyqt6 \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-xinerama0 \
    libxcb-xinput0 \
    libxcb-xfixes0 \
    && rm -rf /var/lib/apt/lists/*

# Arbeitsverzeichnis
WORKDIR /ros2_ws

# Umgebungsvariablen für die GUI
ENV QT_X11_NO_MITSHM=1

# ROS Workspace sourcen
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc