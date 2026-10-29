# syntax=docker/dockerfile:1

# Keep only what rosdep reads, so code edits don't invalidate the rosdep layer
FROM ros:jazzy-ros-base AS manifests
COPY src /src
RUN find /src -type f ! -name package.xml ! -name COLCON_IGNORE ! -name AMENT_IGNORE ! -name CATKIN_IGNORE -delete \
    && find /src -mindepth 1 -type d -empty -delete

FROM ros:jazzy-ros-base

ENV PIP_BREAK_SYSTEM_PACKAGES=1 \
    PIP_NO_CACHE_DIR=1 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Not declared in any package.xml (numpy is for robotarm_description/tools/workspace_envelope)
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-numpy \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

COPY --from=manifests /src /tmp/src
RUN apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths /tmp/src --ignore-src -y --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/* /tmp/src

WORKDIR /robotarm_core

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "[ -f /robotarm_core/install/setup.bash ] && source /robotarm_core/install/setup.bash" >> ~/.bashrc
