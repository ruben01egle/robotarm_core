FROM ros:jazzy-ros-base

# Things not declared in any package.xml: middleware choice and standalone tooling
# (numpy is used by robotarm_description/tools/workspace_envelope)
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-numpy \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# MoveIt 2 (move_group, planners, Setup Assistant, RViz plugin), the pick_ik IK solver and
# ros2_controllers (joint_trajectory_controller, joint_state_broadcaster, kinematics_interface)
# for src/robotarm_moveit_config and robotarm_rbd; pinocchio (robotarm_rbd tests) and
# joint_state_publisher_gui (robotarm_description display_test.launch.py)
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-pick-ik \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-pinocchio \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*

ENV PIP_BREAK_SYSTEM_PACKAGES=1

# Install all package dependencies declared in the package.xml files under src/.
# The whole src/ is copied, so any source change reruns this layer on the next rebuild
# (see .devcontainer/README.md for the manifest-only optimization and why it is not used).
# After adding a dependency, run inside the container instead of rebuilding:
#   rosdep install --from-paths src --ignore-src -y
COPY src /tmp/src
RUN apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths /tmp/src --ignore-src -y -r --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/* /tmp/src

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /robotarm_core

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "[ -f /robotarm_core/install/setup.bash ] && source /robotarm_core/install/setup.bash" >> ~/.bashrc
