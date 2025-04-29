#!/bin/bash
set -e

export ROS_DOMAIN_ID=5

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/ros2_ws/install/setup.bash"

ros2 run ipc_example ipc_node