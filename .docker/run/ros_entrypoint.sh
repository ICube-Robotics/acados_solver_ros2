#!/bin/bash
set -e

# setup ros environment
export ROS_LOCALHOST_ONLY=1
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/ros2_dev/acados_controllers_ros2/install/setup.bash"
exec "$@"
