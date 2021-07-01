#!/bin/bash
echo "INITIALIZE ZEROSIM DOCKER CONTAINER!"
set -e

# setup ros environment
source /etc/profile
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /catkin_ws/devel/setup.bash

exec "$@"
