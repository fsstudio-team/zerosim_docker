#!/bin/bash
echo "INITIALIZE ZEROSIM DOCKER CONTAINER!"
set -e

# setup ros environment
source /etc/profile

# Do not uncomment the lines below if you are going to use ROS Bridge
# source "/opt/ros/$ROS_DISTRO/setup.bash"
# source /catkin_ws/devel/setup.bash

exec "$@"
