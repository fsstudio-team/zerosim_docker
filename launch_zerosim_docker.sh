#!/bin/bash
set -e
set -u

# DOCKER_IMAGE=zerodog/zerosim_ros_vnc:latest
DOCKER_IMAGE=$1:latest
# 11311 = ROS Master
# 9090 = ROS Bridge
# 8083 = VNC

docker run -it --rm \
--publish=9090:9090 \
--publish=11311:11311 \
--publish=8083:8083 \
--publish=80:80 \
--publish=5678:5678 \
--name my_zerosim_docker \
$DOCKER_IMAGE \
bash