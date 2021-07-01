#!/bin/bash
set -e
set -u

# build the base zero_sim docker image
./build_and_update_zerosim_docker.sh

docker build --no-cache -t zerosim_ros_vnc -f docker/vnc/Dockerfile .

