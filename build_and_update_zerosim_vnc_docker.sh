#!/bin/bash

# first build
./build_zerosim_vnc_docker.sh

# upload to docker hub 
docker tag zerosim_ros_vnc zerodog/zerosim_ros_vnc:latest
docker push zerodog/zerosim_ros_vnc:latest

