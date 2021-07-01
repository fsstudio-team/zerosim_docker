#!/bin/bash
set -e
set -u

# first build
./build_zerosim_docker.sh

# upload to docker hub 
docker tag zerosim_ros zerodog/zerosim_ros:latest
docker push zerodog/zerosim_ros:latest

