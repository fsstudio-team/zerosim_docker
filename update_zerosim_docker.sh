#!/bin/bash
set -e
set -u

# upload to docker hub 
docker tag zerosim_ros zerodog/zerosim_ros:latest
docker push zerodog/zerosim_ros:latest

