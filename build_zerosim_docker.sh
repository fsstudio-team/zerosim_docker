#!/bin/bash
set -e
set -u

for ARGUMENT in "$@"
do
   KEY=$(echo $ARGUMENT | cut -f1 -d=)

   KEY_LENGTH=${#KEY}
   VALUE="${ARGUMENT:$KEY_LENGTH+1}"

   export "$KEY"="$VALUE"
done

if [ -z ${ros1+x} ]; 
then 
  echo "ROS1 is unset"; 
else 
  echo "ROS1 Version set to: '$ros1'"; 
  docker build --no-cache -t zerosim_ros -f docker/base/DockerfileROS1_${ros1} .
fi

if [ -z ${ros2+x} ]; 
then 
  echo "ROS2 is unset"; 
else 
  echo "ROS2 Version set to: '$ros2'"; 
  docker build --no-cache -t zerosim_ros -f docker/base/DockerfileROS2_${ros2} .
fi

if [ -z ${vnc+x} ]; 
then 
  echo "vnc is unset"; 
else 
  echo "vnc Version set to: '$vnc'"; 
  docker build --no-cache -t zerosim_ros -f docker/vnc/Dockerfile_${vnc} .
fi