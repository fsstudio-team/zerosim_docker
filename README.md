# ZeroSim Docker 

## Overview 

These are Docker images to work with [ZeroSim](https://github.com/fsstudio-team/ZeroSimROSUnity)
There are two Docker images:

* ROS only Docker that contains all the ROS specific packages for working with [ZeroSim](https://github.com/fsstudio-team/ZeroSimROSUnity).
* ROS + VNC server that allows running ROS GUI applications like RViz and RQT.

## Building

Build the ZeroSim Docker image using the `build_zerosim_docker.sh`. 

The following arguments are available: `ros1`, `ros2` and `vnc`.
You must choose the ROS1 version since ZeroSim only directly supports ROS1 at the moment. If you want to use ROS2, you should bridge all the topics and actions (if necessary) to ROS2. 

Also, choose the VNC Docker image file according to the ROS1 version (ubuntu18 or ubuntu20).

The command line to build the ZeroSim Docker image is:

```bash
./build_zerosim_docker.sh ros1="noetic" ros2="foxy" vnc="ubuntu20"
```
obs.: ros2 argument is optional.

Available ROS versions and the corresponding VNC versions:
- ROS1: `melodic` (VNC version: ubuntu18) and `noetic` (VNC version: ubuntu20)
- ROS2: `dashing` (VNC version: ubuntu18) and `foxy` (VNC version: ubuntu20)

## Running

```bash
# If pulling from Dockerhub
$ DOCKER_IMAGE=zerodog/zerosim_ros_vnc:latest
# If using local image
$ DOCKER_IMAGE=zerosim_ros:latest

# 11311 = ROS Master
# 9090 = ROS Bridge
# 8083 = VNC
$ docker run -it --rm \
--publish=9090:9090 \
--publish=11311:11311 \
--publish=8083:8083 \
--env DO_CATKIN_BUILD=1 \
--name my_zerosim_vnc_docker \
$DOCKER_IMAGE \
bash
```

## (optional) Configuring ROS-Bridge

>Tested with ROS1 Noetic and ROS2 Foxy

Please follow the next build steps if you want to use both ROS1 and ROS2 through ROS-bridge

```bash
$ source /opt/ros/foxy/setup.bash
# If you see the message `ROS_DISTRO was set to 'noetic' before. Please make sure that the environment does not mix paths from different distributions`, execute the following command: exec bash and then source ros foxy again

$ cd /catkin_ws_ros2
$ sudo apt update
$ rosdep install --ignore-src --from-paths src -y -r 
$ colcon build --symlink-install --packages-skip ros1_bridge
$ source /opt/ros/noetic/setup.bash
$ colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure 
```

Check if the ros1_bridge package can be found. It should return: `/catkin_ws_ros2/install/ros1_bridge`
```bash
$ ros2 pkg prefix ros1_bridge
```

If you try to build this package after installing some others and get any error from control_msgs package, do the following and rebuild the workspace:
```bash
$ sudo apt-get remove ros-foxy-controller-manager-msgs  
$ sudo apt-get remove ros-foxy-ros2-control 
$ sudo apt-get remove ros-foxy-ros2-controllers 
```

## (optional) Build the package to control UR10 using MoveIt2 (ROS2)

For this example to work you have do build ZeroSim using ROS1 Noetic and ROS2 Foxy.

Build the package with the following commands:
```bash
$ source /opt/ros/foxy/setup.bash
# If you see the message `ROS_DISTRO was set to 'noetic' before. Please make sure that the environment does not mix paths from different distributions`, execute the following command: exec bash and then source ros foxy again

$ cd /catkin_ws_ros2_ur 
$ sudo apt update 
$ sudo apt-get install ros-foxy-ros2-control -y 
$ sudo apt-get install ros-foxy-ros2-controllers -y 
$ sudo apt install python3-colcon-common-extensions -y 
$ rosdep install --ignore-src --from-paths src -y -r 
$ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```


