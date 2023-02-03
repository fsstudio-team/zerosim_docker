#!/bin/bash

echo "starting xvfb"
Xvfb :99 -ac -screen 0 "$XVFB_WHD" -nolisten tcp  >/xvfb.log &
Xvfb_pid="$!"

echo "start the x11 vnc server"
x11vnc -display :99 --loop -noxrecord  >/x11vnc.log &

echo "checking openGl support"
glxinfo | grep '^direct rendering:'

echo "starting window manager jwm"
jwm  >/jwm.log &
# #openbox &

# echo "starting noVNC"
# /novnc/noVNC/utils/launch.sh --vnc localhost:5900 &
# /novnc/noVNC/utils/launch.sh --vnc localhost:5900 --listen 8083 >/novnc.log &
/novnc/noVNC/utils/novnc_proxy --vnc localhost:5900 --listen 8083 >/novnc.log &

#!/bin/bash
# echo "INITIALIZE ZEROSIM DOCKER CONTAINER!"
# set -e

# setup ros environment
# source /etc/profile
# source "/opt/ros/$ROS_DISTRO/setup.bash"
# source /catkin_ws/devel/setup.bash

# run catkin build if requested by setting DO_CATKIN_BUILD
# if [ -z "$DO_CATKIN_BUILD" ]
# then
#     echo "No Catkin Build"
# else
#     echo "Catkin Build Requested..."
#     catkin build 
#     source /catkin_ws/devel/setup.bash
# fi

echo "Entrypoint.sh finished executing command: $@"

# Execute passed in docker command
exec "$@"

