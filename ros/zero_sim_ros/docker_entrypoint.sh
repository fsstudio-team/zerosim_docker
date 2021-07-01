echo "INITIALIZE ZEROSIM DOCKER CONTAINER!"

export ROS_PYTHON_VERSION=3
source /etc/profile

# -----------------------------------------------------------------------------------------
# ------ SETUP ROS ENVIRONMENT ------------------------------------------------------------
# -----------------------------------------------------------------------------------------

if [ -z "$ROS_SETUP" ] ; then
    echo "SETUP ROS ENVIRONMENT"
    # Add the source command to bash init files for 'docker exec' executions
    echo "source /opt/ros/melodic/setup.bash" >> /etc/profile
    # Source for this execution
    source /opt/ros/melodic/setup.sh

    # -----------------------------------------------------------------------------------------
    # ------ CREATE CATKIN WORKSPACE ----------------------------------------------------------
    # -----------------------------------------------------------------------------------------
    echo "CREATE CATKIN WORKSPACE"
    mkdir -p /catkin_ws/src
    # Symlink so that catkin workspace can see source code in zero_sim_ros folder
    ln -s /zero_sim_ros /catkin_ws/src/zero_sim_ros

    # catkin make and install
    cd /catkin_ws/
    catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
    catkin_make install
    chmod -R a+rwx /catkin_ws/

    # Add the source command to bash init files for 'docker exec' executions
    echo "source /catkin_ws/devel/setup.bash" >> /etc/profile
    # Source for this execution
    source /catkin_ws/devel/setup.sh

    echo "export ROS_SETUP=true" >> /etc/profile
else
    echo "ROS ENVIRONMENT ALREADY SETUP"
fi

# -----------------------------------------------------------------------------------------
# ------ LAUNCH ROSBRIDGE SERVER ----------------------------------------------------------
# -----------------------------------------------------------------------------------------

# echo "Roslaunch rosbridge server."
# roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=true --screen --master-logger-level=debug
