# docker.pkg.github.com/fsstudio-team/zerosimros/zerosim_ros
FROM osrf/ros:melodic-desktop-full

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENV DEBIAN_FRONTEND=noninteractive

# Add ROS Repositories
ENV ROS_DISTRO melodic

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654


# installl apt-util
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils \
    && rm -rf /var/lib/apt/lists/*

# install os drivers such as opengl
RUN apt-get update && apt-get -y install \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    xserver-xorg-video-all \
    && rm -rf /var/lib/apt/lists/*

# install system tools like VIM 
RUN apt-get update && apt-get -y install \
    vim \
    python-pip \
    && rm -rf /var/lib/apt/lists/*

# Micro editor
RUN curl https://getmic.ro | bash
RUN mv micro /usr/bin

# Helpful Python Packages
RUN pip install debugpy
RUN pip install numpy
RUN pip install scipy
RUN pip install pyquaternion


# Install 
# install ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-desktop-full \
    && rm -rf /var/lib/apt/lists/*


# install ROS dependencies and tools
RUN apt-get update && apt install -y \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential \
    python-rosdep \
    python-catkin-pkg \
    python-catkin-tools \
    ros-$ROS_DISTRO-genpy \
    ros-$ROS_DISTRO-moveit \
    ros-$ROS_DISTRO-moveit-core \
    ros-$ROS_DISTRO-geographic-msgs \
    ros-$ROS_DISTRO-moveit-kinematics \
    ros-$ROS_DISTRO-moveit-visual-tools \
    ros-$ROS_DISTRO-joint-trajectory-controller \
    ros-$ROS_DISTRO-effort-controllers \
    ros-$ROS_DISTRO-dwa-local-planner \
    ros-$ROS_DISTRO-panda-moveit-config \
    ros-$ROS_DISTRO-moveit-simple-controller-manager \
    ros-$ROS_DISTRO-moveit-planners-ompl \
    ros-$ROS_DISTRO-gmapping \
    ros-$ROS_DISTRO-rqt-robot-dashboard \
    ros-$ROS_DISTRO-rqt-controller-manager \
    ros-$ROS_DISTRO-rqt-tf-tree \
    ros-$ROS_DISTRO-moveit-visual-tools \
    ros-$ROS_DISTRO-octomap \
    ros-$ROS_DISTRO-octomap-rviz-plugins \
    ros-$ROS_DISTRO-octomap-server \
    ros-$ROS_DISTRO-ros-control \
    ros-$ROS_DISTRO-ros-controllers \    
    ros-$ROS_DISTRO-combined-robot-hw \
    ros-$ROS_DISTRO-turtlebot3-teleop \
    ros-$ROS_DISTRO-rtabmap-ros \
    && rm -rf /var/lib/apt/lists/*

# force upgrade everything
# FIX: https://answers.ros.org/question/360537/unknown-error-handler-name-rosmsg/
# RUN apt-get update && apt-get upgrade -y


# Create catkin workspace
# See: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
ENV ROS_DISTRO melodic
ENV CATKIN_WS=/catkin_ws
RUN mkdir -p $CATKIN_WS/src


# taken from moveit source install Dockerfile (https://github.com/ros-planning/moveit/blob/kinetic-devel/.docker/source/Dockerfile)
# Replacing shell with bash for later docker build commands
RUN mv /bin/sh /bin/sh-old && \
    ln -s /bin/bash /bin/sh

# create ROS ENV
RUN source /opt/ros/$ROS_DISTRO/setup.bash
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS 
RUN catkin init
# RUN catkin config --extend /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3  
# *NO* Python 3 :-(
RUN catkin config --extend /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Release
WORKDIR $CATKIN_WS/src

# Install ZeroSim ROS
COPY ./ros/zero_sim_ros/ $CATKIN_WS/src/zero_sim_ros/

# install Universal Robots 
RUN git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/universal_robot.git

# install ROS bridge from source
# NOTE:  there seems to be issues with ROS bridge > 1.11.10 so we use the specific branch
RUN git clone -b 0.11.10 --single-branch https://github.com/RobotWebTools/rosbridge_suite.git
RUN cp -R ./rosbridge_suite/rosbridge_suite $CATKIN_WS/src
RUN cp -R ./rosbridge_suite/rosbridge_library $CATKIN_WS/src
RUN cp -R ./rosbridge_suite/rosbridge_server $CATKIN_WS/src
RUN cp -R ./rosbridge_suite/rosbridge_msgs $CATKIN_WS/src
RUN cp -R ./rosbridge_suite/rosapi $CATKIN_WS/src
RUN rm -rf ./rosbridge_suite


# and build
WORKDIR $CATKIN_WS
ENV PYTHONIOENCODING UTF-8

# checking dependencies and installing dependencies
RUN apt-get update -y
RUN apt-get dist-upgrade -y
RUN rosdep update
RUN rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src -r -y

# build ROS packages
RUN catkin build
RUN source devel/setup.bash

# Have new bash always source setup.bash
RUN touch /root/.bashrc
RUN echo 'source /catkin_ws/devel/setup.bash' >> /root/.bashrc

# We need to "premake" this directories and change permissions when we start up docker as a user that is not superuser.
# For example for X11 when we want to run things like RVIZ.
RUN mkdir /.ros
RUN mkdir /.rviz
RUN chmod -R a+rwx /catkin_ws/
RUN chmod -R a+rwx /.ros/
RUN chmod -R a+rwx /.rviz/

# Default command - initializes Catkin workspace and installs
# COPY docker entry point
COPY ./docker/base/docker_entrypoint.sh $CATKIN_WS
RUN chmod +x $CATKIN_WS/docker_entrypoint.sh

# Set entrypoint
ENTRYPOINT ["/catkin_ws/docker_entrypoint.sh"]
CMD ["bash"]