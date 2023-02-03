gnome-terminal --tab --title="ZeroSim" --command="bash -c 'source /opt/ros/noetic/setup.bash; 
                                                           source /home/caio/Documents/ZeroSim/devel/setup.bash; 
                                                           roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=true; $SHELL'" \
                --tab --title="URController" --command="bash -c 'sleep 4; 
                                                                source /opt/ros/noetic/setup.bash; 
                                                                source /home/caio/Documents/ZeroSim/devel/setup.bash; 
                                                                roslaunch zero_sim_ros ur10_controller.launch; $SHELL'"  \
                --tab --title="ROS1Bridge" --command="bash -c 'source /opt/ros/foxy/setup.bash;
                                                               source /opt/ros/noetic/setup.bash; 
                                                               source /home/caio/Documents/ZeroSimROS2/install/local_setup.bash; 
                                                               sleep 6; 
                                                               cd /home/caio/Documents/ZeroSim/src/zero_sim_ros/ros2bridge; 
                                                               rosparam load topics.yaml; 
                                                               ros2 run ros1_bridge parameter_bridge; $SHELL'"  \
                # --tab --title="ROS2Control" --command="bash -c 'source /opt/ros/foxy/setup.bash; 
                #                                               source /home/caio/Documents/ZeroSimROS2/install/local_setup.bash; 
                #                                               sleep 8; 
                #                                               ros2 run zerosim_ur10 ur10_controller_node; $SHELL'"  \