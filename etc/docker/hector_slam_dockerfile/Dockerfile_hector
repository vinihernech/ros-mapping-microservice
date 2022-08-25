FROM ros:noetic-ros-base
 
# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]
 

RUN apt-get update && apt-get install -y ros-noetic-hector-slam

COPY  ./ros_mapping_launch/ /opt/ros/noetic/share/hector_slam_launch/launch

RUN cd /opt/ros/noetic/share/hector_slam_launch/launch \
 && mkdir -p /ros_mapping_config 

COPY  ./ros_mapping_config/ /opt/ros/noetic/share/hector_slam_launch/launch/ros_mapping_config

RUN source /opt/ros/noetic/setup.bash 


# Set the working folder at startup
WORKDIR /opt/ros/noetic/share/hector_slam_launch/launch
