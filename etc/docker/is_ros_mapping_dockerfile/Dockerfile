FROM ros:noetic-ros-base

SHELL [ "/bin/bash" , "-c" ]

COPY requirements.txt .
RUN apt-get update \
 && sudo apt-get install -y ros-noetic-map-server \
 && apt install -y python3-pip \
 && apt-get -y install ros-noetic-navigation \
 && pip3 install -r requirements.txt \
 && pip3 install --upgrade protobuf==3.20.0 
                   
WORKDIR /opt/is_ros_mapping/maps
WORKDIR /opt/is_ros_mapping/etc
COPY etc/ .
WORKDIR /opt/is_ros_mapping/src
COPY src/ .
RUN source /opt/ros/noetic/setup.bash 

