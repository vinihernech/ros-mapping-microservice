FROM ros:noetic-ros-base

SHELL [ "/bin/bash" , "-c" ]

COPY requirements.txt .
RUN apt-get update 
RUN sudo apt-get install -y ros-noetic-map-server 
RUN apt install -y python3-pip 
RUN apt-get -y install ros-noetic-navigation 
RUN pip3 install -r requirements.txt
RUN pip3 install --upgrade protobuf==3.20.0 


WORKDIR /opt/is_ros_mapping/maps
WORKDIR /opt/is_ros_mapping
COPY src/ .
COPY etc/config/ .
COPY etc/launch/ .
RUN source /opt/ros/noetic/setup.bash 

