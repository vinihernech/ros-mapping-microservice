#!/bin/bash
#ROS_IP=$(/sbin/ip -o -4 addr list wlan0 | awk '{print $4}' | cut -d/ -f1)
ROS_IP=10.10.1.140
ROS_MASTER_URI="http://10.10.3.188:30015"
export ROS_IP=$ROS_IP
export ROS_MASTER_URI=$ROS_MASTER_URI
source /home/labsea6/catkin_ws/devel/setup.bash
