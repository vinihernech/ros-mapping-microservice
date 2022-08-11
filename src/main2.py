import actionlib
import rospy
import argparse
import yaml
import sys
import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData
from math import cos, sin, pi
from is_msgs.camera_pb2 import FrameTransformation
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
from tf.transformations import quaternion_from_euler
from is_wire.core import Channel, Message, Subscription, StatusCode, Status, Logger
from is_msgs.robot_pb2 import RobotTaskRequest
from is_wire.rpc import ServiceProvider, LogInterceptor
from google.protobuf.empty_pb2 import Empty
from maprequest_pb2 import MapRequest, MapRequestReply
from streamChannel import StreamChannel
import roslaunch

if __name__ == '__main__':

   uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
   roslaunch.configure_logging(uuid)
   launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/labsea6/Documents/ros-mapping-microservice/etc/ros_mapping_launch/test.launch"])
   launch.start()
   launch.spin()


