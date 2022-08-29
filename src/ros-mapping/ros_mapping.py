import actionlib
import rospy
import argparse
import yaml
import sys
import numpy as np
from math import cos, sin, pi
import roslaunch
from nav_msgs.msg import OccupancyGrid, MapMetaData
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
from tf.transformations import quaternion_from_euler

from is_wire.core import Channel, Message, Subscription, StatusCode, Status, Logger
from is_msgs.robot_pb2 import RobotTaskRequest
from is_msgs.camera_pb2 import FrameTransformation
from is_wire.rpc import ServiceProvider, LogInterceptor
from google.protobuf.empty_pb2 import Empty
from std_srvs.srv import Trigger, TriggerRequest

from maprequest_pb2 import MapRequest, MapRequestReply
from streamChannel import StreamChannel
from utils import rotationZ_matrix, isframe_to_robotframe


def reset_map():
   rospy.wait_for_service('/reset_map')
   resetMap = rospy.ServiceProxy('/reset_map', Trigger)
   sos = TriggerRequest()
   result = resetMap(sos)

def save_map():
   uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
   roslaunch.configure_logging(uuid)
   launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/labsea6/Documents/ros-mapping-microservice/etc/save_map.launch/test.launch"])
   launch.start()
   launch.spin()


def send_goal(pose):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map' 
    goal.target_pose.pose.position.x = pose[0]
    goal.target_pose.pose.position.y = pose[1]
    radian = pose[2]*(math.pi/180)
    quaternion = quaternion_from_euler(0, 0, radian)
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]
    client.send_goal(goal)
    log.info('setting final position task to  x:{:.2f}, y:{:.2f}, theta: {:.2f}'.format(pose[0],pose[1],pose[2]))
    client.wait_for_result()


def get_robot_pose(config):
    channel_recontruction = StreamChannel(config['broker_uri'])
    subscription = Subscription(channel_recontruction)
    aruco_id = config['aruco_id']
    subscription.subscribe(topic=f"localization.{aruco_id}.aruco")
    message = channel_recontruction.consume_last()
    if type(message) != bool:
        f = message.unpack(FrameTransformation)
        tf = f.tf.doubles
        x_recontruction = tf[0]
        y_recontruction = tf[1]
        yaw_rad_recontruction = tf[3]
        return np.array([x_recontruction, y_recontruction, yaw_rad_recontruction])
    else:
        return message

def get_is_points(message,ctx):
    poses = list(message.poses)
    #initialPose = get_robot_pose(config)
    initialPose = [0,0,0]
    log.info(f"new map request ID: {message.id}")
    log.info("start mapping ...")
    reset_map()
    for i in range(len(poses)):
        x = poses[i].position.x
        y = poses[i].position.y
        theta = poses[i].orientation.yaw
        pose = [x,y,theta]
        pose = isframe_to_robotframe([pose],initialPose)
        send_goal(pose[0])
        save_map()
    #map_reply = listener() # ?
    log.info("map completed successfully") 
    return Status(StatusCode.OK)

    

# def listener():
#     #rospy.Subscriber("/map_metadata", MapMetaData , callback)
#     #rospy.Subscriber("/map", OccupancyGrid , callback1)
#     map_data = rospy.wait_for_message("/map", OccupancyGrid, timeout = 5)
#     meta_data = rospy.wait_for_message("/map_metadata", MapMetaData, timeout = 5)
#     maprequestreply = MapRequestReply()
#     maprequestreply.width = meta_data.width
#     maprequestreply.height = meta_data.width
#     #print(meta_data.origin.position)
#     maprequestreply.map.extend(map_data.data)
#     return maprequestreply


if __name__ == '__main__':
    log.warning('Here goes the application of this code not as a service (without the IS requirements)') 
