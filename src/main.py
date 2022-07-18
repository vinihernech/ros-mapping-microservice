import actionlib
import rospy
import argparse
import yaml
import sys
import numpy as np
from math import cos, sin, pi
from is_msgs.camera_pb2 import FrameTransformation
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
from tf.transformations import quaternion_from_euler
from is_wire.core import Channel, Message, Subscription, StatusCode, Status, Logger
from is_msgs.robot_pb2 import RobotTaskRequest
from is_wire.rpc import ServiceProvider, LogInterceptor
from google.protobuf.empty_pb2 import Empty
from maprequest_pb2 import MapRequest
from streamChannel import StreamChannel


def rotationZ_matrix(angle_rad):
  R = np.array([ 
               [cos(angle_rad), -sin(angle_rad), 0],
               [sin(angle_rad),  cos(angle_rad), 0],
               [0             ,  0             , 1]
            ])
  return R

def getPoseReconstruction():
    channel_recontruction = StreamChannel("amqp://10.10.3.188:30000")
    subscription = Subscription(channel_recontruction)
    aruco_id = 5
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


def listConvert(pathList, initialPose):
    initialPose_yaw = initialPose[2]
    rotateMatriz = rotationZ_matrix(initialPose_yaw)
    rotateMatriz_inv = np.linalg.inv(rotateMatriz)
    pathListConvert = []
    for x, y, yaw in pathList:
        yaw_rad = (180*yaw)/pi
        pointConvert =  (
                        rotateMatriz_inv@(np.array([x, y, yaw_rad]) - initialPose)
                        )
        pathListConvert.append([pointConvert[0],
                            pointConvert[1], 
                            (pointConvert[2]*pi/180)])

    return pathListConvert

    
   

def get_is_points(message,ctx):

    poses = list(message.poses)
    print(poses)
    #poses = change_referential(poses)  
    for i in range(len(poses)):
        x = poses[i].position.x
        y = poses[i].position.y
        theta = poses[i].orientation.yaw
        pose = [x,y,theta]
        print(pose)
        pose = listConvert([pose],initialPose)
        send_goal(pose[0])
        print(pose[0])
    return Status(StatusCode.OK)
  

    
def send_goal(pose):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map' 
    goal.target_pose.pose.position.x = pose[0]
    goal.target_pose.pose.position.y = pose[1]
    radian = pose[2]*(3.14/180)
    quaternion = quaternion_from_euler(0, 0, radian)
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]
    client.send_goal(goal)
    print('setting final position task to  x:{:.2f}, y:{:.2f}, theta: {:.2f}'.format(pose[0],pose[1],pose[2]))
    client.wait_for_result()
    
    

if __name__ == '__main__':

    try:
        with open(r'../etc/config.yaml') as file:
            config = yaml.load(file, Loader=yaml.FullLoader)
    except:
            print('Unable to load config file')
    
    log = Logger(name='Map')
    try:
        channel = Channel(config['broker_uri'])  
    except:
        log.info("Can't connect to broker")

    robot_id = config['robot_id']
    #map_resolution = config['map_resolution']
    topic = "IsRosMapping.{}.MapRequest".format(robot_id)
    subscription = Subscription(channel)   
    rospy.init_node('send_client_goal')
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move base server")
    client.wait_for_server()  

    provider = ServiceProvider(channel)
    logging = LogInterceptor()
    provider.add_interceptor(logging)
    initialPose = getPoseReconstruction()
    provider.delegate(
        topic = topic,
        function = get_is_points,
        request_type = MapRequest,
        reply_type = Empty)

    provider.run()
  

