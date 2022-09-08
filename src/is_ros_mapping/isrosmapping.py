import actionlib
import rospy
import yaml
import numpy as np
from math import cos, sin, pi
from is_msgs.camera_pb2 import FrameTransformation
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
from tf.transformations import quaternion_from_euler
from is_wire.core import Channel, Message, Subscription, StatusCode, Status, Logger
from is_msgs.robot_pb2 import RobotTaskRequest
from is_wire.rpc import ServiceProvider, LogInterceptor
from maprequest_pb2 import MapRequest
from streamChannel import StreamChannel
from std_srvs.srv import Trigger, TriggerRequest
from utils import  isframe_to_robotframe
import roslaunch


class IsRosMapping():

    def __init__(self, config):
        self.config = config
        self.is_reconstruction = config['is_reconstruction']
        self.robot_id = config['robot_id']
        self.topic = "IsRosMapping.{}.MapRequest".format(self.robot_id)
        rospy.init_node('send_client_goal')
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move base server")
        self.client.wait_for_server() 
        self.log = Logger(name='f{self.topic}')

    def get_robot_pose(self,config):
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

    def reset_map(self):
        rospy.wait_for_service('/reset_map')
        resetMap = rospy.ServiceProxy('/reset_map', Trigger)
        sos = TriggerRequest()
        result = resetMap(sos)

    def save_map(self,map_id):
        cli_args = ['../etc/config/launch/save_map.launch', f'map_id:={map_id}']
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        launch.start()
        launch.spin()

    def send_goal(self, pose, client):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map' 
        goal.target_pose.pose.position.x = pose[0]
        goal.target_pose.pose.position.y = pose[1]
        radian = pose[2]*(pi/180)
        quaternion = quaternion_from_euler(0, 0, radian)
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]
        client.send_goal(goal)
        self.log.info('setting final position task to  x:{:.2f}, y:{:.2f}, theta: {:.2f}'.format(pose[0],pose[1],pose[2]))
        client.wait_for_result()

    def run(self,message):
        poses = list(message.poses)
        if self.is_reconstruction:
            initialPose = self.get_robot_pose(self.config)
        else:
            initialPose = self.config['initial_pose']
        self.log.info(f"new map request ID: {message.id}")
        self.log.info("reset current map ...")
        self.reset_map()
        self.log.info("starting a new mapping ...")
        for i in range(len(poses)):
            x = poses[i].position.x
            y = poses[i].position.y
            theta = poses[i].orientation.yaw
            pose = [x,y,theta]
            pose = isframe_to_robotframe([pose],initialPose)
            self.send_goal(pose[0],self.client)
            self.save_map(f'{message.id}_{i}')
        self.save_map(f'{message.id}_final')
        self.log.info("map completed successfully")
        self.client.cancel_all_goals()
        return Status(StatusCode.OK)