import actionlib
import rospy
import yaml
import numpy as np
from math import cos, sin, pi
from is_msgs.camera_pb2 import FrameTransformation
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
from is_msgs.common_pb2 import Pose
from tf.transformations import quaternion_from_euler
from is_wire.core import Channel, Message, Subscription, StatusCode, Status, Logger
from is_msgs.robot_pb2 import RobotTaskRequest
from is_wire.rpc import ServiceProvider, LogInterceptor
from is_ros_mapping.streamChannel import StreamChannel
from std_srvs.srv import Trigger, TriggerRequest
from is_ros_mapping.utils import  isframe_to_robotframe
import threading
import roslaunch
import time

class IsRosMapping():

    def __init__(self, config):
        self.config = config
        self.is_reconstruction = config['is_reconstruction']
        self.robot_id = config['robot_id']
        self.topic = "IsRosMapping.{}.MappingRequest".format(self.robot_id)
        rospy.init_node('send_client_goal')
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move base server")
        self.client.wait_for_server() 
        self.log = Logger(name='f{self.topic}')

    def get_robot_pose(self,config):
        self.log.info("x_recontruction")
        channel_recontruction = StreamChannel(config['broker_uri'])
        subscription = Subscription(channel_recontruction)
        aruco_id = config['aruco_id']
        subscription.subscribe(topic=f"reconstruction.{aruco_id}.ArUco")
        message = channel_recontruction.consume_last()
        if type(message) != bool:
            pose = message.unpack(Pose)
            x_recontruction = pose.position.x
            y_recontruction = pose.position.y
            yaw_rad_recontruction = pose.orientation.yaw
            return np.array([x_recontruction, y_recontruction, yaw_rad_recontruction])
        else: 
            return np.array([0,0,0])

    def reset_map(self):
        rospy.wait_for_service('/reset_map')
        resetMap = rospy.ServiceProxy('/reset_map', Trigger)
        sos = TriggerRequest()
        result = resetMap(sos)

    def save_map(self,map_id):
        cli_args = ['../etc/launch/save_map.launch', f'map_id:={map_id}']
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        launch.start()
        rospy.sleep(5)
        launch.shutdown()
        


    def send_goal(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map' 
        goal.target_pose.pose.position.x = pose[0]
        goal.target_pose.pose.position.y = pose[1]
        radian = pose[2]*(pi/180)
        quaternion = quaternion_from_euler(0, 0, radian)
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]
        self.client.send_goal(goal)
        self.log.info('setting final position task to  x:{:.2f}, y:{:.2f}, theta: {:.2f}'.format(pose[0],pose[1],pose[2]))
        self.client.wait_for_result()

    def modify_yaml_file(self, map_name, initial_pose,map_path ='../etc/maps/{}.yaml'):          
        with open(map_path.format(map_name), 'r+') as file:
            documents = yaml.safe_load(file)
            documents['is_origin'] = initial_pose
        with open(map_path.format(map_name), 'w') as file:
            yaml.dump(documents, file, default_flow_style=None)


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
                self.send_goal(pose[0])
                self.save_map(f'{message.id}_{i}')
        time.sleep(3)
        self.log.info("map completed successfully")
        self.client.cancel_all_goals()
        x = threading.Thread(target=self.modify_yaml_file, kwargs={"map_name": f'my_map{message.id}_{i}',"initial_pose": initialPose})
        x.start()
        return Status(StatusCode.OK)