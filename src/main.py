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
from utils import rotationZ_matrix, isframe_to_robotframe
import roslaunch

def reset_map():
   rospy.wait_for_service('/reset_map')
   resetMap = rospy.ServiceProxy('/reset_map', Trigger)
   sos = TriggerRequest()
   result = resetMap(sos)

def save_map(map_id):
   cli_args = ['save_map.launch', f'map_id:={map_id}']
   roslaunch_args = cli_args[1:]
   roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
   uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
   roslaunch.configure_logging(uuid)
   launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
   launch.start()
   launch.spin()

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

def get_is_points(message):
    poses = list(message.poses)
    if config['is_reconstruction']:
        initialPose = get_robot_pose(config)
    else:
        initialPose = config['initial_pose']
    log.info(f"new map request ID: {message.id}")
    log.info("reset current map ...")
    reset_map()
    log.info("starting a new mapping ...")
    for i in range(len(poses)):
        x = poses[i].position.x
        y = poses[i].position.y
        theta = poses[i].orientation.yaw
        pose = [x,y,theta]
        pose = isframe_to_robotframe([pose],initialPose)
        send_goal(pose[0],client)
        save_map(f'{message.id}_{i}')
    save_map(f'{message.id}_final')
    log.info("map completed successfully")
    client.cancel_all_goals()
    return Status(StatusCode.OK)

def send_goal(pose, client):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map' 
    goal.target_pose.pose.position.x = pose[0]
    goal.target_pose.pose.position.y = pose[1]
    radian = pose[2]*(3.14/180)
    quaternion = quaternion_from_euler(0, 0, radian)
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]
    client.send_goal(goal)
    log.info('setting final position task to  x:{:.2f}, y:{:.2f}, theta: {:.2f}'.format(pose[0],pose[1],pose[2]))
    client.wait_for_result()
    
    


if __name__ == '__main__':
    log = Logger(name='Map') 
    try:
        with open(r'config.yaml') as file:
            config = yaml.load(file, Loader=yaml.FullLoader)
    except:
            log.info('unable to load config file')
    try:
        channel = Channel(config['broker_uri'])  
        log.info("connected to broker")
    except:
        log.info("can't connect to broker")

    robot_id = config['robot_id']
    topic = "IsRosMapping.{}.MapRequest".format(robot_id)
    subscription = Subscription(channel)   
    subscription.subscribe(topic)
    is_reconstruction = config['is_reconstruction']
    provider = ServiceProvider(channel)
    logging = LogInterceptor()
    provider.add_interceptor(logging)

    rospy.init_node('send_client_goal')
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move base server")
    client.wait_for_server() 

    while True:
        msg = channel.consume()
        msg_unpack = msg.unpack(MapRequest)
        get_is_points(msg_unpack)


