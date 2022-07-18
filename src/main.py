# import actionlib
# import rospy
import argparse
import yaml
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
# from tf.transformations import quaternion_from_euler
from is_wire.core import Channel, Message, Subscription, StatusCode, Status, Logger
from is_msgs.robot_pb2 import RobotTaskRequest
from is_wire.rpc import ServiceProvider, LogInterceptor
from google.protobuf.empty_pb2 import Empty



def get_is_points(message,ctx):
    positions = list(message.basic_move_task.positions)
    if message.basic_move_task.final_orientation is not None:
        theta = message.basic_move_task.final_orientation.yaw
    else:
        theta = 0
    for i in range(len(positions)):
        x = positions[i].x
        y = positions[i].y
        pose = [x,y,theta]
    return Status(StatusCode.OK, why = "Position was changed")
  

    
# def send_goal(pose):
#     goal = MoveBaseGoal()
#     goal.target_pose.header.frame_id = 'map' 
#     goal.target_pose.pose.position.x = pose[0]
#     goal.target_pose.pose.position.y = pose[1]
#     radian = pose[2]*(3.14/180)
#     quaternion = quaternion_from_euler(0, 0, radian)
#     goal.target_pose.pose.orientation.z = quaternion[2]
#     goal.target_pose.pose.orientation.w = quaternion[3]
#     client.send_goal(goal)
#     print('setting final position task to  x:{:.2f}, y:{:.2f}, theta: {:.2f}'.format(pose[0],pose[1],pose[2]))
#     client.wait_for_result()


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
    topic = "RobotController.{}.SetTask".format(robot_id)
    subscription = Subscription(channel)
    subscription.subscribe(topic=topic)    
    map_resolution = config['resolution']
    	
    provider = ServiceProvider(channel)
    logging = LogInterceptor()
    provider.add_interceptor(logging)

    provider.delegate(
        topic = topic,
        function = get_is_points,
        request_type = RobotTaskRequest,
        reply_type = Empty)

    # rospy.init_node('send_client_goal')
    # client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    # rospy.loginfo("Waiting for move base server")
    # goal_x, goal_y = get_is_points()
    # client.wait_for_server()  
    # send_goal([goal_x,goal_y, -90])

