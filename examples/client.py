import yaml
from is_msgs.robot_pb2 import RobotTaskRequest, PathRequest, RobotControllerProgress
from is_msgs.common_pb2 import Position, Pose, Orientation
from is_msgs.camera_pb2 import FrameTransformation,FrameTransformations
from is_wire.core import Channel, Message, Logger,Status,StatusCode, Subscription
from is_wire.rpc import ServiceProvider, LogInterceptor
from google.protobuf.empty_pb2 import Empty
from maprequest_pb2 import MappingRequest



def path_task_robot(path, config):
    cons_channel = Channel(config['broker_uri'])
    topic = "IsRosMapping.{}.MappingRequest".format(config['robot_id'])
    pathlist = []
    
    for x,y,theta in path:
        pose = Pose()
        pose.position.x = x 
        pose.position.y = y
        pose.orientation.yaw = theta
        pathlist.append(pose)
    
    task = MappingRequest()
    task.id = 0
    task.poses.extend(pathlist)
    message = Message(content=task)
    print(task.poses)
    cons_channel.publish(message,topic=topic)


if __name__ == '__main__':
    try:
        with open(r'../etc/config/config.yaml') as file:
            config = yaml.load(file, Loader=yaml.FullLoader)
    except:
            print('Unable to load config file')

    #path = [[2,-3.4,-90],[-1,-2.5,90],[-2.5,-4.68,-90],[13.12,-7.168,180],[17.9,0.726,0],[-13.1159,-9.1402,180],[0,0,0]]
    #path = [[1,-1,90],[-2.5,-4.68,-90], [13.12,-7,90],[17.9,0.6,90], [-13.1159,-9.1402,180], [0,0,0]]
    #path = [[1,-1.5,90],[1,1.5,180],[-1,1.5,180],[-1,-1.5,-90], [-2.5,-3,-90],[-2.7,-7.25,0],[9,-7.25,0],[18.11,-3.4,0],[18.11,-10.5,0],[9,-7.25,0],[-2.7,-7.25,0],[-12,-7.25,0]]
    path = [[1,1,180]]
    path_task_robot(path,config)

   


