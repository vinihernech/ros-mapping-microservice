from is_ros_mapping.isrosmapping import IsRosMapping
import yaml
from is_msgs.camera_pb2 import FrameTransformation
from is_wire.core import Channel, Message, Subscription, StatusCode, Status, Logger
from is_wire.rpc import ServiceProvider, LogInterceptor
from is_ros_mapping.maprequest_pb2 import MapRequest

log = Logger(name='Map') 

def load_configuration():
    try:
        with open(r'../etc/config/config.yaml') as file:
            config = yaml.load(file, Loader=yaml.FullLoader)
            return config
    except:
            log.info('unable to load config file')
    

def main():
    config = load_configuration()
    try:
        channel = Channel(config['broker_uri'])  
        log.info("connected to broker")
    except:
        log.info("can't connect to broker")

    robot_id = config['robot_id']
    topic = "IsRosMapping.{}.MapRequest".format(robot_id)
    subscription = Subscription(channel)   
    subscription.subscribe(topic)
    mapping = IsRosMapping()

    while True:
        try:
            message = channel.consume()
            msg_unpack = message.unpack(MapRequest)
            mapping.run(msg_unpack)
        except:
            pass
     
        
if __name__ == '__main__':
    main()
