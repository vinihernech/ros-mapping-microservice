
#fix  imports 

if __name__ == '__main__':

    try:
        with open(r'../etc/config/config.yaml') as file:
            config = yaml.load(file, Loader=yaml.FullLoader)
    except:
            log.info('Unable to load config file')
    
    log = Logger(name='Map')
    try:
        channel = Channel(config['broker_uri'])  
        log.info("connected to broker")
    except:
        log.info("Can't connect to broker")

    robot_id = config['robot_id']
    topic = "IsRosMapping.{}.MapRequest".format(robot_id)
    subscription = Subscription(channel)   
    rospy.init_node('send_client_goal')
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move base server")
    client.wait_for_server()  

    provider = ServiceProvider(channel)
    logging = LogInterceptor()
    provider.add_interceptor(logging)
    
    provider.delegate(
        topic = topic,
        function = get_is_points,
        request_type = MapRequest,
        reply_type = Empty)

    provider.run()
  

