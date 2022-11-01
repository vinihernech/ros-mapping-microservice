import sys
import math
import time
import argparse
import json

import numpy as np
import cv2
import matplotlib
#matplotlib.use('Qt5Agg')

import matplotlib.pyplot as plt

from is_msgs.robot_pb2 import RobotTaskRequest, RobotTaskReply, RobotControllerProgress
from is_msgs.image_pb2 import Image,Vertex,ObjectAnnotations,BoundingPoly
from is_msgs.camera_pb2 import FrameTransformation,FrameTransformations,GetCalibrationReply,GetCalibrationRequest
from is_wire.core import Channel, Message, Subscription

from utils import image_to_np,tensor_to_np,np_to_tensor
from utils import StreamChannel,wait_for_reply

def transform_resize(point,w,original_w):
    return int(point*float(w)/original_w)


class CameraClient():
    def __init__(self,broker_uri,camera_id,image_shape):
        self.camera_id = camera_id
        self.channel = StreamChannel(broker_uri)
        self.subscription = Subscription(channel = self.channel)

        self.image_shape = image_shape
        self.frame = np.zeros((image_shape[0],image_shape[1],3))
        self.overlay = np.zeros((image_shape[0],image_shape[1],3))
        self.image = cv2.addWeighted(self.frame.astype(np.uint8),0.8,self.overlay.astype(np.uint8),0.5,1)
        self.topics = {}

        self.window = 'Camera {}'.format(camera_id)
        cv2.namedWindow(self.window, cv2.WINDOW_AUTOSIZE)
        cv2.moveWindow(self.window, 0, 0)

        self.set_camera_topic(camera_id)
        self.set_camera_subscription()
        self.get_calibration()

    def set_camera_topic(self,camera_id):
        self.camera_id = camera_id
        self.topics['camera'] = 'CameraGateway.{}.Frame'.format(self.camera_id)

    def set_camera_subscription(self):
        for name,topic in self.topics.items():
            self.subscription.subscribe(topic=topic)
    
    def consume_image(self):
        message = self.channel.consume()
        if message.topic == self.topics['camera']:            
            image = image_to_np(message.unpack(Image)) 
            if (image.shape[0] != self.image_shape[0]) or (image.shape[1]!= self.image_shape[1]):
                image = cv2.resize(image,(self.image_shape[1],self.image_shape[0]))
            self.frame = image
            self.image = cv2.addWeighted(self.frame.astype(np.uint8),0.8,self.overlay.astype(np.uint8),0.5,1)
    
    def get_calibration(self):
        self.topics['calibration'] = 'FrameTransformation.GetCalibration'
        calibration_request =  GetCalibrationRequest()
        calibration_request.ids.extend([self.camera_id])
        calibration_message = Message(content=calibration_request,reply_to=self.subscription) 
        self.channel.publish(message=calibration_message,topic = self.topics['calibration'])           

        try:
           print("waiting for camera calibration")
           calibration_reply = wait_for_reply(self.channel,calibration_message.correlation_id, timeout=None, max_tries = 30)
           print(calibration_reply)
           calibrations_struct = calibration_reply.unpack(GetCalibrationReply)
           print("got calibration reply")
           self.calibration = calibrations_struct.calibrations[0]   
           print('got calibrations')
  #        print(self.calibration)
        except:
             print('Couldn\'t consume camera calibrations')
             self.calibration= None


    def refresh_overlay(self):
        self.overlay = np.zeros((self.image_shape[0],self.image_shape[1],3))
        self.image = cv2.addWeighted(self.frame.astype(np.uint8),0.8,self.overlay.astype(np.uint8),0.5,1)

    def show(self):
        cv2.imshow(self.window,self.image)

    def draw_aruco(self,obj):
        for annotation in obj.objects:
            if annotation.id == 5:
                poly = annotation.region
                max_x,max_y=0,0
                for index,vert in enumerate(poly.vertices):
                    if index<(len(poly.vertices)-1):
                        point1= (int(vert.x),int(vert.y))
                        max_x = max(int(vert.x),max_x)
                        max_y = max(int(vert.y),max_y)
                        point2=(int(poly.vertices[index+1].x),int(poly.vertices[index+1].y))
#                        point1 = (transform_resize(point1[0],self.image_shape[1],1920),transform_resize(point1[1],self.image_shape[0],1080))
#                        point2 = (transform_resize(point2[0],self.image_shape[1],1920),transform_resize(point2[1],self.image_shape[0],1080))
                    elif index == (len(poly.vertices)-1):
                        point1 = (int(poly.vertices[0].x),int(poly.vertices[0].y))
                        point2 = (int(poly.vertices[index].x),int(poly.vertices[index].y))
                    point1 = (transform_resize(point1[0],self.image_shape[1],1270),transform_resize(point1[1],self.image_shape[0],720))
                    point2 = (transform_resize(point2[0],self.image_shape[1],1270),transform_resize(point2[1],self.image_shape[0],720))
    
                    cv2.line(self.overlay,point1,point2,color=(255,0,255),thickness=4)
                text_x,text_y = transform_resize(max_x,self.image_shape[1],1270),transform_resize(max_y,self.image_shape[0],720)
                cv2.putText(self.overlay,str(annotation.id),(text_x,text_y),cv2.FONT_HERSHEY_SIMPLEX,1,color=(255,255,255))


    def set_mouse_callback(self,function,param):
        cv2.setMouseCallback(self.window,function,param)


class MapClient():
    def __init__(self,mapped_area_config):
        '''To-do, could also receive the extrinsic positions of the cameras and insert it on the map
        '''
        self.area_id = mapped_area_config['area_id']
        self.area_name = mapped_area_config['area_name']
        self.scatterplots = {}
        self.position_histories = {}
        
        self.fig,self.axis = plt.subplots(1,1)
        plt.grid(True)
        plt.show(block=False)

      
        self.plot_map(mapped_area_config['area_map'])
        self.plot_obstacles(mapped_area_config['obstacles'])
        # self.plot_debug_map(area_array,obstacles_array)
        # ax = plot_lab(ax)
        
        self.scatterplots['aruco'] = None
        self.position_histories['aruco'] = np.array([])

    def plot_map(self,area_map_config):
        xmin, xmax = float('inf'),-float('inf')
        ymin, ymax = float('inf'),-float('inf')
        for key,point in area_map_config.items():
            xmin = point['x'] if xmin>point['x'] else xmin
            ymin = point['y'] if ymin>point['y'] else ymin
            xmax = point['x'] if xmax<point['x'] else xmax
            ymax = point['y'] if ymax<point['y'] else ymax
            
        self.axis.set_xlim([xmax*1.1, xmin*1.1 ])
        self.axis.set_ylim([ymax*1.1,ymin*1.1])
        room = np.array([[xmax,ymax],[xmax,ymin],[xmin,ymin],[xmin,ymax],[xmax,ymax]])
        self.axis.plot(room[:,0],room[:,1],color='y')

    def plot_obstacles(self,obstacles_config):
        for key,obstacle in obstacles_config.items():
            xmin = obstacle['x_min']
            ymin = obstacle['y_min']
            xmax = obstacle['x_max']
            ymax = obstacle['x_max']
            obs = np.array([[xmax,ymax],[xmax,ymin],[xmin,ymin],[xmin,ymax],[xmax,ymax]])        
            self.axis.plot(obs[:,0],obs[:,1],color='orange')   
        

    def start_scatter(self,position,marker = '.',color='k'):
        x,y = position
        return  self.axis.scatter(x,y,marker=marker,color=color)
    
    def start_position_history(self,position):
        x,y,z,roll = position
        return np.array([x,y,z])

    def get_position_from_trans(self,transformation):
        #transformation received is of type np, as obtained by tensor to np
        x,y,z = transformation[0,3],transformation[1,3],transformation[2,3]
        pitch = - math.asin(transformation[2,0])
        yaw = math.atan2(transformation[2,1]/math.cos(pitch),transformation[2,2]/math.cos(pitch))
        roll = math.atan2(transformation[1,0]/math.cos(pitch),transformation[0,0]/math.cos(pitch))
        return x,y,z,roll

    def update_position(self,name,position,marker = '.',color='k'):
        x,y= position
        print(f'{name} position: {x:.2f}, {y:.2f}')
        if self.scatterplots.get(name) is not None:
            #To-do: this has to check the size of the array, else it will grow forever, not a problem at the moment
            self.position_histories[name] = np.vstack((self.position_histories[name],np.array([x,y,0])))
            self.scatterplots[name].set_offsets(self.position_histories[name][:,:-1])
        else:
            self.position_histories[name] = self.start_position_history([x,y,None,None])
            self.scatterplots[name] = self.start_scatter([x,y],marker = marker,color=color)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()



class Client():
    def __init__(self,config,camera_ids, aruco_id,image_shape,show_map=True,mapping_config= None):
        broker_uri = config['broker_uri']
        self.robot_topics = {}
        self.robot_channel = Channel(broker_uri)
        self.robot_subscription = Subscription(channel=self.robot_channel)
        
        self.aruco_topics = {}
        self.aruco_channel = StreamChannel(broker_uri)
        self.aruco_subscription = Subscription(channel=self.aruco_channel)

        self.transformation_topics = {}
        self.transformation_channel = Channel(broker_uri)
        self.transformation_subscription = Subscription(channel=self.transformation_channel)


        self.cameras = self.set_cameras(broker_uri,camera_ids,image_shape)

        self.camera_options = [str(x) for x in range(0,4)] #should be entered as a config, camera id of all cameras the user can change to
        self.mapping_list = []
        self.mapping_config = mapping_config
        self.set_robot_topic(robot_id=0)
        self.set_aruco_topic(camera_ids)
        self.set_transformation_topic(aruco_id=aruco_id,camera_id=camera_ids[0])
        self.set_mouse_callback(0)

        if show_map:
            self.area_map = MapClient(config['mapped_area'])
        else: 
            self.area_map = None            


    def set_subscription(self,subscription,topics):
        for name,topic in topics.items():
            subscription.subscribe(topic=topic)

    
    def set_cameras(self,broker_uri,camera_ids,image_shape):
        cameras = []
        for camera_id in camera_ids:
            cameras.append(CameraClient(broker_uri,camera_id,image_shape))
        return cameras

    def change_camera(self,camera,camera_id):
        print(f"changing to camera:{camera_id}")
        camera.camera_id = camera_id
        camera.set_camera_topic(camera_id)
        camera.set_camera_subscription()
        camera.get_calibration()
        camera.refresh_overlay()
        return camera

    def set_aruco_topic(self,camera_ids):
        for camera_id in camera_ids:
            self.aruco_topics[camera_id] = 'ArUco.{}.Detection'.format(camera_id)
        self.set_subscription(self.aruco_subscription,self.aruco_topics)

    def set_transformation_topic(self,aruco_id,camera_id):
        '''commented first line cause im still tinking if it is necessary'''
        # self.transformation_topics['transformation'] = 'FrameTransformation.{}.{}.1000'.format(100+aruco_id,camera_id) 
        self.transformation_topics['transformation'] = 'FrameTransformation.{}.1000'.format(100+aruco_id)
        self.transformation_topics['calibration'] = 'FrameTransformation.GetCalibration'
        self.set_subscription(self.transformation_subscription,self.transformation_topics)

    def set_mouse_callback(self,camera_index=0):
        self.cameras[camera_index].set_mouse_callback(self.on_mouse_click,[camera_index])
        

    def send_mapping_list(self):
        from is_msgs.common_pb2 import  Pose
        from maprequest_pb2 import MappingRequest
        cons_channel = Channel(self.mapping_config['broker_uri'])
        topic = "IsRosMapping.{}.MappingRequest".format(self.mapping_config['robot_id'])
        pathlist = []
        for x,y in self.mapping_list:
            pose = Pose()
            pose.position.x = x 
            pose.position.y = y
            pose.orientation.yaw = 0
            pathlist.append(pose)
        task = MappingRequest()
        task.id = 0
        task.poses.extend(pathlist)
        message = Message(content=task)
        print(self.mapping_list)
        cons_channel.publish(message,topic=topic)
        self.mapping_list = []


    def on_mouse_click(self,event,x,y,flags,param):
        camera_index = param[0]
        '''maybe show the clicked button on all cameras, but the function should be transformed later'''
        if event == cv2.EVENT_FLAG_LBUTTON:
            
            image_x,image_y = transform_resize(x,1280,self.cameras[0].image_shape[1]),transform_resize(y,720,self.cameras[0].image_shape[0])

            #self.stop_robot()
            self.cameras[camera_index].refresh_overlay()
            camera_calibration = self.cameras[camera_index].calibration
            
            cv2.circle(self.cameras[camera_index].overlay,(x,y),5,color=(255,255,255),thickness=-1)
            world_x,world_y = self.get_world_point(camera_calibration,(image_x,image_y))

            cv2.putText(self.cameras[camera_index].overlay,'({:.1f},{:.1f})'.format(world_x,world_y),(x+5,y),cv2.FONT_HERSHEY_SIMPLEX,0.5,color=(255,255,255))
            self.cameras[camera_index].image = cv2.addWeighted(self.cameras[camera_index].frame.astype(np.uint8),0.8,self.cameras[camera_index].overlay.astype(np.uint8),0.5,1)
            self.cameras[camera_index].show()
            self.mapping_list.append((world_x,world_y))
            print(f"mapping list:{self.mapping_list}")
            #self.set_robot_task(world_x,world_y)

        #elif event == cv2.EVENT_FLAG_RBUTTON:
        #    self.stop_robot()


    def get_world_point(self,camera_calibration,point):
        x,y = point
        homogenic_point = np.array([x,y,1])
        intrinsic = tensor_to_np(camera_calibration.intrinsic)
        extrinsic = tensor_to_np(camera_calibration.extrinsic[0].tf)
        extrinsic = np.delete(extrinsic,2,1)
        arr = np.zeros((3,1))
        proj_matrix = np.hstack((np.eye(3),arr))       
        h0 = np.matmul(intrinsic,proj_matrix)
        h = np.matmul(h0,extrinsic)
        hinv = np.linalg.inv(h)
        lamb = h[2,0]*homogenic_point[0] + h[2,1]*homogenic_point[1] + h[2,2]
        new_invh = hinv / lamb
        world_point = np.matmul(new_invh,homogenic_point)
        world_point = world_point / world_point[-1]
        world_x,world_y = world_point[0],world_point[1]
        return (world_x,world_y)


    def set_robot_topic(self,robot_id=0):     
        self.robot_id = robot_id
        prefix = "RobotController.{}".format(self.robot_id)
        self.robot_topics['set_task'] = "{}.SetTask".format(prefix)
        self.robot_topics['controller'] = '.'.join([prefix,'Progress'])
        self.set_subscription(self.robot_subscription, self.robot_topics)
        #self.stop_robot()


    #def set_robot_task(self,x,y):
    #    task = final_position(target=(x, y), rate=10)
    #    print('setting final position task to  x:{:.2f}, y:{:.2f}'.format(x,y) )
    #    message = Message(content=task, reply_to=self.robot_subscription)
    #    self.robot_channel.publish(message,topic= self.robot_topics['set_task'])
    #    reply = wait_for_reply(self.robot_channel,message.correlation_id)
    #    if reply is not None:
    #        print(reply.unpack(RobotTaskReply))

    #def set_robot_lemniscate(self):
    #    task = lemniscate_of_bernoulli((2.8,1.5),(0,0),30, rate=5.0, allowed_error=0.1)
    #    print('doing lemniscate task')
    #    message = Message(content=task, reply_to=self.robot_subscription)
    #    self.robot_channel.publish(message,topic=self.robot_topics['set_task'])
    #    reply = wait_for_reply(self.robot_channel,message.correlation_id)
    #    if reply is not None:
    #        print(reply.unpack(RobotTaskReply))
      
    #def stop_robot(self):
        #self.robot_channel.publish(Message(content=stop()),topic=self.robot_topics['set_task'])
    
    
    def run(self):
        aruco_message = None
        robot_message = None
        transformation_message = None
        try:
            while True:
                #try:
                #    robot_message = self.robot_channel.consume(timeout=0.1)
            #   #     print(robot_message)
                #except:
                #    pass
                try:                
                    transformation_message = self.transformation_channel.consume(timeout=0.1)             
                except:
                    pass
                aruco_message = self.aruco_channel.consume()
                for camera in self.cameras:
                    if aruco_message.topic == self.aruco_topics[camera.camera_id]:
                        aruco_obj = aruco_message.unpack(ObjectAnnotations)
                        camera.draw_aruco(aruco_obj)        
                    camera.consume_image() 
                    camera.show()
                    
                #if self.area_map is not None: 
                #    if robot_message is not None:
#               #     if True:
                #        if robot_message.topic == self.robot_topics['controller']:
                #            progress = robot_message.unpack(RobotControllerProgress)              
                #            controller_x,controller_y = progress.current_pose.position.x, progress.current_pose.position.y
                #            self.area_map.update_position('robot',[controller_x,controller_y],color='r')
                #    if transformation_message is not None:
                #        if transformation_message.topic == self.transformation_topics['transformation']:
                #            transformation = transformation_message.unpack(FrameTransformation)
                #            trans_np = tensor_to_np(transformation.tf)
                #            x,y,_,_ = self.area_map.get_position_from_trans(trans_np)
                #            self.area_map.update_position('aruco',[x,y],color='b')

                key = cv2.waitKey(100)
                if chr(key&0xFF) == 'q':
                    print("Quitting, sent stop task")
#                    self.stop_robot()
                    time.sleep(1)
                    break
                elif chr(key&0xFF) in self.camera_options:
                   self.cameras[0]= self.change_camera(self.cameras[0],int(chr(key&0xFF)))  
                elif chr(key&0xFF) == 'l':
                    pass
                    #self.set_robot_lemniscate()
                elif chr(key&0xFF) == 'x':
                    self.send_mapping_list()
                    #self.stop_robot()
                elif chr( key&0xFF) == 'c':
                    for camera in self.cameras:
                        camera.refresh_overlay()
                else: 
                    pass
        
        except Exception as e:
            print("exception")
            print(e)
                


if __name__ == '__main__':

    import yaml
    parser = argparse.ArgumentParser()
    parser.add_argument('-c','--camera', nargs='+',required=True) # -c 1 2 3 
    args = parser.parse_args()
    
    config = json.load(open('../etc/config/config_flavio.json', 'r'))
    with open(r'../etc/config/config.yaml') as file:
            map_config = yaml.load(file, Loader=yaml.FullLoader)

    image_shape = (720,1280)
    #image_shape = (270,480)
    aruco_id = 5
    camera_ids = args.camera
    camera_ids = list(map(int,camera_ids))

    client = Client(config,camera_ids, aruco_id,image_shape,show_map=False,mapping_config=map_config)
    client.run()
