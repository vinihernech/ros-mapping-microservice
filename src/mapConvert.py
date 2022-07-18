# Python
import numpy as np
from math import cos, sin, pi
# IS
from is_wire.core import Channel, Subscription
from is_msgs.camera_pb2 import FrameTransformation

#  my own
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

def main(pathList):
    initialPose = getPoseReconstruction()
    return listConvert(pathList, initialPose)

if __name__ == '__main__':

    pathList = [[1, 2, 20], [2, 5, 30], [3, 5, 90]]
    print(main(pathList))
    

