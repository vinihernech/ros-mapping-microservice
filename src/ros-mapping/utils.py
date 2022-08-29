import numpy as np
from math import cos, sin, pi

import socket

from is_wire.core import Channel

class StreamChannel(Channel):
    def __init__(self, uri="amqp://guest:guest@localhost:5672", exchange="is"):
        super().__init__(uri=uri, exchange=exchange)

    def consume_last(self, return_dropped=False):
        dropped = 0
        try:
            msg = super().consume(timeout=0.1)
        except socket.timeout:
            return False
            
        while True:
            try:
                # will raise an exceptin when no message remained
                msg = super().consume(timeout=0.0)
                dropped += 1
            except socket.timeout:
                return (msg, dropped) if return_dropped else msg

def rotationZ_matrix(angle_rad):
  R = np.array([ 
               [cos(angle_rad), -sin(angle_rad), 0],
               [sin(angle_rad),  cos(angle_rad), 0],
               [0             ,  0             , 1]
            ])
  return R

def isframe_to_robotframe(pathList, initialPose):
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