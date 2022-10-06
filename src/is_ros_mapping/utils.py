import numpy as np
from math import cos, sin, pi


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


