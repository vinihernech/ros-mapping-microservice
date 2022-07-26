def rotationZ_matrix(angle_rad):
  R = np.array([ 
               [cos(angle_rad), -sin(angle_rad), 0],
               [sin(angle_rad),  cos(angle_rad), 0],
               [0             ,  0             , 1]
            ])
  return R