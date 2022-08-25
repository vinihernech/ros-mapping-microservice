import rospy
import sys
import roslaunch

if __name__ == '__main__':

   uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
   roslaunch.configure_logging(uuid)
   launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/labsea6/Documents/ros-mapping-microservice/etc/ros_mapping_launch/test.launch"])
   launch.start()
   launch.spin()


