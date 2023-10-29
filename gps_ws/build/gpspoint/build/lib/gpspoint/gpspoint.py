import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
# from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
# import cv2 # OpenCV library
from rclpy.executors import MultiThreadedExecutor
import os
import numpy as np
# import open3d as o3d

import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs


from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import Imu
from gps_msgs.msg  import GPSFix
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


# from cepton_messages.msg import CeptonPointData
import sys

#from registry import converts_from_numpy, converts_to_numpy

import time

from threading import Thread, Lock
import numpy as np
import math
import message_filters
from message_filters import ApproximateTimeSynchronizer, Subscriber

 
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)* 180.0 / math.pi  
  
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)* 180.0 / math.pi  
  
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)* 180.0 / math.pi  
  
    return roll_x, pitch_y, yaw_z # in degree


lat = []
long = []
vel = []

class ImageListener(Node):

    def __init__(self):
        super().__init__('data_subsriber_node')

        #sensor_msgs.PointCloud2
        # Set up a subscription to the 'pcd' topic with a callback to the 
        # function `listener_callback`
        # GPSFix Imu

        self.gps_msg = message_filters.Subscriber(self, GPSFix, 'gps')
        #self.imu_msg = message_filters.Subscriber(self, Imu, 'imu')


        queue_size = 10
        ts_tolerance = 20
        #ats = ApproximateTimeSynchronizer([self.gps_msg, self.imu_msg], queue_size, ts_tolerance)
        ats = ApproximateTimeSynchronizer([self.gps_msg], queue_size, ts_tolerance)
        ats.registerCallback(self.listener_callback)

    def listener_callback(self, gps_msg):
        """
        Callback function.
        """
        # Display the message on the console
        #self.get_logger().info('Receiving video frame')

        #print(gps_msg.latitude)
        #print(gps_msg.longitude)
        print(gps_msg.speed)

        #qx,qy,qz,qw = imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w
        #rpy = euler_from_quaternion(qx,qy,qz,qw)
        #print(rpy)


        lat.append(float(gps_msg.latitude))
        long.append(float(gps_msg.longitude))
        vel.append(float(gps_msg.speed))

        
        gps_location = np.array((lat, long, vel))
        np.savetxt("/home/osu/sensor_team/gpspoint_1.csv", gps_location.T, delimiter=",")
        #np.savetxt(r"./gps_data/gps_points.txt", gps_location.T)
        #np.save(r"./gps_data/rosbag2_2023_05_29-14_39_54.npy", gps_location.T)
        #print(gps_location)


def main(args=None):

  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  listener1 = ImageListener()

  
  executor = MultiThreadedExecutor()
  executor.add_node(listener1)



  executor.spin()

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  listener1.destroy_node()


  # Shutdown the ROS client library for Python
  rclpy.shutdown()

  


  
if __name__ == '__main__':
  main()
