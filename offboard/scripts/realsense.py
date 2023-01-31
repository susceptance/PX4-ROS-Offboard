'''
#! /usr/bin/env python

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import rospy
import time

def callback_pointcloud(data):
    assert isinstance(data, PointCloud2) # check that data is PointCloud2
    gen = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
    time.sleep(1)
    print (type(gen))
    for p in gen:
        print (" x : %.3f  y: %.3f  z: %.3f" %(p[0],p[1],p[2]))

def main():
    rospy.init_node('pcl_listener', anonymous=True)
    rospy.Subscriber('/camera/depth/color/points', PointCloud2, callback_pointcloud)
    rospy.spin()

if __name__ == "__main__":
    main()
'''



#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import pyrealsense2 as rs
import cv2

def convert_depth_image(ros_image):
    bridge = CvBridge()
    # Use CvBridge() to convert the ROS image to OpenCV format
    try:
    #Convert the depth image using the default passthrough encoding
        depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
        depth_array = np.array(depth_image, dtype=np.float32)
        center_idx = np.array(depth_array.shape) / 2
        print ('center depth:', depth_array[int(center_idx[0]), int(center_idx[1])])

    except CvBridgeError as e:
        print (e)
     #Convert the depth image to a Numpy array


def pixel2depth():
    rospy.init_node('pixel2depth',anonymous=True)
    rospy.Subscriber("/camera/depth/image_raw", Image, callback=convert_depth_image, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    pixel2depth()



#!/usr/bin/env python

'''
import rospy
from sensor_msgs.msg import Image as msg_Image
from darknet_ros_msgs.msg import BoundingBoxes
from cv_bridge import CvBridge, CvBridgeError
import sys
import os

class ImageListener:
    def __init__(self, topic):
        self.topic = topic
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic, msg_Image, self.imageDepthCallback)

    def imageDepthCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            pix = (data.width/2, data.height/2)
            sys.stdout.write('%s: Depth at center(%d, %d): %f(mm)\r' % (self.topic, pix[0], pix[1], cv_image[pix[1], pix[0]]))
            sys.stdout.flush()
        except CvBridgeError as e:
            print(e)
            return


if __name__ == '__main__':
    rospy.init_node("depth_image_processor")
    topic = '/camera/depth/image_raw'  # check the depth image topic in your Gazebo environmemt and replace this with your
    listener = ImageListener(topic)
    rospy.spin()
'''

