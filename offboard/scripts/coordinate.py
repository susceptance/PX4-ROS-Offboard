#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Header
from std_msgs.msg import String
import numpy as np
import pyrealsense2 as rs
import cv2

'''
# Create a pipeline
pipeline = rs.pipeline() # realsense pipeline open
config = rs.config() # create config class 
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

#Start pipeline
profile = pipeline.start(config)
'''

def BoundingBoxes_cb(data):
    for box in data.bounding_boxes:
        #rospy.loginfo("Xmin: {}, Xmax: {} Ymin: {}, Ymax: {}".format(box.xmin, box.xmax, box.ymin, box.ymax))
        global center_x 
        global center_y 
        center_x = (box.xmin + box.xmax) / 2
        center_y = (box.ymin + box.ymax) / 2
        #rospy.loginfo("Center_x: {}, Center_y: {}".format(center_x, center_y))
        #depth = depth_frame.get_distnce(center_x, center_y)
        #rospy.loginfo("Distance: {}".format(depth))


def ImageDepth_cb(ros_image):
    bridge = CvBridge() # Use CvBridge() to convert the ROS image to OpenCV format
    try:
    #Convert the depth image using the default passthrough encoding
        cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
        pix = (center_x, center_y)
        depth = cv_image[int(pix[1]), int(pix[0])]
        rospy.loginfo("Distance: {}".format(depth))
    except CvBridgeError as e:
        print(e)
            


def main():
    while not rospy.is_shutdown():
        rospy.init_node('detection', anonymous=True)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback=BoundingBoxes_cb)
        rospy.Subscriber("/camera/depth/image_raw", Image, callback=ImageDepth_cb)
        rospy.spin()

        

if __name__ == '__main__':
    try :
        main()
    except rospy.ROSInterruptException:
        pass
