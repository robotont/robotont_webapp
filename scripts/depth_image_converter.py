#!/usr/bin/env python

import roslib
roslib.load_manifest('robotont_webapp')
import rospy
import cv_bridge
import cv2
from sensor_msgs.msg import Image

def convertDepthEncoding(imageMsg, bridge):
    #convert from 16UC1 in mm to 32FC1 m
    cvImg = bridge.imgmsg_to_cv2(imageMsg)
    cvImg32F = cvImg.astype('float32') / 1000.0
    convertedImageMsg = bridge.cv2_to_imgmsg(cvImg32F)
    convertedImageMsg.header = imageMsg.header
    return convertedImageMsg

def run():
    rospy.init_node('image_converter')
    bridge = cv_bridge.CvBridge()
    
    pubDepth = rospy.Publisher('/depth_image_converter', Image, queue_size=10)
    callbackDepth = lambda imgMsg : pubDepth.publish(convertDepthEncoding(imgMsg, bridge))
    subDepth = rospy.Subscriber('camera/depth/image_rect_raw', Image, callbackDepth)

    rospy.spin()

if __name__ == '__main__':
    run()