#!/usr/bin/env python3

from __future__ import print_function
import roslib
roslib.load_manifest('enph353_ros_lab')
import sys
import rospy
from std_msgs.msg import String 
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plt
import skimage.measure as m

from cv_bridge import CvBridge
import cv2 as cv

camera_topic = '/robot/camera1/image_raw'
drive_topic = '/cmd_vel'

# constants
kp_y = 0.001
kp_x = 0.01
global prevCenter
prevCenter = (0,0)

class imageConvertor:
    
    def __init__(self):
        print("in init")

        self.bridge = CvBridge()
        self.imageSub = rospy.Subscriber(camera_topic, Image, self.callback)
        
        self.movePub = rospy.Publisher(drive_topic, Twist, queue_size=1)
        # self.rate = rospy.Rate(2)

    def callback(self, data):
        try:
            cvImg = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except:
            print("error")
        
        # print(cvImg.shape, cvImg.dtype) = 400,600,3 , uint8

        imGrey = cv.cvtColor(cvImg, cv.COLOR_BGR2GRAY)
        cv.threshold(imGrey, 90, 255, cv.THRESH_BINARY_INV, imGrey)

        regions = m.regionprops(imGrey)
        regions.sort(key=lambda x: x.area, reverse=True)
        try:
            roadCntr = regions[0].centroid
            prevCenter = roadCntr
        except:
            roadCntr = prevCenter
        
        circleframe = cv.circle(imGrey, (int(roadCntr[1]), int(roadCntr[0])), 10, (0,0,255), -1)

        cv.imshow("img", circleframe)
        
        # print(roadCntr)
        error_x = 300 - roadCntr[1] 
        error_y = 200 - roadCntr[0]
        # print(error_y, error_x)

        self.move = Twist()
        self.move.linear.x = 0.5 + kp_y*error_y
        self.move.angular.z = kp_x*error_x

        self.movePub.publish(self.move)

        cv.waitKey(5)

#501
def main(args):
    ic = imageConvertor()
    rospy.init_node("imageConvertor", anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        print("Err in Main")

# def processImg(img) -> np.ndarray :
#     # threshold the image
#     fr = img

#     frBN = fr[:,:,1] < 90
#     frBN = frBN.astype(int)
    
#     return frBN

    # finding the center of road region
    # regions = m.regionprops(frBN)
    # regions.sort(key=lambda x: x.area, reverse=True)

    # try:
    #     roadCntr = regions[0].centroid
    #     prevCenter = roadCntr
    # except:
    #     roadCntr = prevCenter


    # return roadCntr, prevCenter
