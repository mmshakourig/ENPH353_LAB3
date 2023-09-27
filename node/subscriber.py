#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge

class imageConvertor:
    
    def __init__(self):
        self.bridge = CvBridge
        self.imageSub = rospy.Subscriber(camera_topic, Image, self.callback, queue_size=3)


    def callback(self, data):
      try:
        camera_subscriber()
    except rospy.ROSInterruptException:
        pass  


def camera_subscriber():
    rospy.init_node('camera_subscriber', anonymous=True)

    # Define the topic you want to subscribe to
    camera_topic = '/robot/camera1/image_raw/compressed'  # Replace with your actual camera topic

    # Subscribe to the camera topic with the specified message type (Image in this case)
    rospy.Subscriber(camera_topic, Image, calcDriveCommand)

    # Spin to keep the node alive
    rospy.spin()

if __name__ == '__main__':
    try:
        camera_subscriber()
    except rospy.ROSInterruptException:
        pass


# def skid_steering_publisher():
#     # Initialize the ROS node
#     rospy.init_node('skid_steering_publisher', anonymous=True)

#     # Create a publisher object that will publish Twist messages
#     pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

#     # Create a Twist message to control the skid steering drive
#     twist_msg = Twist()

    
#     # Publish the Twist message
#     pub.publish(twist_msg)

#     # Sleep to control publishing frequency (if necessary)
#     rospy.sleep(0.10)  # Publish once per second

# if __name__ == '__main__':
#     try:
#         skid_steering_publisher()
#     except rospy.ROSInterruptException:
#         pass

def calcDriveCommand(img):
    rospy.loginfo("Got it, \n")


#     frBN = fr[:,:,1] < 90
#     frBN = frBN.astype(int)
    
#     # finding the center of road region
#     regions = m.regionprops(frBN)
#     regions.sort(key=lambda x: x.area, reverse=True)

#     roadCntr = regions[0].centroid
#     prevCenter = roadCntr

#     bridge = CvBridge()
# cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')