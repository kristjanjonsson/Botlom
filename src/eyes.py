#!/usr/bin/env python

import rospy
# from Botlom.srv import KeyboardService
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
# from colordetect import ColorDetector


class Eyes:
    '''The eyes of our beloved Botlom.'''

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self.bridge = CvBridge()
        self.raw = rospy.Publisher('raw', Image, queue_size=1)
        self.cap = cv2.VideoCapture(0)  # Does this work or do we need a ROS video nodes?

        rospy.loginfo('{0} initialized.'.format(node_name))

    def watch(self):
        rate = rospy.Rate(24)
        while not rospy.is_shutdown():
            success, img = self.cap.read()
            if not success:
                rospy.loginfo('Could not read image.')
                break

            img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            self.raw.publish(image)

            rate.sleep()



if __name__ == "__main__":
    eyes = Eyes('Eyes')
    eyes.watch()

