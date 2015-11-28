#!/usr/bin/env python

import rospy
from Botlom.srv import ChangeColor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
from colordetect import ColorDetector


green = ((59, 85), (0, 255), (0, 255))


class Eyes:
    '''The eyes of our beloved Botlom.'''

    def __init__(self, node_name):
        self.detectors = [ColorDetector(*green), ColorDetector(*green)]
        self.cap = cv2.VideoCapture(0)  # Does this work or do we need a ROS video nodes?

        rospy.init_node(node_name)
        self.bridge = CvBridge()
        self.raw = rospy.Publisher('raw', Image, queue_size=1)
        self.bounding_box = rospy.Publisher('bounding_box', Image, queue_size=1)

        rospy.Service('motion_mode_keyboard', ChangeColor, self.set_detector)
        rospy.loginfo('{0} initialized.'.format(node_name))

    def watch(self):
        rate = rospy.Rate(24)
        while not rospy.is_shutdown():
            success, frame = self.cap.read()
            if not success:
                rospy.loginfo('Could not read image.')
                break

            # Publish the raw frame.
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.raw.publish(img_msg)

            # Draw bounding boxes on frame and publish.
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            for i in range(2):
                bbox = self.detectors[i].bounding_box(hsv_frame)
                if bbox:
                    (x, y, w, h) = bbox
                    cv2.rectangle(frame, (x, y), (x+w, y+h), color=(0, 255, 0))
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.bounding_box.publish(img_msg)

            rate.sleep()
        rospy.loginfo('Shutting down.')

    def set_detector(self, request):
        try:
            if request.which != 0 or request.which != 1:
                raise ValueError('Invalid color detector {0}.\nCan only change detectors 0 or 1'.format(request.which))

            self.detectors[request.which] = ColorDetector(request.hue, request.saturation, request.value)
            return ''
        except Exception as e:
            return str(e)


if __name__ == "__main__":
    eyes = Eyes('Eyes')
    eyes.watch()
