#!/usr/bin/env python

import rospy
from Botlom.srv import ChangeColor, FlagLocation
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
from colordetect import ColorDetector


pink = ((154, 179), (0, 168), (207, 255))
green = ((73,  96), (0, 255), (0, 255))

convert = CvBridge()


class Eyes:
    '''The eyes of our beloved Botlom.'''

    def __init__(self, node_name):
        self.detectors = [ColorDetector(*green), ColorDetector(*pink)]
        self.locations = [None, None]  # detected locations.

        rospy.init_node(node_name)
        self.image_feed = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=1)
        self.bounding_box = rospy.Publisher('bounding_box', Image, queue_size=1)
        self.mask_pubs = [rospy.Publisher('mask1', Image, queue_size=1),
                          rospy.Publisher('mask2', Image, queue_size=1)]

        rospy.Service('change_color', ChangeColor, self.set_detector)
        rospy.Service('get_location', FlagLocation, self.get_location)
        rospy.loginfo('{0} initialized.'.format(node_name))

        rospy.spin()

    def image_callback(self, image):
        frame = convert.imgmsg_to_cv2(image, "bgr8")

        # Draw bounding boxes on frame and publish.
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        for i, detector in enumerate(self.detectors):
            bbox = detector.bounding_box(hsv_frame)

            self.mask_pubs[i].publish(convert.cv2_to_imgmsg(detector.mask, 'mono8'))
            if bbox:
                (x, y, w, h) = bbox
                self.locations[i] = bbox
                cv2.rectangle(frame, (x, y), (x+w, y+h), color=(0, 255, 0))
        img_msg = convert.cv2_to_imgmsg(frame, "bgr8")
        self.bounding_box.publish(img_msg)

    def set_detector(self, request):
        try:
            if request.which != 0 or request.which != 1:
                raise ValueError('Invalid color detector {0}.\nCan only change detectors 0 or 1'.format(request.which))

            self.detectors[request.which] = ColorDetector(request.hue, request.saturation, request.value)
            return ''
        except Exception as e:
            return str(e)

    def get_location(self, request):
        try:
            if request.which != 0 or request.which != 1:
                raise ValueError('Invalid color detector {0}.\nCan only get location from 0 or 1.'.format(request.which))

            (x, y, w, h) = self.locations[request.which]
            return x, y, w, h, ''

        except Exception as e:
            return -1, -1, -1, -1, str(e)


if __name__ == "__main__":
    eyes = Eyes('eyes')
