#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3

class Navigator:
    def __init__(self):
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.init_node('Navigator', anonymous=True)
        
    def go_around(self, distance, circle_r, direction, degrees=3.1415, speed = 1):
        '''
        This func would navigate the boltom directly to the target and go around the target, after the direction is set towards the target
        :param distance: distance from boltom's head to the target
        :param circle_r: radius of the circle when going around
        :param direction: 1 means anti-clockwise (from right), -1 means clockwise (from left)
	:param degrees: degrees to spin in the circle
	:speed: speed we want to set, should be 1~10
        '''
        
        msg = Twist()
        frequency = 10 # 10hz
        rate = rospy.Rate(frequency) 

        # Go straight towards target
        msg.linear = Vector3(speed, 0, 0)
        for i in xrange(int((distance-circle_r)*frequency/speed)):
            self.pub.publish(msg)
            rate.sleep()

        # Turn 90 degree
        msg.linear = Vector3(0, 0, 0)
        msg.angular = Vector3(0, 0, -direction*speed)
        pi = 3.1415
        for i in xrange(int(pi/2*frequency/speed)):
            self.pub.publish(msg)
            rate.sleep()

        # Go around the target
        msg.linear = Vector3(speed*circle_r, 0, 0)
        msg.angular = Vector3(0, 0, direction*speed)
        for i in xrange(int(degrees*frequency/speed)):
            self.pub.publish(msg)
            rate.sleep()

        # Stop
        msg.linear = Vector3(0, 0, 0)
        msg.angular = Vector3(0, 0, 0)
        self.pub.publish(msg)

if __name__ == '__main__':
    Navigator().go_around(5, 1, -1, degrees=6.0, speed=5)
