#!/usr/bin/env python
import rospy
from botlom.srv import CanPosition

info_values = None

class Brain:
    def __init__(self, node_name):
        rospy.init_node(node_name)

        rospy.wait_for_service('get_can_position')
        rospy.loginfo('Subscribed to magic_sensor topic.')

        self.sensor_handle = rospy.ServiceProxy('get_can_position', CanPosition)

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            answer = self.sensor_handle()
            print answer
            rate.sleep()

    # will make a wide scan and choose closest can
    def choose_target_can(self):
        pass

    # will make an effort to align robot with can
    def center_can(self):
        pass

    # get closer to the can
    def approach_can(self):
        pass

if __name__ == "__main__":
    driver = Brain('brain')
