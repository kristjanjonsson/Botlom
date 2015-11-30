#!/usr/bin/env python
import math
import rospy
from Botlom.srv import FlagPosition
from geometry_msgs.msg import Twist, Vector3

info_values = None
pi = math.pi

class Brain:
    def __init__(self, node_name):

        self.target_flag = None
        self.all_flags = None

        rospy.init_node(node_name)
        rospy.loginfo('waiting for service')
        rospy.wait_for_service('get_flag_position')
        rospy.loginfo('Subscribed to magic_sensor topic.')
        self.sensor_handle = rospy.ServiceProxy('get_flag_position', FlagPosition)

        # Will publish in the following topics
        self.velocity_publisher = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # Update all flags
            # TODO : dont this all the time
            self.update_data()
            self.aquire_target()

            if self.target_flag is not None:

                # Align if needed
                self.center_flag(self.target_flag)

                # Move towards it if possible
                self.approach_flag(self.target_flag)

            rate.sleep()

    # chooses a can
    # TODO: separate this into a method that updates all flags
    def aquire_target(self):
        self.sort_flags_by_distance()
        self.target_flag = self.all_flags[0]

    # Update all available flags data
    def update_data(self):
        self.all_flags = []
        flag_num = 0
        while True:
            flag = self.sensor_handle(flag_num)
            if flag.error == "No Flag":
                break
            else:
                self.all_flags.append(flag)
            flag_num += 1

    def sort_flags_by_distance(self):
        sorted(self.all_flags, key = lambda d: d.distance, reverse=True)

    # will make an effort to align robot with flag
    def center_flag(self, flag):
        if not self.is_centered(flag):
            print "gotta align"
            print "angle is currently " + str(flag.angle)
            msg = Twist()
            msg.linear = Vector3(0, 0, 0)
            msg.angular = Vector3(0, 0, flag.angle)
            self.velocity_publisher.publish(msg)

    # get closer to the flag
    def approach_flag(self, flag):
        if self.is_centered(flag):
            print "gotta approach"
            msg = Twist()
            msg.linear = Vector3(1, 0, 0)
            self.velocity_publisher.publish(msg)

    def is_centered(self, flag):
        print "angle is currently " + str(flag.angle)
        return abs(flag.angle) < 0.1

if __name__ == "__main__":
    driver = Brain('brain')
