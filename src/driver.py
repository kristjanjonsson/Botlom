#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

info_values = None

class Driver:
    def __init__(self, node_name, implementation):
        rospy.init_node(node_name)


def get_info(info):
    global info_values
    info_values = info

if __name__ == "__main__":
    driver = Driver('driver', 'virtual')
