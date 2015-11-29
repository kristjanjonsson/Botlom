#!/usr/bin/env python

import math
import rospy
import tf
from botlom.srv import CanPosition, CanPositionResponse
from block_finder import get_blocks
from nav_msgs.msg import Odometry

x, y = None, None
roll, pitch, yaw = None, None, None

relative_positions = None

class MagicSensor:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        rospy.Subscriber('base_pose_ground_truth', Odometry, update_pose)
        rospy.Service('get_can_position', CanPosition, get_can_position)

        # need the spin...
        rospy.spin()

        # DEBUG STUFF
        # rate = rospy.Rate(10) # 10hz
        # while not rospy.is_shutdown():
        #     print "x = " + str(x)
        #     print "y = " + str(y)
        #     print "roll = " + str(roll)
        #     print "pitch = " + str(pitch)
        #     print "yaw = " + str(yaw)
        #     get_relative_block_position(3)
        #     print "Relative Positions = " + str(relative_positions)
        #     print " ===== "
        #     rate.sleep()

# get the relative block position the closest block_num blocks
def get_relative_block_position(block_num):
    global relative_positions
    relative_positions = []
    blocks = get_blocks()[0:block_num]

    if x is not None and y is not None:
        for block in blocks:
            x_diff = x - block[0]
            y_diff = y - block[1]
            distance = math.sqrt(x_diff**2 + y_diff**2)
            angle = -yaw

            # Currently: how much turn left.
            # TODO: make left turns when it makes more sense to do so.
            if x_diff < 0 and y_diff < 0 :
                theta = math.radians(math.atan(y_diff / x_diff)) / math.pi
                angle += theta
            if x_diff > 0 and y_diff < 0:
                theta = math.radians(math.atan(x_diff / y_diff)) / math.pi
                angle += theta + 0.5
            if x_diff < 0 and y_diff > 0:
                theta = math.radians(math.atan(x_diff / y_diff)) / math.pi
                angle += theta + 0.75
            if x_diff > 0 and y_diff > 0 :
                theta = math.radians(math.atan(y_diff / x_diff)) / math.pi
                angle += theta +  1

            relative_positions.append([distance, angle])

def update_pose(ground_truth):
    global x, y
    global roll, pitch, yaw
    x, y = get_robot_position(ground_truth)
    roll, pitch, yaw = get_robot_rotation(ground_truth)

def get_robot_position(ground_truth):
    x = ground_truth.pose.pose.position.x
    y = ground_truth.pose.pose.position.y
    return x, y

def get_robot_rotation(ground_truth):
    quat_x = ground_truth.pose.pose.orientation.x
    quat_y = ground_truth.pose.pose.orientation.y
    quat_z = ground_truth.pose.pose.orientation.z
    quat_w = ground_truth.pose.pose.orientation.w

    # Convert quaternions to Euler angles.
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([quat_x, quat_y, quat_z, quat_w])
    return roll / math.pi , pitch / math.pi, yaw / math.pi

def get_can_position(request):
    get_relative_block_position(1)
    block = relative_positions[0]
    return CanPositionResponse(block[0], block[1], "Color", "No Errors")

if __name__ == "__main__":
    driver = MagicSensor('magic_sensor')
