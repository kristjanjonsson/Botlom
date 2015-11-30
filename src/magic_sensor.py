#!/usr/bin/env python

import math
import rospy
import tf
from Botlom.srv import FlagPosition, FlagPositionResponse
from block_finder import get_flags
from nav_msgs.msg import Odometry

x, y = None, None
roll, pitch, yaw = None, None, None

class MagicSensor:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        rospy.Subscriber('base_pose_ground_truth', Odometry, update_robot_pose)
        rospy.Service('get_flag_position', FlagPosition, get_flag_coordinates)

        # need the spin...
        rospy.spin()

# get the relative block position the closest block_num blocks
def get_relative_flag_info():
    flag_positions = []
    flags = get_flags()[0:]

    # Make sure ground_truth is available
    if x is not None and y is not None:
        for flag in flags:
            x_diff = abs(x - flag[0])
            y_diff = abs(y - flag[1])
            distance = math.sqrt(x_diff**2 + y_diff**2)
            angle = -yaw

            # avoiding zero division problems
            if x_diff == 0:
                x_diff += 0.0001
            if y_diff == 0:
                y_diff += 0.0001

            # Currently: how much turn left.
            # TODO: make left turns when it makes more sense to do so.
            if x < flag[0] and y < flag[1]:
                theta = math.radians(math.atan(y_diff / x_diff)) / math.pi
                angle += theta
            if x > flag[0] and y < flag[1]:
                theta = math.radians(math.atan(x_diff / y_diff)) / math.pi
                angle += theta + 0.5
            if x < flag[0] and y > flag[1]:
                theta = math.radians(math.atan(y_diff / y_diff)) / math.pi
                angle += theta + 1.5
            if x > flag[0] and y > flag[1]:
                theta = math.radians(math.atan(y_diff / x_diff)) / math.pi
                angle += theta +  1

            if angle > 1:
                print "change because angle is " + str(angle)
                angle = - (2 - angle)

            flag_positions.append([distance, angle, flag[-1]])
            print "yaw  = " + str(yaw)
            print "robot X = " + str(x)
            print "robot y = " + str(y)
            print "flag x = " + str(flag[0])
            print "flag y = " + str(flag[1])
            print "theta = " + str(theta)
            print ""
            break

    return flag_positions


def update_robot_pose(ground_truth):
    global x, y
    global roll, pitch, yaw
    x, y = get_robot_coordinates(ground_truth)
    roll, pitch, yaw = get_robot_rotation(ground_truth)

def get_robot_coordinates(ground_truth):
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

def get_flag_coordinates(request):
    flags = get_relative_flag_info()
    flag_num = request.flag_num
    if flag_num == len(flags):
        return FlagPositionResponse(0.0, 0.0, "No Flag", "No Flag")
    else:
        flag = flags[flag_num]
        return FlagPositionResponse(flag[0], flag[1], flag[2], "No Errors")

if __name__ == "__main__":
    driver = MagicSensor('magic_sensor')
