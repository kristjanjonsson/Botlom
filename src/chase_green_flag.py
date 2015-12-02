#!/usr/bin/env python

from controller import *
from Botlom.srv import ChangeColor, FlagLocation
import rospy


# Camera resolution
X_RES = 640
Y_RES = 480

# Positioning constants
X_MID = X_RES / 2
Y_MID = Y_RES /2
X_SLACK = 0.05 * X_MID
Y_SLACK = 0.05 * Y_MID

class ControllerTest:
    def __init__(self, node_name):
        rospy.init_node(node_name)

        # For checking flag positions
        rospy.wait_for_service('get_flag_position')
        self.get_location_service = rospy.ServiceProxy('get_location', FlagLocation)
        rospy.loginfo('Got handle toget_location service.')

        # Get controller
        self.controller = Controller(rospy)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # Center on flag
            self.center_on_flag()

            rate.sleep()

    # Center on flag if needed
    # returns whether or not we are centerd.
    def center_on_flag(self):
        centered_on_flag = False
        while not centered_on_flag:

            flag_position =  self.get_location_service(0) # 0 is green
            flag_center = flag_position.x + (flag_position.w/2.0)

            # Centered
            if (X_MID - X_SLACK) < flag_center < (X_MID + X_SLACK):
                centered_on_flag = True

            elif flag_center > (X_MID + X_SLACK):
                self.controller.turn_right(0.1, 100)
            else:
                self.controller.turn_left(0.1, 100)


if __name__ == "__main__":
    driver = ControllerTest('chase_green_flag')