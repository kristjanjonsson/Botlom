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
        rospy.wait_for_service('get_location')
        self.get_location_service = rospy.ServiceProxy('get_location', FlagLocation)
        rospy.loginfo('Got handle toget_location service.')

        # Get controller
        self.controller = Controller(rospy)

        # Initially go to green flag
        # 0 is green
        # 1 is red
        self.target_color = 0

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # Center on flag
            self.center_on_flag()

            # Move to flag
            self.approach_flag()

            # Make turn
            # TODO: This

            # Change target color
            # TODO: This

            rate.sleep()

    # Center on flag if needed
    def center_on_flag(self):
        centered_on_flag = False
        while not centered_on_flag:

            flag_position =  self.get_location_service(self.target_color) # 0 is green
            flag_center = flag_position.x + (flag_position.w/2.0)

            # Centered
            if (X_MID - X_SLACK) < flag_center < (X_MID + X_SLACK):
                centered_on_flag = True

            elif flag_center > (X_MID + X_SLACK):
                self.controller.turn_right(0.1, 25)
            else:
                self.controller.turn_left(0.1, 25)

    def approach_flag(self):
        close_to_flag = False
        while not close_to_flag:

            flag_position =  self.get_location_service(self.target_color) # 0 is green
            flag_width = flag_position.w

            if flag_width > X_MID  + X_SLACK:
                close_to_flag = True
            else:
                self.controller.move_forward(25)



if __name__ == "__main__":
    driver = ControllerTest('chase_green_flag')
