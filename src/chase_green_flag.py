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
        self.is_centered = False
        self.is_close = False

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # Look at flag
            self.flag_position =  self.get_location_service(self.target_color)

            # Center on flag
            self.center_on_flag()

            # Move to flag
            self.approach_flag()

            # Make turn
            # TODO: This

            # Change target color
            # TODO: This

            rate.sleep()

    # Center on flag if needed.
    def center_on_flag(self):
        self.is_centered = False
        flag_center = self.flag_position.x + (self.flag_position.w/2.0)

        # Some slack in our calculations
        X_SLACK = 0.15 * X_MID

        if (X_MID - X_SLACK) < flag_center < (X_MID + X_SLACK):
            self.is_centered = True
            print "Flag is centered!"

        elif flag_center > (X_MID + X_SLACK):
            self.controller.turn_right(0.1, 25)
            self.is_centered = False
            print "Gotta turn right..."
        else:
            self.controller.turn_left(0.1, 25)
            self.is_centered = False
            print "Gotta turn left..."

    def approach_flag(self):
        if not self.is_centered:
            print "I won't even try to approach it now..."
        else:
            self.is_close = False
            flag_width = self.flag_position.w

            # Some slack in our calculations
            X_SLACK = 0.15 * X_MID

            if flag_width > X_MID - X_SLACK:
                self.is_close = True
                print "Pretty close to a flag now."
            else:
                self.controller.move_forward(25)
                self.is_close = False
                print "Gotta move closer to the flag..."


if __name__ == "__main__":
    driver = ControllerTest('chase_green_flag')
