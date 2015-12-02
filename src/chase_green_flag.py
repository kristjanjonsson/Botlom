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

            # Do I see a flag?
            if self.flag_position.x == -1 and self.flag_position.y == -1:
                print "Where is the flag?"
                #TODO: something smarter here
                rate.sleep()
                break

            # Center on flag
            self.center_on_flag()

            # Move to flag
            self.approach_flag()

            # Turn around flag
            if self.is_close:
                self.turn_around_flag()
            #
                #self.find_next_flag()
                #
                #
                # TODO: Sorta find next flag.
                #
                ## reset closeness and centeredeness.
                # self.is_centered = False
                # self.is_close = False

            rate.sleep()

    # Center on flag if needed.
    def center_on_flag(self):
        self.is_centered = False
        flag_center = self.flag_position.x + (self.flag_position.w/2.0)

        # Some slack in our calculations
        X_SLACK = 0.10 * X_MID

        if (X_MID - X_SLACK) < flag_center < (X_MID + X_SLACK):
            self.is_centered = True
            print "Flag is centered!"

        elif flag_center > (X_MID + X_SLACK):
            self.controller.turn_right(0.1, 10)
            self.is_centered = False
            print "Gotta turn right..."
        else:
            self.controller.turn_left(0.1, 10)
            self.is_centered = False
            print "Gotta turn left..."

    # approach if possible
    def approach_flag(self):

        # only approach if centered...
        if self.is_centered:
            self.is_close = False
            flag_width = self.flag_position.w

            # Some slack in our calculations
            X_SLACK = 0.10 * X_MID

            if flag_width > (X_RES/2) - X_SLACK:
                self.is_close = True
                print "Pretty close to a flag now."
            else:
                self.controller.move_forward(0.5, 200)
                self.is_close = False
                print "Gotta move closer to the flag...width is " + str(flag_width)

    def turn_around_flag(self):
        if self.target_color == 1:
            # we go to the right of greens
            self.controller.circle_left(5,100)
        else:
            # we go to the left of reds.
            self.controller.circle_right(5, 100)

    # finds next flag after turn.
    # target i
    def find_next_flag(self):

        #swap color
        self.target_color =  (self.target_color + 1) % 2

        if self.target_color == 0:
            pass
            # was pink, so we went through its left side.
            # we should look mostly to the left to find next green flag.
            # TODO: This
        else:
            pass
            # was green, so we went through its right side.
            # we should look mostly to the right to find next green flag.
            # TODO: This




if __name__ == "__main__":
    driver = ControllerTest('chase_green_flag')
