#!/usr/bin/env python

from controller import Controller
from Botlom.srv import FlagLocation
import rospy


# Camera resolution
X_RES = 640
Y_RES = 480

# Positioning constants
X_MID = X_RES / 2
Y_MID = Y_RES / 2


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
        self.tried_recovery = False
        self.total_flags = 6
        self.completed_flags = 0

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # Look at flag
            self.flag_position = self.get_location_service(self.target_color)

            # Do I see a flag?
            if self.flag_position.x == -1 and self.flag_position.y == -1:
                print "Where is the flag?"
                if not self.tried_recovery:
                    self.try_recovery()
                    rate.sleep()
                    continue
                else:
                    print "Still couldn't find the flag. I will stop now."
                    break
            # If i got here, I may try recovery again soon.
            self.tried_recovery = False

            # Center on flag
            self.center_on_flag()

            # Move to flag
            self.approach_flag()

            # Turn around flag
            if self.is_close:
                self.turn_around_flag()
                self.find_next_flag()

            # Sweet success:
            if self.completed_flags == self.total_flags:
                self.controller.play_song()
                while True:
                    self.controller.turn_left(0.25, 200)

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
            self.controller.turn_right(0.1, 25)
            self.is_centered = False
            print "Gotta turn right..."
        else:
            self.controller.turn_left(0.1, 25)
            self.is_centered = False
            print "Gotta turn left..."

    # approach if possible
    def approach_flag(self):

        # only approach if centered...
        if self.is_centered:
            self.is_close = False
            flag_width = self.flag_position.w

            # Some slack in our calculations
            X_SLACK = 0.0 * X_MID

            if flag_width > (X_RES/3) - X_SLACK:
                self.is_close = True
                print "Pretty close to a flag now."
            else:
                self.controller.move_forward(0.5, 200)
                self.is_close = False
                print "Gotta move closer to the flag...width is " + str(flag_width)

    def turn_around_flag(self):
        print "Time to turn around the flag!"
        if self.target_color == 1:
            # we go to the right of greens
            self.controller.circle_left(6, 200)
        else:
            # we go to the left of reds.
            self.controller.circle_right(6, 200)

        # another flag down
        self.completed_flags += 1

    # finds next flag after turn.
    # target i
    def find_next_flag(self):

        # Swap color, reset values
        self.target_color = (self.target_color + 1) % 2
        self.is_centered = False
        self.is_close = False

        print "Where is the next flag?"
        while True:
            self.flag_position = self.get_location_service(self.target_color)
            if self.flag_position.x != -1 and self.flag_position.y != -1:
                print "Found it"
                break
            else:
                if self.target_color == 1:
                    # we should look mostly to the right to find next green flag.
                    self.controller.turn_left(0.1, 75)
                else:
                    # we should look mostly to the left to find next green flag.
                    self.controller.turn_right(0.1, 75)

    def try_recovery(self):
        print "I will now try to recover"

        self.controller.move_backwards(1, 100)
        if self.flag_visible(self.target_color):
            return

        # self.controller.turn_left(2, 50)
        # if self.flag_visible(self.target_color):
        #     return

        self.controller.move_backwards(1, 100)
        if self.flag_visible(self.target_color):
            return

        # self.controller.turn_right(2, 50)
        # if self.flag_visible(self.target_color):
        #     return

        self.controller.move_backwards(1, 100)
        if self.flag_visible(self.target_color):
            return

        # self.controller.turn_right(2, 50)
        # if self.flag_visible(self.target_color):
        #     return

        # will stop trying
        self.tried_recovery = True

    def flag_visible(self, color):
        flag_position = self.get_location_service(color)
        return flag_position.x != -1 and flag_position.y != -1


if __name__ == "__main__":
    driver = ControllerTest('chase_green_flag')
