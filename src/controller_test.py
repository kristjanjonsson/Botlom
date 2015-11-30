#!/usr/bin/env python

from controller import *
import time
import rospy

class ControllerTest:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        controller = Controller(rospy)

        time.sleep(1)
        print "Will now test moving forwards"
        controller.move_forward(1)

        time.sleep(1)
        print "Will now test moving backwards"
        controller.move_backwards(1)

        time.sleep(1)
        print "Will now test moving sideways"
        controller.turn_left(1)
        controller.move_forward(1)
        time.sleep(2)
        controller.turn_left(2)
        controller.move_backwards(1)

        time.sleep(1)
        print "Will now play music"
        controller.play_song()

        print "See ya!"

if __name__ == "__main__":
    driver = ControllerTest('controller_test')