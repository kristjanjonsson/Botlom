#!/usr/bin/env python

import rospy
from Botlom.srv import CommandService
import time

class Controller:
    def __init__(self, rospy):
        # rospy.init_node(node_name)
        rospy.wait_for_service('command_receiver')
        self.command_handle = rospy.ServiceProxy('command_receiver', CommandService)

    def move_forward(self, press_time = 0.25, speed = 400):
        self.command_handle("UP", "2", speed)
        time.sleep(press_time)
        response = self.command_handle("UP", "3", speed)

        if response.error:
            print response.error

    def move_backwards(self, press_time = 0.25, speed = 400):
        self.command_handle("DOWN", "2", speed)
        time.sleep(press_time)
        response = self.command_handle("DOWN", "3", speed)

        if response.error:
            print response.error

    def turn_left(self, press_time = 0.25, speed = 400):
        self.command_handle("LEFT", "2", speed)
        time.sleep(press_time)
        response = self.command_handle("LEFT", "3", speed)

        if response.error:
            print response.error

    def turn_right(self, press_time = 0.25, speed = 400):
        self.command_handle("RIGHT", "2", speed)
        time.sleep(press_time)
        response = self.command_handle("RIGHT", "3", speed)

        if response.error:
            print response.error

    def circle_right(self, press_time = 0.25, speed = 400):

        # must turn before circling
        self.command_handle("LEFT", "2", 400)
        time.sleep("5")
        self.command_handle("LEFT", "3", 400)

        self.command_handle("RIGHT", "2", speed)
        self.command_handle("UP", "2", speed)
        time.sleep(press_time)
        self.command_handle("RIGHT", "3", speed)
        self.command_handle("UP", "3", speed)

    def circle_left(self, press_time = 0.25, speed = 400):

        # must turn before circling
        self.command_handle("RIGHT", "2", 400)
        time.sleep("5")
        self.command_handle("RIGHT", "3", 400)

        self.command_handle("LEFT", "2", speed)
        self.command_handle("UP", "2", speed)
        time.sleep(press_time)
        self.command_handle("LEFT", "3", speed)
        self.command_handle("UP", "3", speed)

    def play_song(self, press_time = 0.25, speed = 400):
        self.command_handle("S", "2", speed)
        time.sleep(press_time)
        response = self.command_handle("S", "3", speed)

        if response.error:
            print response.error


    # A handler for keyboard events. Feel free to add more!
    def callbackKey(self, event, speed):
        response = self.command_handle(k, event.type, speed)
        if response.error:
            print response.error
            print

if __name__ == "__main__":
    app = Controller('create_controller')
