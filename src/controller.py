#!/usr/bin/env python

import rospy
from Botlom.srv import CommandService
import time

class Controller:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        rospy.wait_for_service('command_receiver')
        self.command_handle = rospy.ServiceProxy('command_receiver', CommandService)

    def move_forward(self, press_time = 0.25):
        self.command_handle("UP", "2")
        time.sleep(press_time)
        response = self.command_handle("UP", "3")

        if response.error:
            print response.error

    def move_backwards(self, press_time = 0.25):
        self.command_handle("DOWN", "2")
        time.sleep(press_time)
        response = self.command_handle("DOWN", "3")

        if response.error:
            print response.error

    def turn_left(self, press_time = 0.25):
        self.command_handle("LEFT", "2")
        time.sleep(press_time)
        response = self.command_handle("LEFT", "3")

        if response.error:
            print response.error

    def turn_right(self, press_time = 0.25):
        self.command_handle("RIGHT", "2")
        time.sleep(press_time)
        response = self.command_handle("RIGHT", "3")

        if response.error:
            print response.error

    def play_song(self, press_time = 0.25):
        self.command_handle("S", "2")
        time.sleep(press_time)
        response = self.command_handle("S", "3")

        if response.error:
            print response.error


    # A handler for keyboard events. Feel free to add more!
    def callbackKey(self, event):
        response = self.command_handle(k, event.type)
        if response.error:
            print response.error
            print

if __name__ == "__main__":
    app = Controller('create_controller')
