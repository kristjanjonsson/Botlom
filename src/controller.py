#!/usr/bin/env python

import os

from Tkinter import *
import rospy
from Botlom.srv import CommandService

# turn off autorepeat behavior
os.system('xset r off')

VALID_KEY_PRESSES = ['UP', 'DOWN', 'LEFT', 'RIGHT', 'S']

class Controller(Tk):
    def __init__(self, node_name):
        rospy.init_node(node_name)
        rospy.wait_for_service('command_keyboard')
        self.command_handle = rospy.ServiceProxy('command_keyboard', CommandService)


        Tk.__init__(self)
        self.title("Keyboard Control Receiver")
        self.option_add('*tearOff', FALSE)
        self.bind("<Key>", self.callbackKey)
        self.bind("<KeyRelease>", self.callbackKey)

    # A handler for keyboard events. Feel free to add more!
    def callbackKey(self, event):
        k = event.keysym.upper()
        response = self.command_handle(k, event.type)
        if response.error:
            print response.error
            print

if __name__ == "__main__":
    app = Controller('create_controller')
    app.mainloop()
