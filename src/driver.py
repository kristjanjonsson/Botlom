#!/usr/bin/env python

import rospy
from Botlom.srv import CommandService
import serial
import struct

VELOCITYCHANGE = 400
ROTATIONCHANGE = 600

class Driver:

    def __init__(self, node_name):
        self.callbackKeyUp = False
        self.callbackKeyDown = False
        self.callbackKeyLeft = False
        self.callbackKeyRight = False
        self.callbackKeyBeep = False
        self.callbackKeyLastDriveCommand = ''

        # connecting
        self.connection = None
        self.connectToRobot()

        # quick hack to make robot responsive
        self.setSafe()

        # start node and spin.
        rospy.init_node(node_name)
        rospy.Service('command_receiver', CommandService, self.command_callback)
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            velocity = 0
            velocity += VELOCITYCHANGE if self.callbackKeyUp is True else 0
            velocity -= VELOCITYCHANGE if self.callbackKeyDown is True else 0
            rotation = 0
            rotation += ROTATIONCHANGE if self.callbackKeyLeft is True else 0
            rotation -= ROTATIONCHANGE if self.callbackKeyRight is True else 0

            # compute left and right wheel velocities
            vr = velocity + (rotation/2)
            vl = velocity - (rotation/2)

            # create drive command
            cmd = struct.pack(">Bhh", 145, vr, vl)
            if cmd != self.callbackKeyLastDriveCommand:
                self.sendCommandRaw(cmd)
                self.callbackKeyLastDriveCommand = cmd

            rate.sleep()

    def command_callback(self, request):
        try:
            self.send_command(request)
        except Exception as error:
            return str(error)

        return ''

    def send_command(self, request):
        k = request.keypress

        if request.type == '2': # KeyPress; need to figure out how to get constant
            if k == 'S': # MUSIC!
                self.playSong()
            elif k == 'UP':
                self.callbackKeyUp = True
            elif k == 'DOWN':
                self.callbackKeyDown = True
            elif k == 'LEFT':
                self.callbackKeyLeft = True
            elif k == 'RIGHT':
                self.callbackKeyRight = True
            else:
                raise ValueError(repr(k), "not handled")
        elif request.type == '3': # KeyRelease; need to figure out how to get constant
            if k == 'UP':
                self.callbackKeyUp = False
            elif k == 'DOWN':
                self.callbackKeyDown = False
            elif k == 'LEFT':
                self.callbackKeyLeft = False
            elif k == 'RIGHT':
                self.callbackKeyRight = False


    def sendCommandASCII(self, command):
        cmd = ""
        for v in command.split():
            cmd += chr(int(v))
        self.sendCommandRaw(cmd)

    def sendCommandRaw(self, command):
        try:
            if self.connection is not None:
                self.connection.write(command)
            else:
                raise ValueError("Not connected.")
        except serial.SerialException:
            print "Lost connection"
            self.connection = None
            raise
        print ' '.join([ str(ord(c)) for c in command ])

    # Automagically connect. Only to TTYUSB0 for now.
    def connectToRobot(self):
        port = "/dev/ttyUSB0"
        print "Trying " + str(port) + "... "
        try:
            self.connection = serial.Serial(port, baudrate=115200, timeout=1)
            print "Connected!"
        except:
            print "Failed to connect to robot."
            print "Either the robot is not connected, or your user does not own the device."
            print "Connect robot or set user as owner of the device."

    def playSong(self):
        self.sendCommandASCII('140 3 15'
                              ' 65 24' + # F
                              ' 65 8' + # F
                              ' 65 32' + # F
                              ' 60 32' + # C
                              ' 69 24' + # A
                              ' 69 8' + # A
                              ' 69 32' + # A
                              ' 65 32' + # F
                              ' 65 24' + # F
                              ' 69 8' + # A
                              ' 72 48' + # C
                              ' 72 16' + # C
                              ' 70 16' + # Bb
                              ' 69 16' + # A
                              ' 67 64' + # G
                              ' 141 3') # Play music

    def setSafe(self):
        # quick dirty hack to make robot responsive.
        self.sendCommandASCII('128') # set passive
        self.sendCommandASCII('131') # set safe

if __name__ == "__main__":
    driver = Driver('create_driver')
