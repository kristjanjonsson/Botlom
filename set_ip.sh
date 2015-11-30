#!/bin/sh
export ROS_HOSTNAME=`hostname -I | awk '{print $1}'`
export ROS_IP=`hostname -I | awk '{print $1}'`
