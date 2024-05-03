#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import sys, os

RESOURCE_PATH = os.path.abspath(os.path.join(sys.path[0], "../../"))
sys.path.append(RESOURCE_PATH)

from libs.Resources import Writer
from libs.Resources.utils import *


class DriveController(Writer):
    ###
    # Controller for the robot drive
    # extends the Writer parent class
    # handles differential drive controll for the robot
    ###
    def __init__(self, hostname):
        print("test5")
        Writer.__init__(
            self, ["mob.driveControlWheelSpeeds.l", "mob.driveControlWheelSpeeds.r"]
        )
        print("test2")
        # runs a subscriber for setting the wheel velocity.
        #Denna rad var ursprungligen: rospy.Subscriber("/" + hostname + "/cmd_vel", Twist, self.callback), vilket inte bör fungera då du inte fångar subscription, så detta är högst förvirrande att översätta
        Node.create_subscription(Twist,"/" + hostname + "/cmd_vel", self.callback, 10)
        #subscription = Node.create_subscription(Twist,"/hai-1095.local/cmd_vel", self.callback, 10)
        #Vi kommer ej hit om någon av ovan rader är avkommenterade
        print("test")

    def callback(self, cmd_msg):
        # differential drive callback.
        vel = twist_to_vel(cmd_msg)
        Writer.callback(self, vel)
