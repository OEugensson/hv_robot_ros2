#!/usr/bin/env python
import rclpy
from std_msgs.msg import Float32
import sys, os

RESOURCE_PATH = os.path.abspath(os.path.join(sys.path[0], "../../"))
sys.path.append(RESOURCE_PATH)

from libs.Resources import Writer


class ArmController(Writer):
    ###
    # Controller for the robot arm
    # Extends the Setter class
    ###
    def __init__(self, hostname):
        # Create writer for this probe
        # manip.armRequestedTargetPositionInPercent
        Writer.__init__(self, ["manip.armRequestedTargetPositionInPercent"])
        # runs a subscriber for setting the arm position.
        rclpy.create_subscription(Float32, "/" + hostname + "/set_arm_pos", self.callback, 10)

    def callback(self, cmd_msg):
        # callback function - extends the setter.callback functionality
        # formats data to be passed to mindprobe
        pos = cmd_msg.data
        if pos > 100:
            pos = 100
        elif pos < 0:
            pos = 0
        Writer.callback(self, pos)
