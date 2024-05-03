#!/usr/bin/env python
import rclpy
from std_msgs.msg import Float32
import sys, os

RESOURCE_PATH = os.path.abspath(os.path.join(sys.path[0], "../../"))
sys.path.append(RESOURCE_PATH)

from libs.Resources import Writer


class GripperController(Writer):
    ###
    # Controller for the robot gripper
    # Extends the Setter class
    ###
    def __init__(self, hostname):
        # Create writer for this probe
        # manip.gripperRequestedTargetPositionInPercent
        Writer.__init__(self, ["manip.gripperRequestedTargetPositionInPercent"])
        # intializes the mp connection and runs a subscriber for setting the gripper position.
        rclpy.create_subscription(Float32,"/" + hostname + "/set_gripper_pos", self.callback, 10)

    def callback(self, cmd_msg):
        # callback function - extends the setter.callback functionality
        # formats data to be passed to mindprobe
        pos = cmd_msg.data
        if pos > 100:
            pos = 100
        elif pos < 0:
            pos = 0
        Writer.callback(self, pos)
