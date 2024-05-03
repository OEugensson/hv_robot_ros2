#!/usr/bin/env python
from std_msgs.msg import Float32
import rclpy
import sys, os

RESOURCE_PATH = os.path.abspath(os.path.join(sys.path[0], "../../"))
sys.path.append(RESOURCE_PATH)

from libs.Resources import Reader
from libs.Resources.utils import *
from libs.Resources.config import *


class IrListener(Reader):
    ###
    # IR sensor class extending the getter, for handling the publishing of data
    ###

    def __init__(self, h, p):
        self.hostname = h
        Reader.__init__(self, p)
        self.pub = node.create_publisher(Float32, "/" + self.hostname + "/gripper/" + resource_name(os.path.commonprefix(self.probes)), 1)

    def publish(self):
        ## handles the publishing of data, should be run at an acceptable update interval
        # get data from queue
        try:
            data_list = [self.queue[label] for label in self.probes]
            data = sum(data_list) / len(data_list)
            # publish data
            Reader.publish(self, data)
        except Exception as e:
            rclpy.logerr("IR Sensor publish failed")
            raise e


class IrListenerRightFront(IrListener):
    def __init__(self, h):
        IrListener.__init__(self, h, [IR_RIGHT_FRONT_A, IR_RIGHT_FRONT_B])


class IrListenerLeftFront(IrListener):
    def __init__(self, h):
        IrListener.__init__(self, h, [IR_LEFT_FRONT_A, IR_LEFT_FRONT_B])


class IrListenerRightBack(IrListener):
    def __init__(self, h):
        IrListener.__init__(self, h, [IR_RIGHT_BACK_A, IR_RIGHT_BACK_B])


class IrListenerLeftBack(IrListener):
    def __init__(self, h):
        IrListener.__init__(self, h, [IR_LEFT_BACK_A, IR_LEFT_BACK_B])
