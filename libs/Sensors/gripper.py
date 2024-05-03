#!/usr/bin/env python
from std_msgs.msg import Bool
import rclpy
import sys, os

RESOURCE_PATH = os.path.abspath(os.path.join(sys.path[0], "../../"))
sys.path.append(RESOURCE_PATH)

from libs.Resources import Reader
from libs.Resources.utils import *
from libs.Resources.config import *


class GripperListener(Reader):
    ###
    # Gripper information class extending the getter, for handling the publishing of data
    ###

    def __init__(self, h):
        self.hostname = h
        Reader.__init__(self, [HAS_POT])
        self.pub = node.create_publisher(Bool, "/" + self.hostname + "/gripper/" + resource_name(Reader.get_probes(self)), 1)

    def publish(self):
        ## handles the publishing of data, should be run at an acceptable update interval
        # get data from queue
        data = Bool(self.queue[HAS_POT])
        # publish data
        Reader.publish(self, data)
