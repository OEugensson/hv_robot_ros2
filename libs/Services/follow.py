#!/usr/bin/env python
import rclpy
from mp_interfaces.srv import Toggle
import sys, os

RESOURCE_PATH = os.path.abspath(os.path.join(sys.path[0], "../../"))
sys.path.append(RESOURCE_PATH)

from libs.Resources import Writer
from libs.Resources.utils import *
from libs.Resources.config import *

# probe id 3842: behaviors.followMe.run type bool length 1


class FollowService(Writer):
    def __init__(self):
        # contstruct basics for
        Writer.__init__(self, [FOLLOW_ME])
        self.serv = node.create_service(Toggle, "FollowMe", self.toggle_follow)

    def stop(self):
        try:
            self.serv.shutdown("end of life")
            print("follow me service ended")
            return True
        except:
            print("follow me service shutdown failed")
            return False

    def toggle_follow(self, req):
        # toggle follow person on/off
        Writer.callback(self, 1 if req.val == 1 else 0)
        return "success: value set to " + ("true" if req.val == 1 else "false")
