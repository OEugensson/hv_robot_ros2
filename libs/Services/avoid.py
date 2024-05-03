#!/usr/bin/env python
import rclpy
from mp_interfaces.srv import Toggle
import sys, os

RESOURCE_PATH = os.path.abspath(os.path.join(sys.path[0], "../../"))
sys.path.append(RESOURCE_PATH)

from libs.Resources import Writer
from libs.Resources.utils import *
from libs.Resources.config import *

# probe id 3601: behaviors.scriptAvoidCollision.isEnabled type bool length 1
# probe id 3602: behaviors.script.run type bool length 1

AVOID_CODE = 3601
PSEUDO_START_CODE = 1125


class AvoidService(Writer):
    def __init__(self):
        # contstruct basics for
        Writer.__init__(self, [AVOID])
        self.serv = node.create_service(mp_toggle, "AvoidCollision", self.toggle_avoid)

    def stop(self):
        try:
            self.serv.shutdown("end of life")
            print("avoid collision service ended")
            return True
        except:
            print("avoid collision service shutdown failed")
            return False

    def toggle_avoid(self, req):
        # toggle follow person on/off
        Writer.callback(self, 1 if req.val == 1 else 0)
        return "success: value set to " + ("true" if req.val == 1 else "false")
