#!/usr/bin/env python
import rclpy
from mp_interfaces.srv import Toggle
from mp_interfaces.srv import SetTarget
import sys, os

RESOURCE_PATH = os.path.abspath(os.path.join(sys.path[0], "../../"))
sys.path.append(RESOURCE_PATH)

from libs.Resources import Writer
from libs.Resources.utils import *
from libs.Resources.config import *

# probe id 3486: behaviors.spacing.pick.pickTarget.x type float length 4
# probe id 3487: behaviors.spacing.pick.pickTarget.y type float length 4
# probe id 3304: behaviors.spacing.runCollection type bool length 1
# probe id 2438: loc.actual.x type float length 4
# probe id 2439: loc.actual.y type float length 4
SET_X_LOC_CODE = 2438
SET_Y_LOC_CODE = 2439
SET_H_LOC_CODE = 2440


class PickTargetService(Writer):
    def __init__(self):
        # contstruct basics for
        Writer.__init__(self, [BOT_GOT_POT, SET_PICK_X, SET_PICK_Y])
        self.serv_set = node.create_service(mp_set_target, "SetPickTarget", self.set_pick_target)

    def stop(self):
        try:
            self.serv_set.shutdown("end of life")
            print("pick target service ended")
            return True
        except:
            print("pick target service shutdown failed")
            return False

    def set_pick_target(self, req):
        # set values for target
        Writer.callback(self, [0, req.x, req.y])
        return "suceess: set target to (" + str(req.x) + "," + str(req.y) + ")"


class TogglePickTargetService(Writer):
    def __init__(self):
        # contstruct basics for
        Writer.__init__(self, [RUN_SPACING])
        self.serv_run = node.create_service(mp_toggle, "RunPick", self.toggle_pick)

    def stop(self):
        try:
            self.serv_run.shutdown("end of life")
            print("pick target service ended")
            return True
        except:
            print("pick target service shutdown failed")
            return False

    def toggle_pick(self, req):
        # toggle follow person on/off
        Writer.callback(self, 1 if req.val == 1 else 0)
        return "success: value set to " + ("true" if req.val == 1 else "false")
