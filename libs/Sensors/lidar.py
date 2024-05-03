#!/usr/bin/env python
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2 as pc2
from geometry_msgs.msg import PointStamped
import rclpy
import struct
import sys, os
import numpy as np
import traceback

RESOURCE_PATH = os.path.abspath(os.path.join(sys.path[0], "../../"))
sys.path.append(RESOURCE_PATH)

from libs.Resources import Reader
from libs.Resources.utils import *
from libs.Resources.config import *


class LidarListener(Reader):
    ###
    # Lidar sensor class extending the getter, for handling the publishing of data
    ###

    def __init__(self, h):
        self.hostname = h
        Reader.__init__(self, [GLOBAL_SCAN])
        self.pub = node.create_publisher(PointCloud2, "/" + self.hostname + "/lidar/" + resource_name(Reader.get_probes(self)), 1 )

    def publish(self):
        ## handles the publishing of data, should be run at an acceptable update interval
        # get data from queue
        try:
            data = self.queue[GLOBAL_SCAN]

            # print data
            if type(data) != str:
                raise TypeError("Data not in correct format!")
            # get the data from the message
            data = data.split("v=")[-1]
            if data == "":
                raise TypeError("lidar data is empty")

            # create x and y arrays from the base16 string of data
            strarr = data.split(":")
            datStr = ""
            # convert hex numbers to char string
            for elem in strarr:
                if elem != "":
                    datStr += chr(int(elem, 16))

            # parse char string as floats
            dat = [
                struct.unpack("<f", datStr[i : i + 4]) for i in range(0, len(datStr), 4)
            ]

            # fix all nan or inf values
            dat = list(np.nan_to_num(np.array(dat)))

            # parse float array to pointcloud xyz matrix
            elem_arr = []
            j = 0
            for i in range(0, len(dat), 3):
                if len(dat[i:]) < 3:
                    break
                elem_arr.append([dat[i], dat[i + 1], 0.05])
            elem_arr = np.array(elem_arr)

            header = Header()
            header.stamp = node.get_clock().now().to_msg()
            header.frame_id = self.hostname

            msg = pc2.create_cloud_xyz32(header, elem_arr)

            # publish data
            Reader.publish(self, msg)

        except TypeError as e:
            pass
            # rlcpy.logwarn(e)
        except Exception as e:
            # well this is sad
            rclpy.logerr("error publishing lidar data")
            raise e


class LidarNObsListener(Reader):
    ###
    # Lidar sensor class to get the nearest object and for handling the publishing of data
    ###
    def __init__(self, h):
        self.hostname = h
        Reader.__init__(self, [OBSTACLE_X, OBSTACLE_Y])
        self.pub = node.create_publisher(PointStamped, "/" + self.hostname + "/lidar/" + resource_name(os.path.commonprefix(Reader.get_probes(self))), 1)

    def publish(self):
        ## handles the publishing of data, should be run at an acceptable update interval
        # get data from queue
        try:
            x = float(self.queue[OBSTACLE_X])
            y = float(self.queue[OBSTACLE_Y])
            z = 0.0

            msg = PointStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = self.hostname
            msg.point.x = x
            msg.point.y = y
            msg.point.z = z

            # publish data
            Reader.publish(self, msg)

        except Exception as e:
            # well this is sad
            rlcpy.logerr("error publishing nearest object data")
            raise e
