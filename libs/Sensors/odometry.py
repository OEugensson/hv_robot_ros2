#!/usr/bin/env python
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    PoseWithCovariance,
    Pose,
    Twist,
    Vector3,
    Quaternion,
    Point,
)
import rclpy
import tf2_ros
import sys, os

RESOURCE_PATH = os.path.abspath(os.path.join(sys.path[0], "../../"))
sys.path.append(RESOURCE_PATH)

from libs.Resources import Reader
from libs.Resources.utils import *
from libs.Resources.config import *


class Odometer(Reader):
    ###
    # Odometer class for reading the positional state data and for handling the publishing of data
    ###

    def __init__(self, h, p):
        self.hostname = h
        Reader.__init__(self, p)
        self.pub = node.create_publisher(Odometry, "/" + self.hostname + "/odom/" + resource_name(os.path.commonprefix(Reader.get_probes(self))), 1)

    def publish(self):
        ## handles the publishing of data, should be run at an acceptable update interval
        # get data from queue
        try:
            odom = Odometry()
            odom.header.stamp = node.get_clock().now().to_msg()
            odom.header.frame_id = self.hostname

            [X, Y, H, V, W] = list(map(self.queue.get, Reader.get_probes(self)))

            heading = tf2_ros.transformations.quaternion_from_euler(0.0, 0.0, H)
            odom.pose.pose = Pose(
                Point(X, Y, 0.0),
                Quaternion(heading[0], heading[1], heading[2], heading[3]),
            )
            odom.twist.twist = Twist(Vector3(V, 0, 0), Vector3(0, 0, W))
            # publish data
            Reader.publish(self, odom)
        except Exception as e:
            rclpy.logerror("publishing odometry data failed")
            raise e


class OdometryListener(Odometer):
    # Basic Odometer
    def __init__(self, h):
        Odometer.__init__(self, h, [ODOM_X, ODOM_Y, ODOM_H, ODOM_V, ODOM_W])


class OdometryGyroListener(Odometer):
    # Odometer with Gyro corrections
    def __init__(self, h):
        Odometer.__init__(
            self, h, [ODOM_GYRO_X, ODOM_GYRO_Y, ODOM_GYRO_H, ODOM_GYRO_V, ODOM_GYRO_W]
        )
