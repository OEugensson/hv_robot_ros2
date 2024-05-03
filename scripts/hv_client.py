import socket
import select
import struct
import code
import threading
import queue
import time
import os
import sys
import subprocess
import traceback
from socket import error as SocketError
import errno
from rclpy.node import Node

# add local python scripts to the path so that they can be iported
sys.path.append(os.path.abspath(os.path.join(sys.path[0], "../")))
sys.path.append(os.path.abspath(os.path.join(sys.path[0], "../srv")))

from libs import *

### --------- ###
#
### Harvest Vehicle Client Connection ###
#
# This is extending the base client code to handle useage specific for bridge features
#
### -------- ###


class HVClient(Client):
    stopped = False
    cmdr = []
    lstnr = []
    srvs = []
    using_lidar = False
    recording = False

    # constructor for the hv bridge instance of the client connection
    def __init__(self, hostname, services):
        self.hostname = hostname
        name = resource_name(hostname)
        if DRIVE_CMD in services:
            # enable drive controller
            self.cmdr.append(DriveController(name))
            ##Vi kommer ej hit
        if GRIPPER_CMD in services:
            # enable gripper controller
            self.cmdr.append(GripperController(name))
        if ARM_CMD in services:
            # enable arm controller
            self.cmdr.append(ArmController(name))

        if ODOM_LSTN in services:
            # enable default odometry listener
            self.lstnr.append(OdometryListener(name))
        if IR_LSTN in services:
            # enable default ir listeners
            self.lstnr.append(IrListenerRightFront(name))
            self.lstnr.append(IrListenerLeftFront(name))
            self.lstnr.append(IrListenerRightBack(name))
            self.lstnr.append(IrListenerLeftBack(name))
        if LIDAR_LSTN in services:
            # enable default lidar listener
            self.using_lidar = True
            self.lstnr.append(LidarListener(name))
        if GRIPPER_LSTN in services:
            # enable default gripper has pot detection
            self.lstnr.append(GripperListener(name))

        if NEAREST_OBS in services:
            # enable nearest obsticale location listener
            self.using_lidar = True
            self.lstnr.append(LidarNObsListener(name))
        if ODOM_GYRO_LSTN in services:
            # enable odometry with gryro correction listener
            self.lstnr.append(OdometryGyroListener(name))

        if FOLLOW_SRV in services:
            # enable odometry with gryro correction listener
            self.srvs.append(FollowService())
            
        if PICK_SRV in services and not FOLLOW_SRV in services:
            # enable odometry with gryro correction listener
            self.srvs.append(PickTargetService())
            self.srvs.append(TogglePickTargetService())
            
        if AVOID_SRV in services and not FOLLOW_SRV in services:
            # enable odometry with gryro correction listener
            self.srvs.append(AvoidService())
            
        
        print("hej")
        Client.__init__(self, hostname)

    def start(self, silent=True):
        # start up ROS bridge mode for robot
        try:
            # this will prevent the robot from emiting beeps
            if silent:
                Client.write_probe(self, BEEP, 0)

            # start the robot
            Client.write_probe(self, START, 1)
            time.sleep(2)

            # this will start collection with the lidar sensor
            if self.using_lidar:
                Client.write_probe(self, LIDAR_STANDBY, 0)
                time.sleep(1)

            # if data needs to be collected from the robot, enable the probes
            if len(self.lstnr) != 0:
                self.enable_services()
            time.sleep(1)

            # Client.start_probes(self)
        except Exception as e:
            err = traceback.format_exc()
            rlcpy.logerr(err)
            raise e

    def stop(self):
        # end everything that is running
        try:
            # turn off ROS services
            self.end_ros_services()
            # sets the start to FALSE. this 'stops' the robot.
            # sets flag pulled to true
            Client.stop_probes(self)
            Client.write_probes(self, [(ESTOP, 1), (FLAG_PULL, 1)])
            print("virtual e-stop triggered")
            time.sleep(1)
            Client.write_probes(self, [(ESTOP, 0), (FLAG_PULL, 0)])
            Client.write_probe(self, START, 0)
            print (str(self.hostname) + " has stopped all running processes")
        except Exception as e:
            rlcpy.logerr(e)
            raise e
        finally:
            # disconnect client connection
            Client.disconnect(self)
            # clear all services
            self.cmdr = []
            self.lstnr = []
            self.srvs = []
            self.stopped = True
            self.recording = False

    def restart(self):
        # stop everything and restart the connection
        self.stop()
        time.sleep(5)
        self.start()

    def enable_services(self):
        # take all probes that need to be listened for and enable them
        try:
            # this will throw an error if the lstnr list is empty
            get_probe_arr = (
                lambda a, b: get_probe_arr(a.union(b[0]), b[1:])
                if len(b) > 1
                else tuple(a.union(b[0]))
            )
            probes = get_probe_arr(
                set([]), map((lambda a: set(a.get_probe_list())), self.lstnr)
            )
            print ("Probes to enable: ")
            print (probes)
            Client.enable_probes(self, probes)
        except Exception as e:
            raise e

    def write(self):
        # gather all messages from controll class instances and write to the robot
        msgs = []
        try:
            get_msg_arr = (
                lambda a, b: get_msg_arr(a + b[0], b[1:]) if len(b) > 1 else a + b[0]
            )
            # collect all subscription messages to write to robot
            if len(self.cmdr) > 0:
                msgs = get_msg_arr([], map((lambda a: a.get_msgs()), self.cmdr))
            # collect all services messages to write to robot
            if len(self.srvs) > 0:
                msgs += get_msg_arr([], map((lambda a: a.get_msgs()), self.srvs))
            v = ""
            if len(msgs) == 0:
                return
            for (probe, value) in msgs:
                value_type = type(value)
                if value_type == tuple or value_type == list:
                    # raise TypeError("message value incorrectly formatted")
                    rclpy.logerr("Probe [%s] has an incorrectly formatted value", probe)
                    continue
                if value_type != str and value_type != int and value_type != float:
                    rclpy.logerr("Probe [%s] has an incorrectly type value", probe)
                    continue
                probe_id = Client.lookup_probe_id(self, probe)
                probe_type = Client.lookup_probe_type(self, probe_id)

                v += make_tlv(
                    probe_id, Client.encode_probe_value(self, probe_type, value)
                )
            Client.send_message(self, make_tlv(MP_TLV_WRITE_PROBES, v))
        except SocketError as e:
            print(msgs)
            if e.errno == errno.ECONNRESET:
                rlcpy.logerr("CONNECTION RESET BY PEER WHILE WRITING TO SERVER")
                rlcpy.loginfo("Reconnecting to Server")
                # self.restart()
                Client.reconnect(self, self.hostname)
            else:
                raise e
        except Exception as e:
            raise e

    def do_async_message(self, t, l, v):
        # async msg handler
        if t == MP_TLV_PROBE_DATA:
            self.read_probe_msg(v) if self.recording else self.capture_data(v)
            return True
        elif t == MP_TLV_DEBUG_MESSAGE:
            rclpy.logdebug(Client.get_debug_message(self, v))
            return True
        elif t == MP_TLV_MISSED_DATA:
            print ("Missed %d probe messages" % struct.unpack("<I", v))
            return True
        elif t == MP_TLV_MISSED_DEBUG_MESSAGES:
            print ("Missed %d debug messages" % struct.unpack("<I", v))
            return True
        else:
            return False  # not an asynchronous message

    def read_probe_msg(self, data):
        # pass data to the correct child class
        try:
            while len(data):
                (t, l, v, data) = next_tlv(data)
                (probe_name, probe_type) = Client.lookup_probe(self, t)
                if any([lst.needs(probe_name) for lst in self.lstnr]):
                    try:
                        # attempt to decode the probe data
                        val = Client.decode_probe_data(self, probe_type, v)
                        # print ("New Message[name: " + str(probe_name) + " val: " + str(val) + "]")
                        # if the data is valid assign it to relavent listeners
                        [serv.add(probe_name, val) for serv in self.lstnr]
                    except RuntimeError as e:
                        # Catch the runtime error emitted when the probe data decoding fails
                        rclpy.logwarn(
                            "Error processing probe data of type %s",
                            type_table[probe_type][0],
                        )
                        err = traceback.format_exc()
                        rclpy.logwarn(err)
                        continue
                else:
                    # msg can be used for debugging. otherwise this block is probably not needed
                    rclpy.logdebug("Probe info not needed: %s" % probe_name)
                    pass

        except RuntimeWarning as e:
            # msg can be used for debugging. otherwise this block is probably not needed
            rclpy.logdebug(e)
            pass
        except Exception as e:
            err = traceback.format_exc()
            rclpy.logwarn(err)
            raise e

    def publish(self):
        # publish the data to ROS
        try:
            for lstn in self.lstnr:
                lstn.publish()
        except Exception as e:
            raise e

    def end_ros_services(self):
        # stop all ROS services
        try:
            for ros_srv in self.srvs:
                ros_srv.stop()
        except Exception as e:
            raise e
