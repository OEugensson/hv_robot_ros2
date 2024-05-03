#!/usr/bin/env python
import rclpy
import argparse
import traceback
import errno
import sys, os, time
from socket import error as SocketError

# add local python scripts to the path so that they can be iported
sys.path.append(os.path.abspath(os.path.join(sys.path[0], "../")))
sys.path.append(os.path.abspath(os.path.join(sys.path[0], "../srv")))

from scripts.hv_client import *
from libs import *

# Global variables
host_name = "hai-1095.local"  # this is the hostname the robot being controlled.



if __name__ == "__main__":
    main()
 
def main():
    ###
    # Services Main Node
    # rosrun services main_node [HOSTNAME (optional)]
    # This node must be run to allow the connection between ROS and the HarvestAI bots over
    # the Mindprobe protocol
    ###
    
    hv_bot = None
    try:
        # Init the connection with the ROS system
        #rospy.init_node("hai_main_node", anonymous=True)
        rclpy.init(args=sys.argv)
        node = rclpy.create_node("hai_main_node")
        
        # Init the argument parser
        parser = argparse.ArgumentParser(
            description="ROS Connection Application for comunication and control of the HV Robots"
        )
        
        # Add the arguments to handle setting various sensor and controller connections
        parser.add_argument(
            "-n",
            "--name",
            "--hostname",
            metavar="hostname",
            dest="hostname",
            type=str,
            default="hai-1095.local",
            nargs="?",
            help="The name of the robot, generally of the form hai-####.local (default: hai-1095.local",
        )
        parser.add_argument(
            "--all",
            action="store_true",
            dest="all",
            help="Use all the sensors and controllers",
        )
        parser.add_argument(
            "--all-controllers",
            action="store_true",
            dest="controller_flag",
            help="Enable all controller classes",
        )
        parser.add_argument(
            "-c",
            "--controllers",
            type=int,
            nargs="+",
            dest="controllers",
            choices=[1, 2, 3],
            help="Set the options for which contollers to use (choices: 1 - arm, 2 - gripper, 3 - drive)",
        )
        parser.add_argument(
            "--all-sensors",
            action="store_true",
            dest="sensor_flag",
            help="Enable all sensor classes",
        )
        parser.add_argument(
            "-s",
            "--sensors",
            type=int,
            nargs="+",
            dest="sensors",
            choices=[1, 2, 3, 4, 5, 6],
            help="Set the options for which sensors to use (choices: 1 - lidar, 2 - gripper, 3 - ir, 4 - odometry, 5 - odometry w/ gyro, 6 - nearest object)",
        )
        parser.add_argument(
            "--services",
            type=str,
            nargs="*",
            dest="services",
            choices=[None, "all", "follow", "pick", "avoid"],
            default="all",
            help="Set the services to make available",
        )
        parser.add_argument(
            "sys",
            type=str,
            nargs="*",
            help="System included variables from launch file (not set bu user)",
        )
        
        args = parser.parse_args()

        # set the hostname based on the argument inputed by the user (default: hai-1095.local)
        host_name = args.hostname
        
        services = []
        if args.all:
            # connect all sensors and controllers
            services = [
                DRIVE_CMD,
                GRIPPER_CMD,
                ARM_CMD,
                ODOM_LSTN,
                IR_LSTN,
                LIDAR_LSTN,
                GRIPPER_LSTN,
                NEAREST_OBS,
                ODOM_GYRO_LSTN,
            ]
        else:
            if args.sensor_flag:
                # add all sensors
                services += [
                    ODOM_LSTN,
                    IR_LSTN,
                    LIDAR_LSTN,
                    GRIPPER_LSTN,
                    NEAREST_OBS,
                    ODOM_GYRO_LSTN,
                ]
            elif args.sensors != None:
                # add selected sensors
                if 1 in args.sensors:
                    services.append(LIDAR_LSTN)
                if 2 in args.sensors:
                    services.append(GRIPPER_LSTN)
                if 3 in args.sensors:
                    services.append(IR_LSTN)
                if 4 in args.sensors:
                    services.append(ODOM_LSTN)
                if 5 in args.sensors:
                    services.append(ODOM_GYRO_LSTN)
                if 6 in args.sensors:
                    services.append(NEAREST_OBS)
            if args.controller_flag:
                # add all controllers
                services += [DRIVE_CMD, GRIPPER_CMD, ARM_CMD]
            elif args.controllers != None:
                # add only the selected controllers
                if 3 in args.sensors:
                    services.append(DRIVE_CMD)
                if 2 in args.controllers:
                    services.append(GRIPPER_CMD)
                if 1 in args.controllers:
                    services.append(ARM_CMD)
        if "all" in args.services:
            # add all services
            services += [FOLLOW_SRV, PICK_SRV, AVOID_SRV]
        elif args.services != []:
            if "follow" in args.services:
                services.append(FOLLOW_SRV)
            if "pick" in args.services:
                services.append(PICK_SRV)
            if "avoid" in args.services:
                services.append(AVOID_SRV)
        
        node.get_logger().info("Connecting")
        
        
        hv_bot = HVClient(host_name, services)
        #Vi kommer ej hit
        
        # handle closing everything nicely when ros shutsdown
        rclpy.on_shutdown(hv_bot.stop)

        try:
            # start the connection to the robot
            hv_bot.start()
            time.sleep(1)

            # set rate based on the mindprobe refresh rate.
            # default to 100. (mp runs at 200)
            hz = 100
            if hv_bot.hz:
                hz = hv_bot.hz / 2

            # Start the ROS main loop
            rate = rclpy.Rate(hz)

            node.get_logger().info(
                "starting publishing data from " + host_name + " at " + str(hz) + "hz"
            )
            hv_bot.start_probes()
            hv_bot.recording = True
            while not rclpy.is_shutdown():
                # write any messages from ROS to the robot
                # hv_bot.capture(1 / 200)
                hv_bot.write()
                hv_bot.publish()

                rate.sleep()
        except OSError as e:
            #rclpy.logerr("Connection failed!")
            err = traceback.format_exc()
            #rclpy.logerr(e)
            node.get_logger().info(err)
            if e.errno == errno.ECONNRESET:
                #rclpy.logerr("CONNECTION RESET BY PEER")
                if hv_bot != None and not hv_bot.stopped:
                    node.get_logger().info("Reconnecting to Server")
                    hv_bot.reconnect(host_name)
        except SocketError as e:
            # this means that the connection has failed at some point
            # if the socket error has not been handled at a previous point, assume the connection is broken
            err = traceback.format_exc()
            #rclpy.logerr(e)
            node.get_logger().info(err)
            hv_bot.reconnect(hostname)
        except RuntimeError as e:
            err = traceback.format_exc()
            rclpy.logging.get_logger(e)
            #print(e)
            node.get_logger().info(err)
        except RuntimeWarning as e:
            rclpy.logging.get_logger(e)
            print(e)
            pass
        except:
            # if not a specifically handled error, raise end service
            raise
    #except rclpy.ROSInterruptException:
    #    rclpy.logerr("ROS interupt called -- Ending Processes")
    except Exception as e:
        # print the error type - to be used to add better error handling in
        rclpy.logging.get_logger("the type should be: %s" % type(e))
        # general error handling
        err = traceback.format_exc()
        rclpy.logging.get_logger(err)
        #print(err)
        #rclpy.logfatal(e)
        pass
    finally:
        # last code to run before everything closes
        if hv_bot != None and not hv_bot.stopped:
            node.get_logger().info("Stopping Client")
            hv_bot.stop()
        rclpy.logging.get_logger("GOODBYE")
