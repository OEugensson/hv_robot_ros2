from enum import Enum

TOTAL_PROBES = 3900

###
# Type names for probe types (t in tlv)
###
MP_TLV_NULL = 0
MP_TLV_CONNECTED = 1
MP_TLV_DISCONNECTED = 2
MP_TLV_PROTOCOL_VERSION = 3
MP_TLV_DISCOVER_PROBES = 4
MP_TLV_HZ = 5
MP_TLV_PROBE_DEF = 6
MP_TLV_ENABLE_PROBES = 7
MP_TLV_START_PROBES = 8
MP_TLV_STOP_PROBES = 9
MP_TLV_PROBE_DATA = 10
MP_TLV_CURRENT_TICK = 11
MP_TLV_MISSED_DATA = 12
MP_TLV_DEBUG_MESSAGE = 13
MP_TLV_MESSAGE_TEXT = 14
MP_TLV_MISSED_DEBUG_MESSAGES = 15
MP_TLV_WRITE_PROBES = 16

type_table = {
    1: ("uint8", "<B"),  # 1-byte unsigned integer
    2: ("int8", "<b"),  # 1-byte signed integer
    3: ("uint16", "<H"),  # 2-byte unsigned integer
    4: ("int16", "<h"),  # 2-byte signed integer
    5: ("uint32", "<I"),  # 4-byte unsigned integer
    6: ("int32", "<i"),  # 4-byte signed integer
    7: ("uint64", "<Q"),  # 8-byte unsigned integer
    8: ("int64", "<q"),  # 8-byte signed integer
    9: ("float", "<f"),  # 4-byte float (IEEE 754)
    10: ("double", "<d"),  # 8-byte float (IEEE 754)
    11: ("tlv", ""),  # Variable-length TLV data
    12: ("bool", "<B"),  # 1-byte boolean integer (1 or 0)
    13: ("string", ""),  # variable-size null-terminated char string
}


# Declare function types
HV_Services = Enum(
    "HV_Services",
    "DRIVE_CMD GRIPPER_CMD ARM_CMD ODOM_CMD ODOM_LSTN IR_LSTN LIDAR_LSTN GRIPPER_LSTN NEAREST_OBS ODOM_GYRO_LSTN FOLLOW_SRV PICK_SRV AVOID_SRV",
)

# Function toggles

DRIVE_CMD = HV_Services.DRIVE_CMD
GRIPPER_CMD = HV_Services.GRIPPER_CMD
ARM_CMD = HV_Services.ARM_CMD
ODOM_CMD = HV_Services.ODOM_CMD
ODOM_LSTN = HV_Services.ODOM_LSTN
IR_LSTN = HV_Services.IR_LSTN
LIDAR_LSTN = HV_Services.LIDAR_LSTN
GRIPPER_LSTN = HV_Services.GRIPPER_LSTN
NEAREST_OBS = HV_Services.NEAREST_OBS
ODOM_GYRO_LSTN = HV_Services.ODOM_GYRO_LSTN
FOLLOW_SRV = HV_Services.FOLLOW_SRV
PICK_SRV = HV_Services.PICK_SRV
AVOID_SRV = HV_Services.AVOID_SRV


# Probe names

START = "ui.fakeStartButtonNoSpacing"  # 1125
LIDAR_STANDBY = "vis.sick.useStandByMode"  # 2877
ARM_ENABLED = "manip.armEnableRequest"  # 1082
GRIPPER_ENABLED = "manip.gripperEnableRequest"  # 1083
DRIVE_ENABLED = "drive.enableRequest"  # 580
DRIVE_COMMAND = "mob.driveCommand.enable"  # 1220
ROB_SENSOR = "vis.robotSensor.enabled"  # 3100
LINE_SENSOR = "vis.lineSensor.enabled"  # 3171
ESTOP = "ui.fakeEstop"
FLAG_PULL = "estop.fakeFlagPulled"

BEEP = "ui.enableBeeping"  # 1120

HAS_POT = "manip.botGotPot"

IR_RIGHT_FRONT_A = "bound.rightFront.signalA"
IR_LEFT_FRONT_A = "bound.leftFront.signalA"
IR_RIGHT_BACK_A = "bound.rightRear.signalA"
IR_LEFT_BACK_A = "bound.leftRear.signalA"
IR_RIGHT_FRONT_B = "bound.rightFront.signalB"
IR_LEFT_FRONT_B = "bound.leftFront.signalB"
IR_RIGHT_BACK_B = "bound.rightRear.signalB"
IR_LEFT_BACK_B = "bound.leftRear.signalB"

LOCAL_SCAN = "vis.sick.localScanData"
GLOBAL_SCAN = "vis.sick.globalScanData"
OBSTACLE_X = "vis.forwardObstacle.x"
OBSTACLE_Y = "vis.forwardObstacle.y"


ODOM_X = "loc.rawOdometry.x"
ODOM_Y = "loc.rawOdometry.y"
ODOM_H = "loc.rawOdometry.h"
ODOM_V = "loc.rawOdometry.v"
ODOM_W = "loc.rawOdometry.w"

ODOM_GYRO_X = "loc.rawOdometryWGyro.x"
ODOM_GYRO_Y = "loc.rawOdometryWGyro.y"
ODOM_GYRO_H = "loc.rawOdometryWGyro.h"
ODOM_GYRO_V = "loc.rawOdometryWGyro.v"
ODOM_GYRO_W = "loc.rawOdometryWGyro.w"

FOLLOW_ME = "behaviors.followMe.run"
BOT_GOT_POT = "manip.botGotPot"
SET_PICK_X = "behaviors.spacing.pick.pickTarget.x"
SET_PICK_Y = "behaviors.spacing.pick.pickTarget.y"
RUN_SPACING = "behaviors.spacing.runCollection"
AVOID = "behaviors.scriptAvoidCollision.isEnabled"
