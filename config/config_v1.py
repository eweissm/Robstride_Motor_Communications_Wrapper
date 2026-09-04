###################################
# Robstride Config
# By Eric Weissman
##################################
import numpy as np
from numpy import pi
from dataclasses import dataclass


##############################
# Robot Params
##############################
NUM_JOINTS = 1 # number io

# joint order on the low level control
# this order must be matched on the teensey
ROBOT_ORDER = ['top']

###########################
# Motor Properties
##########################

@dataclass
class MotorConfig:
    CAN_ID: int                           #CAN ID for the motors
    DEFAULT_STANCE_JOINT_ANGLES: float    #joint angles of the default stance (measured from standing straight up) (joint direction define in policy) (rads)
    INVERT_DIRECTION: bool                #Do we flip the direction of the policy output to match quadruped?

MOTORS = {
    "top": MotorConfig(CAN_ID = 0x0E, DEFAULT_STANCE_JOINT_ANGLES = 0, INVERT_DIRECTION= False)
}


#################
# Communications
#################
TEENSEY_COMMS_PORT = "/dev/ttyACM0" #teensey serial port
TEENSEY_COMMS_BAUD = 2_000_000

#################
# Rates
#################
DRIVER_RATE     = 400       # Joint Write publish rate (hz)
MOTOR_COMS_RATE = 400  # motor communication rate
PRINT_HZ = 10

#################
# Low Level Motor Control Params
#################

# Motor Motion Parameters.
#parameter order is in "robot order"
INIT_MAX_SPD = [2.5]*NUM_JOINTS # speed limit for initialization (rad/s)
MAX_SPD = [30]* NUM_JOINTS # speed limits for normal operation     rad/s
MAX_ACC = [300]*NUM_JOINTS       # acceleration limits for normal operation   rad/s^2
LIMIT_CURRENT =[11]*NUM_JOINTS
MAX_TORQUE = [5.5]*NUM_JOINTS
CONTROL_MODE = ['CSP']*NUM_JOINTS
ZERO_STA_MODE = 0x01 #DO NOT CHANGE. Makes zeroing read from [-pi,pi] rather than [0,2pi]
ZEROING = [True]*NUM_JOINTS #sets initial pose to new zero. Will remain on sucessive power ups
LOC_KP = [20]*NUM_JOINTS
VEL_KP = [1.5]*NUM_JOINTS
VEL_KI= [0.0]*NUM_JOINTS
VEL_FILTER_GAIN = [.1]*NUM_JOINTS
MOTOR_TYPES = ["RS00"]*NUM_JOINTS


#Motion Limits
JOINT_MAX_ANGLES = [pi ,
                    ]

JOINT_MIN_ANGLES =  [-pi ,
                    ]
