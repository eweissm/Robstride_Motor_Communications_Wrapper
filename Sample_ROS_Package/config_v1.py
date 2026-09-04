###################################
# Squad Config
# By Eric Weissman
#
#
#
##################################
import numpy as np
from numpy import pi
from dataclasses import dataclass

ENCODER_TOPIC = 'encoder_state'

##############################
# Robot Params
##############################
NUM_JOINTS = 2 # number io

# joint order on the low level control
# this order must be matched on the teensey
# DONT CHANGE !!!! THE motor parameters class below doesnt contain all params used in the driver. order matters
ROBOT_ORDER = ['top', 'bottom']

###########################
# Motor Properties
##########################

@dataclass
class MotorConfig:
    CAN_ID: int                           #CAN ID for the motors
    DEFAULT_STANCE_JOINT_ANGLES: float    #joint angles of the default stance (measured from standing straight up) (joint direction define in policy) (rads)
    INVERT_DIRECTION: bool                #Do we flip the direction of the policy output to match quadruped?

MOTORS = {
    "top": MotorConfig(CAN_ID = 0x01, DEFAULT_STANCE_JOINT_ANGLES = 0, INVERT_DIRECTION= False),

    "bottom": MotorConfig(CAN_ID = 0x02, DEFAULT_STANCE_JOINT_ANGLES = -.1, INVERT_DIRECTION= False),
}


#################
# Communications
#################
TEENSEY_COMMS_PORT = "/dev/ttyACM1"
TEENSEY_COMMS_BAUD = 2_000_000

ENCODER_SERIAL_PORT = "/dev/ttyACM0"
ENCODER_SERIAL_BAUDRATE = 9600
#################
# Rates
#################
DRIVER_RATE     = 400       # Joint Write publish rate (hz)
MOTOR_COMS_RATE = 400  # motor communication rate
PRINT_HZ = 10

#################
#data Logging
#################
LOG_MOTOR_STATE = False   #bool if we should record motor state to csv
MOTOR_STATE_CSV_PATH = 'src/cocontraction_analysis_platform/cocontraction_analysis_platform/exp_data/motor_state_log_v1.csv' #If we LOG_MOTOR_STATE, path to csv

LOG_ENCODER_STATE = False   #bool if we should record motor state to csv
ENCODER_STATE_CSV_PATH = 'src/cocontraction_analysis_platform/cocontraction_analysis_platform/exp_data/encoder_state_log_v1.csv' #If we LOG_MOTOR_STATE, path to csv


LOG_ERROR = True   #bool if we should record motor state to csv
ERROR_CSV_PATH = 'src/cocontraction_analysis_platform/cocontraction_analysis_platform/exp_data/co_contraction_modelless_yarn_pretension_sweep_v1.csv' #If we LOG_MOTOR_STATE, path to csv

#################
# Low Level Motor Control Params
#################

# Motor Motion Parameters.
#parameter order is in "robot order"
INIT_MAX_SPD = [2.5]*NUM_JOINTS # speed limit for initialization (rad/s)
MAX_SPD = [50]* NUM_JOINTS # speed limits for normal operation     rad/s
MAX_ACC = [300]*NUM_JOINTS       # acceleration limits for normal operation   rad/s^2
LIMIT_CURRENT =[1.5]*NUM_JOINTS
MAX_TORQUE = [5.5]*NUM_JOINTS
CONTROL_MODE = ['operation']*NUM_JOINTS
ZERO_STA_MODE = 0x01 #DO NOT CHANGE. Makes zeroing read from [-pi,pi] rather than [0,2pi]
ZEROING = [True]*NUM_JOINTS #sets initial pose to new zero. Will remain on sucessive power ups
LOC_KP = [20]*NUM_JOINTS
VEL_KP = [1.5]*NUM_JOINTS
VEL_KI= [0.0]*NUM_JOINTS
VEL_FILTER_GAIN = [.1]*NUM_JOINTS
MOTOR_TYPES = ["RS05"]*NUM_JOINTS


#Motion Limits
JOINT_MAX_ANGLES = [pi ,
                    pi ,
                    ]

JOINT_MIN_ANGLES =  [-pi ,
                     -pi ,
                    ]


def toRads(deg):
            return deg*np.pi/180

co_contraction = 5
PRETENSION_ANGLES = [toRads(co_contraction), -toRads(co_contraction)] #joint angles of the pretensioning pose (measured from standing straight up) (joint direction define in policy) (rads)

JOINT_ANGLE_KP = .0
JOINT_ANGLE_KD = 0.0

KINEMATIC_TABLE = "src/cocontraction_analysis_platform/cocontraction_analysis_platform/exp_data/co_contraction_kinematic_table_v4.csv"
