import time


import numpy as np
from numpy import pi
from RobstrideMotorMessagesClass_V6 import MotorMessages
import os
import csv
import config_v1 as cfg

################################################3
# Driver ROS2 Node
################################################3

        
num_joints = cfg.NUM_JOINTS
#motor setpoint is the reference for the motors. This is what the policy will update
setpoint = np.array([0.0]*num_joints)

#######################################################
#Setup Motor driver
#######################################################
# Motor Motion Parameters
Init_MaxSpeed = cfg.INIT_MAX_SPD #  speed limit for initialization (rad/s)
MaxSpeed = cfg.MAX_SPD             # speed limits for normal operation     rad/s
MaxAcc = cfg.MAX_ACC      # acceleration limits for normal operation   rad/s^2
limit_Current =cfg.LIMIT_CURRENT
maxTorque = cfg.MAX_TORQUE
controlMode = cfg.CONTROL_MODE
Zero_sta_mode=cfg.ZERO_STA_MODE #DO NOT CHANGE. Makes zeroing read from [-pi,pi] rather than [0,2pi]
Zeroing=cfg.ZEROING #sets initial pose to new zero. Will remain on sucessive power ups
loc_kp = cfg.LOC_KP
vel_kp = cfg.VEL_KP
vel_ki= cfg.VEL_KI
vel_filter_gain = cfg.VEL_FILTER_GAIN 

#build CAN ids in robot order
robot_order_can_ids=[]
for name in cfg.ROBOT_ORDER:
    robot_order_can_ids.append(cfg.MOTORS[name].CAN_ID)

MotorController  = MotorMessages(coms_port=cfg.TEENSEY_COMMS_PORT,
                                    coms_freq=cfg.MOTOR_COMS_RATE,
                                    motor_ids= robot_order_can_ids,
                                    min_angles = cfg.JOINT_MIN_ANGLES,
                                    max_angles = cfg.JOINT_MAX_ANGLES,
                                    serial_baudrate=cfg.TEENSEY_COMMS_BAUD,
                                    motor_types=cfg.MOTOR_TYPES,
                                    logger=None
                                    )
        
        
        

######################################################
# Motor Initializtion
######################################################

print("Driver node ready... \n Starting motors...")

MotorController.stop_motors()
MotorController.set_zero_sta_mode(Zero_sta_mode)

time.sleep(1)

MotorController.zero_motors(Zeroing) #motor angles at startup is home

if np.any(Zeroing):
    print(f"WARNING: Setting current pose as new zero for motors: {[i for i, val in enumerate(Zeroing) if val]}")


MotorController.set_control_mode(controlMode)
MotorController.set_max_speed_PP(Init_MaxSpeed)
MotorController.set_max_CSP_speed(Init_MaxSpeed)
MotorController.set_max_accel(MaxAcc)
MotorController.set_max_torque(maxTorque)
MotorController.set_current_limit(limit_Current)
MotorController.set_loc_kp(loc_kp)
MotorController.set_vel_kp(vel_kp)
MotorController.set_vel_ki(vel_ki)
MotorController.set_vel_filter_gain(vel_filter_gain)
MotorController.reset_over_torque_timers()
MotorController.start_motors()

#setPos Ref to zero
print("Running to zero...")

#build control reference
control_ref = []
for i in range(num_joints):
    if controlMode[i] == 'CSP':
        control_ref = control_ref + [0]
    elif controlMode[i] == 'operation':
        control_ref = control_ref + [[0, 0, 0, loc_kp[i], vel_kp[i]]]
    else:
        print('control mode should be CSP or operation')

MotorController.set_control_reference(control_ref, threading=False)

MotorController.set_control_reference([0]*num_joints, threading=False) #set to zero
time.sleep(2) # give joints time to run to zero before starting


print("Setting Speed limit for normal operation.")
MotorController.set_max_speed_PP(MaxSpeed)
MotorController.set_max_CSP_speed(MaxSpeed)

print("Starting motor communication thread.")

MotorController.communicator.start() #start coms thread

start = time.monotonic_ns()/10**9

while True:
    now = time.monotonic_ns()/10**9 - start
    
    # Generate a sinusoidal setpoint
    setpoint = 0.5 * np.array([np.sin(2*np.pi*now)]*num_joints)


    # Send motor commands
    control_ref = []
    for i in range(num_joints):
        if controlMode[i] == 'CSP':
            control_ref = control_ref + [setpoint[i]]
        elif controlMode[i] == 'operation':
            control_ref = control_ref + [[setpoint[i], 0, 0, loc_kp[i], vel_kp[i]]]
        else:
            print('control mode should be CSP or operation')

    MotorController.set_control_reference(control_ref, threading=True)

    JointAngles, jointVelocities, Motor_torques, temperatures, comsErrorFlags = MotorController.get_motor_states()

    print(f"JointAngles: {JointAngles}")

    time.sleep(.01)

  