#simple script to test motor communications and control. 
# This script will run the motors in a sinusoidal motion.

import time
import numpy as np
from numpy import pi

from RobstrideMotorMessagesClass_V6 import MotorMessages
import config_v1 as cfg

#######################################################
#Setup Motor driver
#######################################################
num_joints = cfg.NUM_JOINTS

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
MotorController.set_zero_sta_mode(cfg.ZERO_STA_MODE)

time.sleep(1)

MotorController.zero_motors(cfg.ZEROING) #motor angles at startup is home

if np.any(cfg.ZEROING):
    print(f"WARNING: Setting current pose as new zero for motors: {[i for i, val in enumerate(cfg.ZEROING) if val]}")


MotorController.set_control_mode(cfg.CONTROL_MODE)
MotorController.set_max_speed_PP(cfg.INIT_MAX_SPD)
MotorController.set_max_CSP_speed(cfg.INIT_MAX_SPD)
MotorController.set_max_accel(cfg.MAX_ACC)
MotorController.set_max_torque(cfg.MAX_TORQUE)
MotorController.set_current_limit(cfg.LIMIT_CURRENT)
MotorController.set_loc_kp(cfg.LOC_KP)
MotorController.set_vel_kp(cfg.VEL_KP)
MotorController.set_vel_ki(cfg.VEL_KI)
MotorController.set_vel_filter_gain(cfg.VEL_FILTER_GAIN)
MotorController.reset_over_torque_timers()
MotorController.start_motors()

#setPos Ref to zero
print("Running to zero...")

#build control reference
control_ref = []
for i in range(num_joints):
    if cfg.CONTROL_MODE[i] == 'CSP':
        control_ref = control_ref + [0]
    elif cfg.CONTROL_MODE[i] == 'operation':
        control_ref = control_ref + [[0, 0, 0, cfg.LOC_KP[i], cfg.VEL_KP[i]]]
    else:
        print('control mode should be CSP or operation')

MotorController.set_control_reference(control_ref, threading=False)

MotorController.set_control_reference([0]*num_joints, threading=False) #set to zero
time.sleep(2) # give joints time to run to zero before starting


print("Setting Speed limit for normal operation.")
MotorController.set_max_speed_PP(cfg.MAX_SPD)
MotorController.set_max_CSP_speed(cfg.MAX_SPD)

print("Starting motor communication thread.")

MotorController.communicator.start() #start coms thread

start = time.monotonic_ns()/10**9

#motor setpoint is the reference for the motors. This is what the policy will update
setpoint = np.array([0.0]*num_joints)
######################################################
# Main Control Loop
######################################################

while True:
    now = time.monotonic_ns()/10**9 - start
    
    # Generate a sinusoidal setpoint
    setpoint = 0.5 * np.array([np.sin(2*np.pi*now)]*num_joints)


    # build control reference
    control_ref = []
    for i in range(num_joints):
        if cfg.CONTROL_MODE[i] == 'CSP':
            control_ref = control_ref + [setpoint[i]]
        elif cfg.CONTROL_MODE[i] == 'operation':
            control_ref = control_ref + [[setpoint[i], 0, 0, cfg.LOC_KP[i], cfg.VEL_KP[i]]]
        else:
            print('control mode should be CSP or operation')

    # send control reference to motor controller
    MotorController.set_control_reference(control_ref, threading=True)

    #read motor states
    JointAngles, jointVelocities, Motor_torques, temperatures, comsErrorFlags = MotorController.get_motor_states()

    print(f"JointAngles: {JointAngles}")

    time.sleep(.01)

  