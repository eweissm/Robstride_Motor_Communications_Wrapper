import rclpy
from rclpy.node import Node
import time


import numpy as np
from numpy import pi
from cocontraction_analysis_platform.RobstrideMotorMessagesClass_V4 import MotorMessages
import os
import csv
import cocontraction_analysis_platform.config_v1 as cfg
from std_msgs.msg import Float32
from scipy.signal import butter, lfilter, lfilter_zi
import pandas as pd
from collections import deque

################################################3
# Driver ROS2 Node
################################################3
class Driver(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        self.num_joints = cfg.NUM_JOINTS

        self.active = False #bool storing if the motors are active. 
        ##########################################################
        #Setup ROS coms stuff
        #########################################################
        
        self.logDataToCSV = cfg.LOG_MOTOR_STATE
        self.log_data_csv_path = os.path.expanduser(cfg.MOTOR_STATE_CSV_PATH)

        self.encoder_subcriber = self.create_subscription(Float32, cfg.ENCODER_TOPIC, self._on_encoder, 10)

        #setup timers to call functions
        self.create_timer(1.0/cfg.DRIVER_RATE, self._motor_Control_Thread)
        self.create_timer(1.0/cfg.DRIVER_RATE, self._state_publish_Thread)
        self.create_timer(1.0/cfg.PRINT_HZ,    self._print)


        ############################################################
        # controls stuff
        ############################################################

        #motor setpoint is the reference for the motors. This is what the policy will update
        self.setpoint = np.array([0.0]*self.num_joints)

        self.joint_angles_offset = None

        self.actual_joint_angles = 0.0
        self.actual_joint_vel = 0.0
        self.actual_joint_vel_filt = 0.0

        self.prev_joint_angle = None
        self.prev_joint_time = None

        # Filter settings
        self.vel_cutoff_hz = 5.0      # lower = smoother, more lag
        self.vel_filter_order = 2

        # If your encoder callback is roughly fixed-rate, set an expected sample rate
        self.vel_fs_hz = cfg.DRIVER_RATE       # adjust to your callback rate

        self.vel_b, self.vel_a = butter(
            self.vel_filter_order,
            self.vel_cutoff_hz / (0.5 * self.vel_fs_hz),
            btype='low'
        )

        self.vel_filter_initialized = False
        self.vel_zi = None

        self.joint_angles = np.zeros(self.num_joints)
        self.joint_velocities = np.zeros(self.num_joints)
        self.motor_torques = np.zeros(self.num_joints)


        table = pd.read_csv(cfg.KINEMATIC_TABLE)

        self.joint_angles_kinematics_map = table["joint_angle"].to_numpy()
        self.motor_top_kinematics_map = table["motor_angle_top"].to_numpy()
        self.motor_bottom_kinematics_map = table["motor_angle_bottom"].to_numpy()        
        


        WINDOW = 400

        # One deque per motor
        self.motor_error_history = [
            deque(maxlen=WINDOW),  # Motor 1
            deque(maxlen=WINDOW)   # Motor 2
]

        #######################################################
        #Setup Motor driver
        #######################################################
        # Motor Motion Parameters
        self.Init_MaxSpeed = cfg.INIT_MAX_SPD #  speed limit for initialization (rad/s)
        self.MaxSpeed = cfg.MAX_SPD             # speed limits for normal operation     rad/s
        self.MaxAcc = cfg.MAX_ACC      # acceleration limits for normal operation   rad/s^2
        self.limit_Current =cfg.LIMIT_CURRENT
        self.maxTorque = cfg.MAX_TORQUE
        self.controlMode = cfg.CONTROL_MODE
        self.Zero_sta_mode=cfg.ZERO_STA_MODE #DO NOT CHANGE. Makes zeroing read from [-pi,pi] rather than [0,2pi]
        self.Zeroing=cfg.ZEROING #sets initial pose to new zero. Will remain on sucessive power ups
        self.loc_kp = cfg.LOC_KP
        self.vel_kp = cfg.VEL_KP
        self.vel_ki= cfg.VEL_KI
        self.vel_filter_gain = cfg.VEL_FILTER_GAIN 

        #build CAN ids in robot order
        robot_order_can_ids=[]
        for name in cfg.ROBOT_ORDER:
            robot_order_can_ids.append(cfg.MOTORS[name].CAN_ID)

        self.MotorController  = MotorMessages(coms_port=cfg.TEENSEY_COMMS_PORT,
                                            coms_freq=cfg.MOTOR_COMS_RATE,
                                            motor_ids= robot_order_can_ids,
                                            min_angles = cfg.JOINT_MIN_ANGLES,
                                            max_angles = cfg.JOINT_MAX_ANGLES,
                                            serial_baudrate=cfg.TEENSEY_COMMS_BAUD,
                                            logger=self.get_logger(),
                                            motor_types=cfg.MOTOR_TYPES
                                            )
        
        
        #####################################################
        # --- CSV logging for ActNet ---
        ######################################################        
        if self.logDataToCSV:
            self.get_logger().info(f"Logging CSV to {self.log_data_csv_path}")


            self._motor_csv_file = open(self.log_data_csv_path, 'w', newline='')
            self._motor_csv_writer = csv.writer(self._motor_csv_file)

            # Build header
            self._motor_csv_writer.writerow(
            ["t"]+ 
            [f"SetPoint_{cfg.ROBOT_ORDER[i]}" for i in range(self.num_joints)] + 
            [f"actual_angle_{cfg.ROBOT_ORDER[i]}" for i in range(self.num_joints)]+ 
            [f"actual_vel_{cfg.ROBOT_ORDER[i]}" for i in range(self.num_joints)] +
            [f"Motor_Torque_{cfg.ROBOT_ORDER[i]}" for i in range(self.num_joints)])
            self._motor_csv_file.flush()

        if cfg.LOG_ERROR:
            self.get_logger().info(f"Logging CSV to {cfg.ERROR_CSV_PATH}")

            self._error_csv_file = open(cfg.ERROR_CSV_PATH, 'w', newline='')
            self._error_csv_writer = csv.writer(self._error_csv_file)

            # Build header
            self._error_csv_writer.writerow(["t"]+
                                            [f"setpoint"]+
                                            [f"actual_angle"]+
                                            ["Error"]+ 
                                            [f"pretension"]+
                                            [f"actual_angle_{cfg.ROBOT_ORDER[i]}" for i in range(self.num_joints)]+ 
                                            [f"actual_vel_{cfg.ROBOT_ORDER[i]}" for i in range(self.num_joints)] +
                                            [f"motor_torque_{cfg.ROBOT_ORDER[i]}" for i in range(self.num_joints)]+
                                            [f"motor_kp_{cfg.ROBOT_ORDER[i]}" for i in range(self.num_joints)]
                                            )
            self._error_csv_file.flush()
        ######################################################
        # Motor Initializtion
        ######################################################

        self.get_logger().info("Driver node ready... \n Starting motors...")
        
        self.MotorController.stop_motors()
        self.MotorController.set_zero_sta_mode(self.Zero_sta_mode)
        
        time.sleep(1)
        
        self.MotorController.zero_motors(self.Zeroing) #motor angles at startup is home

        if np.any(self.Zeroing):
            self.get_logger().warning(f"WARNING: Setting current pose as new zero for motors: {[i for i, val in enumerate(self.Zeroing) if val]}")
        

        self.MotorController.set_control_mode(self.controlMode)
        self.MotorController.set_max_speed_PP(self.Init_MaxSpeed)
        self.MotorController.set_max_CSP_speed(self.Init_MaxSpeed)
        self.MotorController.set_max_accel(self.MaxAcc)
        self.MotorController.set_max_torque(self.maxTorque)
        self.MotorController.set_current_limit(self.limit_Current)
        self.MotorController.set_loc_kp(self.loc_kp)
        self.MotorController.set_vel_kp(self.vel_kp)
        self.MotorController.set_vel_ki(self.vel_ki)
        self.MotorController.set_vel_filter_gain(self.vel_filter_gain)
        self.MotorController.reset_over_torque_timers()
        self.MotorController.start_motors()

        #setPos Ref to zero
        self.get_logger().info("Running to zero...")
        
        #build control reference
        control_ref = []
        for i in range(self.num_joints):
            if self.controlMode[i] == 'CSP':
                control_ref = control_ref + [0]
            elif self.controlMode[i] == 'operation':
                control_ref = control_ref + [[0, 0, 0, self.loc_kp[i], self.vel_kp[i]]]
            else:
                self.get_logger().error('control mode should be CSP or operation')
        
        self.MotorController.set_control_reference(control_ref, threading=False)

        # self.MotorController.Set_PP_Mode_PosRef_single([0]*self.num_joints)
        time.sleep(2) # give joints time to run to zero before starting

        self.get_logger().info("Pretensioning cables...")

        #build control reference
        control_ref = []
        for i in range(self.num_joints):
            if self.controlMode[i] == 'CSP':
                control_ref = control_ref + [cfg.PRETENSION_ANGLES[i]]
            elif self.controlMode[i] == 'operation':
                control_ref = control_ref + [[cfg.PRETENSION_ANGLES[i], 0, 0, self.loc_kp[i], self.vel_kp[i]]]
            else:
                self.get_logger().error('control mode should be CSP or operation')
        
        self.MotorController.set_control_reference(control_ref, threading=False)

        time.sleep(2) # give joints time to run to zero before starting
        
        self.get_logger().info("Setting Speed limit for normal operation.")
        self.MotorController.set_max_speed_PP(self.MaxSpeed)
        self.MotorController.set_max_CSP_speed(self.MaxSpeed)

        self.get_logger().info("Starting motor communication thread.")

        self.MotorController.communicator.start() #start coms thread

        self.start = time.monotonic_ns()/10**9

        self.active = True

    def _on_encoder(self, msg):
        angle = msg.data

        now = time.monotonic_ns()/10**9 - self.start

        if self.joint_angles_offset is None and self.active:
            self.joint_angles_offset = angle
            self.get_logger().info(f"Setting joint angle offset to {self.joint_angles_offset} rad")
            self.prev_joint_angle = None
            self.prev_joint_time = None
            self.vel_filter_initialized = False
            self.vel_zi = None
            self.actual_joint_angles = 0.0
            self.actual_joint_vel = 0.0
            self.actual_joint_vel_filt = 0.0
            return

        if self.joint_angles_offset is None:
            return

        self.actual_joint_angles = -(angle - self.joint_angles_offset)

        if self.prev_joint_angle is not None and self.prev_joint_time is not None:
            dt = now - self.prev_joint_time
            if dt > 1e-6:
                raw_vel = (self.actual_joint_angles - self.prev_joint_angle) / dt
                self.actual_joint_vel = raw_vel

                # Initialize filter state on first valid sample
                if not self.vel_filter_initialized:
                    self.vel_zi = lfilter_zi(self.vel_b, self.vel_a) * raw_vel
                    self.vel_filter_initialized = True

                self.actual_joint_vel_filt, self.vel_zi = lfilter(
                    self.vel_b,
                    self.vel_a,
                    [raw_vel],
                    zi=self.vel_zi
                )
                self.actual_joint_vel_filt = float(self.actual_joint_vel_filt[0])

        self.prev_joint_angle = self.actual_joint_angles
        self.prev_joint_time = now

    def _motor_Control_Thread(self):
        now = time.monotonic_ns()/10**9 - self.start
        pretention = np.array(cfg.PRETENSION_ANGLES)

        cycle_time = 13.0
        t_cycle = now % cycle_time

        #joint angle set point
        if t_cycle < 1.0:
            angle_sp = 0.0
        elif t_cycle>1.0 and t_cycle < 11.0:
            angle_sp = (np.pi/6)*np.sin(2*np.pi*2*(t_cycle-1))
        else:
            angle_sp = 0.0

        cycle_count = now//cycle_time

        pretention_vals = [cfg.toRads(0), cfg.toRads(5) , cfg.toRads(10),  cfg.toRads(15),  cfg.toRads(20)]
        
        pretention = np.array([pretention_vals[int(cycle_count%len(pretention_vals))], -pretention_vals[int(cycle_count%len(pretention_vals))]])

        if cycle_count > len(pretention_vals)-1:
            self.get_logger().info("Pretensioning complete.")


        # angle_sp = (np.pi/6)*np.sin(2*np.pi*.5*now)

        #joint angle errors
        joint_angle_error = angle_sp - self.actual_joint_angles

        #motor error
        motor_angle_error = self.setpoint - self.joint_angles
        
        #use moving average to smooth error
        avg_motor_angle_error = self.moving_average_motor_error(motor_angle_error)


        u = cfg.JOINT_ANGLE_KP*joint_angle_error - cfg.JOINT_ANGLE_KD*self.actual_joint_vel_filt 

        #calculate the feed forward cable angles from the joint angle setpoint
        # cable_angles = self.calc_angles(angle_sp+u)
        cable_angles = self.joint_to_motor(angle_sp)
        self.setpoint = np.array([cable_angles[0], -cable_angles[1]])+  pretention#add pretensioning angles to setpoint

        # hyper_elasticity_slope = 200.0
        # hyper_elastic_kp = np.clip(5.+ hyper_elasticity_slope*np.abs(avg_motor_angle_error), 5., 30.)
        # self.loc_kp = hyper_elastic_kp


        # Send motor commandsmoving_average
        control_ref = []
        for i in range(self.num_joints):
            if self.controlMode[i] == 'CSP':
                control_ref = control_ref + [self.setpoint[i]]
            elif self.controlMode[i] == 'operation':
                control_ref = control_ref + [[self.setpoint[i], 0, 0, self.loc_kp[i], self.vel_kp[i]]]
            else:
                self.get_logger().error('control mode should be CSP or operation')
        
        self.MotorController.set_control_reference(control_ref, threading=True)


        #####################################################
        # --- CSV logging ---
        ######################################################
        if cfg.LOG_ERROR:
            row = [time.time()] +\
                  [angle_sp]+ \
                  [self.actual_joint_angles]  + \
                  [joint_angle_error]+ \
                  [pretention] +\
                  self.to_1d_list(self.joint_angles) \
                  + self.to_1d_list(self.joint_velocities) \
                  + self.to_1d_list(self.motor_torques) \
                  + self.to_1d_list(self.loc_kp) 
            self._error_csv_writer.writerow(row)
            self._error_csv_file.flush()

                   
    def _state_publish_Thread(self):
        
        joint_angles, joint_velocities, motor_torques, temperatures, comsErrorFlags = self.MotorController.get_motor_states()

        self.joint_angles = joint_angles
        self.joint_velocities = joint_velocities
        self.motor_torques = motor_torques

        #####################################################
        # --- CSV logging for ActNet ---
        ######################################################
        if self.logDataToCSV:
            

            row = [time.time()] \
                + self.to_1d_list(self.setpoint) \
                + self.to_1d_list(joint_angles) \
                + self.to_1d_list(joint_velocities) \
                + self.to_1d_list(motor_torques) 
                        
            self._motor_csv_writer.writerow(row)

            self._motor_csv_file.flush()
        
    
    def _print(self):
        pass
    
    # ensure terminal restored even on Ctrl+C
    def destroy_node(self):
        super().destroy_node()

    def to_1d_list(self, x):
        if isinstance(x, np.ndarray):
            return x.ravel().tolist()
        if isinstance(x, (list, tuple)):
            # flatten nested 1D lists if necessary
            return list(x)
        # scalar -> single-element list
        return [x]

    def calc_angles(self, theta):
        #all units in mm
        PULLEY_RADIUS =.0335/2 #radius of pulley in meters 
        D1 =.016
        D2 = .047
        D3 = .027
        D4 = .03

        ##################################################
        # calc L0 (the cable length at zero angle)
        ##################################################
        delta = np.pi/2 - np.arctan2(D1, D2) 

        h1 = np.sqrt(D1**2 + D2**2)    
        h3 = D3 - h1*np.cos(delta)
        h4 = D4 + h1*np.sin(delta)

        L0 = np.sqrt(h3**2 + h4**2)

        ##################################################
        # calc L_l (the cable length for the left cable)
        ##################################################
        delta = np.pi/2 - np.arctan2(D1, D2) -theta

        h1 = np.sqrt(D1**2 + D2**2)    
        h3 = D3 - h1*np.cos(delta)
        h4 = D4 + h1*np.sin(delta)

        L_l = np.sqrt(h3**2 + h4**2)

        ##################################################
        # calc L_r (the cable length for the right cable)
        ##################################################
        delta = np.pi/2 - np.arctan2(D1, D2)  + theta

        h1 = np.sqrt(D1**2 + D2**2)    
        h3 = D3 - h1*np.cos(delta)
        h4 = D4 + h1*np.sin(delta)

        L_r = np.sqrt(h3**2 + h4**2)

        ##################################################
        # calc pulley angles 
        ##################################################
        theta_l = (L0 - L_l)/PULLEY_RADIUS
        theta_r = (L0 - L_r)/PULLEY_RADIUS

        return [theta_l, theta_r]
    
    def joint_to_motor(self, desired_joint_angle):
        """
        Parameters
        ----------
        desired_joint_angle : float or ndarray
            Desired joint angle [rad]

        Returns
        -------
        ndarray
            [motor_top, motor_bottom] for a scalar input, or
            an (N,2) array for an array input.
        """
        q = np.asarray(desired_joint_angle)

        top = np.interp(q, self.joint_angles_kinematics_map, self.motor_top_kinematics_map)
        bottom = -np.interp(q, self.joint_angles_kinematics_map, self.motor_bottom_kinematics_map)
        
        if q.ndim == 0:
            return np.array([top, bottom])

        return np.column_stack((top, bottom))

    def moving_average_motor_error(self, motor_error):
        """
        motor_error: list like [motor1_error, motor2_error]
        """
        averages = []

        for i, torque in enumerate(motor_error):
            self.motor_error_history[i].append(torque)
            averages.append(sum(self.motor_error_history[i]) / len(self.motor_error_history[i]))

        return np.array(averages)

def main():
    rclpy.init()
    node = Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception:
            pass
        print('\nBye')


if __name__ == '__main__':
    main()
