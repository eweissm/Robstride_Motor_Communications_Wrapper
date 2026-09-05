'''
By: Eric weissman
9/4/2026

Helper class which builds CAN messages for robstride motors and handles response 
'''

import struct
from dataclasses import dataclass
import numpy as np
from numpy import pi
import itertools
import logging


from MotorSerialCommunicator import SerialCommunicator
    
import struct
import time

# build a data class for the rs motors. 
# will make it easy to add additional motors or parameters
@dataclass
class robstride_motor_parameters:
    max_speed: float                          
    max_torque: float
    max_continuous_torque:float
    max_kp: float
    max_kd: float
    max_current: float

#build data class of rs motors
ROBSTRIDE_MOTOR_PARAMS = {
    "RS00": robstride_motor_parameters(max_speed = 33.0,                          
                                        max_torque = 14.0,
                                        max_continuous_torque= 5,
                                        max_kp = 500,
                                        max_kd = 5,
                                        max_current = 16,),

    "RS03": robstride_motor_parameters(max_speed = 20,                          
                                        max_torque = 60,
                                        max_continuous_torque = 20,
                                        max_kp = 5000,
                                        max_kd = 100,
                                        max_current = 43),
}


HOST_ID: int = 0xFD # CAN ID of the teensey

def _make_default_logger(name: str = "MotorMessages") -> logging.Logger:
    logger = logging.getLogger(name)

    if not logger.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter("[%(levelname)s] %(name)s: %(message)s"))
        logger.addHandler(handler)

    logger.setLevel(logging.INFO)
    return logger

class MotorMessages:
    def __init__(self,
                 coms_port: str,
                 coms_freq: int,
                 motor_ids: list[int],
                 min_angles: list[float],
                 max_angles: list[float],
                 motor_types: list[str],
                 logger,
                 serial_baudrate: int =1_000_000,
                 ):
        # NOTE: THE ORDER MATTERS!!! motor_ids, motor_ids, max_angles, and motor_types must have same joint order


        #coms variables
        self.coms_port =  coms_port # which port is the teensey CAN bridge connected to?
        self.coms_freq = coms_freq # serial coms rate
        self.baudrate = serial_baudrate #serial baudrate. must match the teensey

        self.logger = logger or _make_default_logger() # ros logger or Python fallback for printing
        self.motor_ids = motor_ids # CAN ids of the motors. Order must match all other joint parameters
        self.num_motors = len(self.motor_ids)
        
        self.max_angles = max_angles
        self.min_angles = min_angles

        self.warning_temp = 65 #temp where we start printing warning on the temp

        self.temp_warning_interval = 0.5  # seconds
        self.last_temp_warning_time = [0.0] * self.num_motors

        self.comms_warning_counter = [0]*self.num_motors # counter for how many comms warnings recieved.
        self.max_conseq_comms_errors = 5 #max comms errors allowed before flagging an issue

        self.torque_limit_timeout = 10 #max time motor is permitted to exceed the  continuous Torque limit
        self.torque_limit_timers = [0]*self.num_motors

        self.control_modes = None #control mode of the motors. Run set_control_mode to initialize  
        self.control_mode_options = {'operation':0x00,
                                    'position':0x01,
                                    'velocity':0x02,
                                    'current':0x03,
                                    'CSP':0x05}
        
        #handle motor Types
        #ensure valid motor types
        if not all(motor_type in ROBSTRIDE_MOTOR_PARAMS for motor_type in motor_types):
            raise Exception("Motor type not recognized")

        #store if valid
        self.motor_types = motor_types


        #motor limits. We will need to build these lists based on motor_types
        # in order of joints
        self.motor_max_torques = np.zeros(self.num_motors)
        self.motor_max_vel= np.zeros(self.num_motors)
        self.motor_max_currents = np.zeros(self.num_motors)
        self.motor_max_continuous_torque = np.zeros(self.num_motors)
        self.motor_max_kp = np.zeros(self.num_motors)
        self.motor_max_kd = np.zeros(self.num_motors)

        # depending on the motor type, lets build some lists containing limits for the motors.
        for i in range(self.num_motors):
            self.motor_max_torques[i] = ROBSTRIDE_MOTOR_PARAMS[self.motor_types[i]].max_torque
            self.motor_max_vel[i] = ROBSTRIDE_MOTOR_PARAMS[self.motor_types[i]].max_speed
            self.motor_max_currents[i] = ROBSTRIDE_MOTOR_PARAMS[self.motor_types[i]].max_current
            self.motor_max_continuous_torque[i] = ROBSTRIDE_MOTOR_PARAMS[self.motor_types[i]].max_continuous_torque
            self.motor_max_kp[i] = ROBSTRIDE_MOTOR_PARAMS[self.motor_types[i]].max_kp
            self.motor_max_kd[i] = ROBSTRIDE_MOTOR_PARAMS[self.motor_types[i]].max_kd

        #initialize with motors stopped
        self.motor_state = 'Stopped'

        self.max_speed_pp = self.motor_max_vel #for Position mode only

        self.FRAME_SIZE = 13 #bytes in a motor message FROM the teensey.

        self.communicator = SerialCommunicator(port=self.coms_port,
                                  baudrate=self.baudrate,
                                  n_Bytes_to_arduino=12*self.num_motors, #when the thread is started, this code will send all motor commands combined as one long message
                                  n_bytes_from_arduino=self.FRAME_SIZE*self.num_motors,
                                  verbose=False,
                                  logFreq=True,
                                  DesiredFreq=self.coms_freq,
                                  buffer_size=50,
                                  CSVPath=None)
        time.sleep(1)
        self.communicator.drainSerial(FirstMsg=True) #clear any serial msgs

        self.flag_names = [
            'is_running',
            'mode'
            "Uncalibrated fault",
            "Gridlock overload fault",
            "Magnetic coding fault",
            "Over temperature fault",
            "Over current fault",
            "Under voltage fault",
            ]
    ######################################
    # Motor control commands
    ######################################
    def stop_motors(self):
        #stop all motion and cancel coms threads. 

        self.communicator.running = False # stop any coms threads

        #send stop msg
        msg = list(itertools.chain.from_iterable([MotorMessages._stop_motor_msg(motor_id = self.motor_ids[i]) 
                                                  for i in range(self.num_motors)])) #combines all motor commands into 1 serial msg
        self.logger.info("Stopping Motors.")

        self.communicator.SingleWrite(msg)
        time.sleep(.1) #give time to allow motors to stop

        self.motor_state = 'Stopped'

    def set_control_mode(self, modes:str | list[str]):
        #set control mode for all motors.
        
        if isinstance(modes, str):
        # Handle a single string provided for mode

            #ensure motors are stopped
            if(self.motor_state == 'Stopped'):

                #ensure all the modes are valid
                if modes in self.control_mode_options:

                    #store modes
                    self.control_modes = modes
                    
                    msg = list(itertools.chain.from_iterable([MotorMessages._set_control_mode_msg(motor_id = self.motor_ids[i],
                                                                                                 mode =self.control_mode_options[modes])
                                                            for i in range(self.num_motors)]))
                    
                    self.logger.info("Setting Control Mode.")

                    self.communicator.SingleWrite(msg)
                else:
                    raise ValueError("Selected Control Mode Not Found.")
            else:
                self.logger.warning("Could not set motor mode since motors are not stopped")

        elif isinstance(modes, list):
            # Handle a list of modes
            #ensure motors are stopped
            if(self.motor_state == 'Stopped'):

                #ensure all the modes are valid
                if all(mode in self.control_mode_options for mode in modes):

                    #store modes
                    self.control_modes = modes
                    
                    msg = list(itertools.chain.from_iterable([MotorMessages._set_control_mode_msg(motor_id = self.motor_ids[i],
                                                                                                 mode =self.control_mode_options[modes[i]])
                                                            for i in range(self.num_motors)]))
                    
                    self.logger.info("Setting Control Mode.")

                    self.communicator.SingleWrite(msg)
                else:
                    raise ValueError("Selected Control Mode Not Found.")
            else:
                self.logger.warning("Could not set motor mode since motors are not stopped")
        else:
            raise TypeError("modes must be a string or a list of strings")

    def set_max_speed_PP(self, max_speed:list[float]):
        #set max speed for all motors for PP mode only

        assert len(max_speed) == self.num_motors, 'Incorrect number of values'

        max_speed = np.array(max_speed)

        self.max_speed_pp = np.clip(max_speed, 0, self.motor_max_vel)#clamp max speed to be within motor's capabilities

        msg = list(itertools.chain.from_iterable([self._set_max_speed_msg(motor_id = self.motor_ids[i], max_speed=self.max_speed_pp[i])
                                                    for i in range(self.num_motors)]))
        
        self.logger.info(f"Setting Max Speed to {max_speed}.")
        self.communicator.SingleWrite(msg)

        if(self.motor_state != 'Stopped'):
            self.logger.warning("Warning: setting max speed while motor is enabled")

    def set_max_CSP_speed(self, max_speed:list[float]):
        #set max speed for all motors for CSP mode
        
        assert len(max_speed) == self.num_motors, 'Incorrect number of values'

        max_speed = np.array(max_speed)

        self.max_speed_pp = np.clip(max_speed, 0, self.motor_max_vel)#clamp max speed to be within motor's capabilities
        msg = list(itertools.chain.from_iterable([self._set_CSP_max_speed_msg(motor_id = self.motor_ids[i], speed=max_speed[i])
                                                    for i in range(self.num_motors)]))
        self.logger.info(f"Setting Max CSP Speed to {max_speed}.")
        self.communicator.SingleWrite(msg)

        if(self.motor_state != 'Stopped'):
            self.logger.warning("Warning: setting max speed while motor is enabled")

    def zero_motors(self, zeroing: list[bool]):
        #if motor i is marked true for zeroing, send zero cmd, otherwise pad with read_position_msg

        assert len(zeroing) == self.num_motors, 'Incorrect number of values'


        if(self.motor_state == 'Stopped'):

            msg = []
            for i in range(self.num_motors):
                if zeroing[i]:
                    msg = msg+ MotorMessages._zero_motor_msg(motor_id = self.motor_ids[i])
                else:
                    msg = msg+ MotorMessages._read_position_msg(motor_id = self.motor_ids[i])
            self.logger.info(f"Zeroing Motors: {zeroing}")
            self.communicator.SingleWrite(msg)
        else:
            self.logger.warning("Could not zero since motors are not stopped")

    def set_zero_sta_mode(self, zero_sta_mode):
        #set Set_Zero_sta_Mode for all motors
        if(self.motor_state == 'Stopped'):

            msg = list(itertools.chain.from_iterable([self._set_zero_sta_mode_msg(motor_id = self.motor_ids[i],
                                                                                 Zero_sta_mode = zero_sta_mode) 
                                                      for i in range(self.num_motors)]))
            
            self.logger.info(f"Setting zero_sta mode to {zero_sta_mode}.")

            self.communicator.SingleWrite(msg)

        else:
            self.logger.warning("Could not set zero_sta since motors are not stopped")

    def set_max_accel(self, max_accel:list[float]):
        #set maxself.maxSpeed speed for all motors

        assert len(max_accel) == self.num_motors, 'Incorrect number of values'

        if(self.motor_state == 'Stopped'):
            
            msg = list(itertools.chain.from_iterable([MotorMessages._set_max_acc_msg(motor_id = self.motor_ids[i],
                                                                                    max_acc = max_accel[i])
                                                       for i in range(self.num_motors)]))
            
            self.logger.info(f"Setting Max Accel to {max_accel}.")

            self.communicator.SingleWrite(msg)


        else:
            self.logger.warning("Could not set max Accel since motors are not stopped")

    def set_max_torque(self, max_torque:list[float]):
        #set max torque for all motors

        assert len(max_torque) == self.num_motors, 'Incorrect number of values'


        max_torque = np.array(max_torque)

        self.maxTorque = np.clip(max_torque, 0, self.motor_max_torques)#clamp max speed to be within motor's capabilities

        if(self.motor_state == 'Stopped'):

            msg = list(itertools.chain.from_iterable([self._set_max_torque_msg(motor_id = self.motor_ids[i], 
                                                                              max_torque = max_torque[i])
                                                       for i in range(self.num_motors)]))
            
            self.logger.info(f"Setting Max Torque to {max_torque}.")
            self.communicator.SingleWrite(msg)
            
        else:

            self.logger.warning("Could not set max torque since motors are not stopped")

    def set_current_limit(self, limit_current:list[float]):
        # set max current for current control mode
        assert len(limit_current) == self.num_motors, 'Incorrect number of values'

        limit_current = np.array(limit_current)

        limit_current = np.clip(limit_current, 0, self.motor_max_currents)#clamp max speed to be within motor's capabilities

        if(self.motor_state == 'Stopped'):
            
            msg = list(itertools.chain.from_iterable([MotorMessages._set_limit_current_msg(motor_id = self.motor_ids[i], 
                                                                                          limit_Current = limit_current[i]) 
                                                      for i in range(self.num_motors)]))
            
            self.logger.info(f"Setting Max Curret to {limit_current}.")
            self.communicator.SingleWrite(msg)
        else:
            self.logger.warning("Could not set max current since motors are not stopped")
    
    def set_loc_kp(self, loc_kp:list[float]):
        
        assert len(loc_kp) == self.num_motors, 'Incorrect number of values'

        if(self.motor_state == 'Stopped'):
            
            msg = list(itertools.chain.from_iterable([MotorMessages._set_loc_kp_msg(motor_id = self.motor_ids[i],
                                                                                   loc_kp = MotorMessages.clamp(loc_kp[i],0,self.motor_max_kp[i])) 
                                                      for i in range(self.num_motors)]))
            
            self.logger.info(f"Setting loc kp to {loc_kp}.")
            self.communicator.SingleWrite(msg)
        else:
            self.logger.warning("Could not set loc kp since motors are not stopped")

    def set_vel_kp(self, vel_kp:list[float]):
        
        assert len(vel_kp) == self.num_motors, 'Incorrect number of values'

        if(self.motor_state == 'Stopped'):
            msg = list(itertools.chain.from_iterable([MotorMessages._set_vel_kp_msg(motor_id = self.motor_ids[i],
                                                                                    vel_kp = MotorMessages.clamp(vel_kp[i],0,self.motor_max_kd[i])) 
                                                      for i in range(self.num_motors)]))
            
            self.logger.info(f"Setting vel kp to {vel_kp}.")

            self.communicator.SingleWrite(msg)
        else:
            self.logger.warning("Could not set vel kp since motors are not stopped")

    def set_vel_ki(self, vel_ki:list[float]):
        assert len(vel_ki) == self.num_motors, 'Incorrect number of values'

        if(self.motor_state == 'Stopped'):
            msg = list(itertools.chain.from_iterable([MotorMessages._set_vel_ki_msg(motor_id = self.motor_ids[i],
                                                                                   vel_ki = vel_ki[i]) 
                                                      for i in range(self.num_motors)]))
            
            self.logger.info(f"Setting vel ki to {vel_ki}.")
            self.communicator.SingleWrite(msg)
        else:
            self.logger.warning("Could not set vel ki since motors are not stopped")

    def set_vel_filter_gain(self, vel_filter_gain:list[float]):

        assert len(vel_filter_gain) == self.num_motors, 'Incorrect number of values'

        if(self.motor_state == 'Stopped'):
            msg = list(itertools.chain.from_iterable([MotorMessages._set_vel_filter_gain_msg(motor_id = self.motor_ids[i],
                                                                                            vel_filter_gain = vel_filter_gain[i]) 
                                                      for i in range(self.num_motors)]))
            
            self.logger.info(f"Setting vel_filter_gain to {vel_filter_gain}.")

            self.communicator.SingleWrite(msg)

        else:
            self.logger.warning("Could not set vel_filter_gain since motors are not stopped")

    def start_motors(self):
        #send start command to motors
        
        msg = list(itertools.chain.from_iterable([MotorMessages._start_motor_msg(motor_id = self.motor_ids[i])
                                                   for i in range(self.num_motors)]))
        self.logger.info("Starting Motors")
        self.communicator.SingleWrite(msg)
        self.motor_state = 'Started'

    def set_control_reference(self, reference, threading: bool):
        #this function will generate the refernce command based on the motor ControlModes this code will work for ControlMode being a list or string
        #for current, csp or position modes, refernce will be a single numbersetting current, vel, or position reference. 
        # for operation mode, refernce will be a list in form [ref angle, ref vel, feed forward torque, kp, kd]

        if threading and (self.motor_state != 'Started' or not self.communicator.running ):
            self.logger.error('Verify (1) serial threads have started, and (2) motors have been started')
            return

        if isinstance(self.control_modes, str):
            modes = [self.control_modes]*self.num_motors
        else:
            modes = self.control_modes
        
        msg = [] #initialize empty message

        #loop through all motors and build message specific to their control mode
        for idx in range(self.num_motors):

            #handle position control modes
            if modes[idx] in ('position', 'CSP'):
                msg = msg + list(itertools.chain.from_iterable([MotorMessages._set_position_ref_msg(motor_id = self.motor_ids[idx],
                                                              position = MotorMessages.clamp(reference[idx],
                                                                                             self.min_angles[idx],
                                                                                             self.max_angles[idx]))]))
            
            #handle current modes
            elif modes[idx] == 'current':
                
                msg = msg + list(itertools.chain.from_iterable([MotorMessages._set_current_ref_msg(motor_id = self.motor_ids[idx],
                                                              CurrentRef = MotorMessages.clamp(reference[idx],
                                                                                                -self.motor_max_currents[idx],
                                                                                                self.motor_max_currents[idx]))]))
            
            #handle ocm 
            elif modes[idx] == 'operation':

                angle_setpoint= MotorMessages.clamp(reference[idx][0], self.min_angles[idx], self.max_angles[idx])
                vel_setpoint = MotorMessages.clamp(reference[idx][1], -self.motor_max_vel[idx], self.motor_max_vel[idx])
                feed_forward_torque= MotorMessages.clamp(reference[idx][2], -self.motor_max_torques[idx], self.motor_max_torques[idx])
                kp_val =MotorMessages.clamp(reference[idx][3], 0, self.motor_max_kp[idx])
                kd_val =MotorMessages.clamp(reference[idx][4], 0, self.motor_max_kd[idx])


                msg = msg + list(itertools.chain.from_iterable([MotorMessages._set_OCM_ref_msg(motor_id = self.motor_ids[idx],
                                                                angle_setpoints = angle_setpoint,
                                                                vel_setpoint = vel_setpoint,
                                                                feed_forward_torque = feed_forward_torque,
                                                                kp = kp_val,
                                                                kd = kd_val,
                                                                maxVel = self.motor_max_vel[idx],
                                                                maxTorque= self.motor_max_torques[idx],
                                                                kp_max = self.motor_max_kp[idx],
                                                                kd_max = self.motor_max_kd[idx])]))
            elif modes[idx] == 'velocity':
                self.logger.error(f'Velocity control mode has not been implemented... so ... like go implement it or something.')
            else:
                self.logger.error(f'Control mode {modes[idx]} not recognized')

        if threading:
            # threading, so queue up msg
            self.communicator.set_data_to_send(msg)
        else:
            #not threading, use single write
            self.communicator.SingleWrite(msg) 


    ##########################################################
    #  Handle motor states
    #########################################################
    def reset_over_torque_timers(self):
        self.torque_limit_timers = [time.time()]*self.num_motors

    def get_motor_states(self):
        #initialize arrays for motor state variables
        angles = [0.0]*self.num_motors
        vels = [0.0]*self.num_motors
        torques= [0.0]*self.num_motors
        temps= [0.0]*self.num_motors
        coms_error_flags = [False]*self.num_motors

        #get the buffer from the communicator
        msg_buffer = self.communicator.get_buffer() 
        
        if msg_buffer:

            #get just the latest response (LIFO)
            latest_response = list(msg_buffer[-1])

            for i in range(self.num_motors):
                motor_i_response = latest_response[i*self.FRAME_SIZE:self.FRAME_SIZE +i*self.FRAME_SIZE ] # get motor i's state
                
                try:
                    #parse raw message
                    id, angle, vel, torque, temperature, flags, msgType  = self.parse_motor_response(motor_i_response, self.motor_types[i])
                    #if msgType is 0xff, then motor did not respond
                    if msgType == 0xFF:
                        coms_error_flags[i] = True
                        self.comms_warning_counter[i]+=1 #increment counter

                        # if counter is over limit, start issuing warning messages
                        if self.comms_warning_counter[i] > self.max_conseq_comms_errors:
                            self.logger.warning(f"Comms Error on motor {id:#x}")   

                    else:
                        self.comms_warning_counter[i]=0
                    
                    #############################
                    # Protections for the motors.
                    ############################
                    # #over temp warning
                    if temperature > self.warning_temp:
                        now = time.time()
                        if (now - self.last_temp_warning_time[i]) >= self.temp_warning_interval:
                            self.logger.warning(
                                f"Motor {id} temperature is {temperature:.1f}°C "
                                f"(warning threshold: {self.warning_temp}°C)"
                            )
                            self.last_temp_warning_time[i] = now

                    #check flags
                    faults = flags[2:]  # skip is_run and mode_status

                    active_faults = [
                        name for name, active in zip(self.flag_names[2:], faults)
                        if active
                    ]

                    if active_faults:
                        self.logger.warning(f"Faults detected on motor {id:#x}:")
                        for fault in active_faults:
                            print(f"- {fault}")
                  
                   
                except:
                    raise Exception("full message:", latest_response)
                
                angles[i] = angle
                vels[i] = vel
                torques[i] = torque
                temps[i] = temperature
                

        return(angles, vels, torques, temps, coms_error_flags)

    
    def parse_motor_response(self, msg, motor_type = "RS03"):
        #this function is expecting msg to be self.FRAME_SIZE bytes long and to be a 0x02 response from a motor
        # Function will parse the message into the motor id, torque, angle, temp and velocity 
        if len(msg) == self.FRAME_SIZE :
            
            if msg[0] == 0x02: #reply is general motor state (type 2 response)
                
                MotorID = msg[1] #parse id
                flags_byte = msg[2]
                data = msg[-10:] #parse data, 4 for angle float, 2 for vel int, 2 for torque int, 2 for temp int
                
                #get motor type params
                max_vel = ROBSTRIDE_MOTOR_PARAMS[motor_type].max_speed
                max_torque = ROBSTRIDE_MOTOR_PARAMS[motor_type].max_torque

                #convert from bytes to floats
                angle =struct.unpack('<f', bytes(data[:4]))[0]
                vel_int = struct.unpack('>H', bytes(data[4:6]))[0]
                vel = (vel_int / 65535.0) * (2*max_vel) - max_vel #convert to float
                torque_int = struct.unpack('>H', bytes(data[6:8]))[0]
                torque = (torque_int / 65535.0) * (2*max_torque) - max_torque #convert to float
                temperature = struct.unpack('>H', bytes(data[8:10]))[0]/10.

                flags = [bool((flags_byte >> i) & 1) for i in reversed(range(8))]

                return [MotorID, angle, vel, torque, temperature, flags, 0x02]
            
            elif msg[0] == 0x11: #reply is to angle request
                MotorID = msg[2] #parse id
                data = msg[-8:] #parse data

                if(data[0:2] == [0x19,0x70]):
                    angle = struct.unpack('<f', bytes(data[-4:]))[0]

                    vel=None
                    torque=None
                    temperature=None
                    return[MotorID, angle, vel, torque, temperature, 0x11]
                
                else:
                    self.logger.warning(f"Did not recognize parameter id: {data[0:2]}")

            elif msg[0] == 0xFF: #reply Flagging error
                
                MotorID = msg[1]

                #If msg is 0xff, then try to decode as type 2 msg
                try:
                    flags_byte = msg[2]
                    data = msg[-10:]

                    #get motor type params
                    max_vel = ROBSTRIDE_MOTOR_PARAMS[motor_type].max_speed
                    max_torque = ROBSTRIDE_MOTOR_PARAMS[motor_type].max_torque

                    #convert from bytes to floats
                    angle =struct.unpack('<f', bytes(data[:4]))[0]
                    vel_int = struct.unpack('>H', bytes(data[4:6]))[0]
                    vel = (vel_int / 65535.0) * (2*max_vel) - max_vel #convert to float
                    torque_int = struct.unpack('>H', bytes(data[6:8]))[0]
                    torque = (torque_int / 65535.0) * (2*max_torque) - max_torque #convert to float
                    temperature = struct.unpack('>H', bytes(data[8:10]))[0]/10.
                    flags = [bool((flags_byte >> i) & 1) for i in reversed(range(8))]
                except:
                    self.logger.warning(f"Recieved Message with 'No Reply' flag. Attempted to decode as 0x02 msg but failed.")

                return [MotorID, angle, vel, torque, temperature, flags, 0xFF]
            else:
                self.logger.warning(f"Did not recognize reply identifier {msg[0]}")
                self.logger.warning(msg)
        else:
            self.logger.warning(f"Message should be {self.FRAME_SIZE} long")

    

    
        


    ######################################################   
    ##   VVVVVVVVVVVVVV Helper Functions  VVVVVVVVVVVVVVV
    ######################################################
    @staticmethod
    def clamp(x, lo, hi, verbose = False):
        clip = max(lo, min(hi, x))
                
        if verbose and clip!= x:
            print("Warning: Angle clipped")

        return clip
    
    @staticmethod
    def build_eid(cmd: int, host: int, motor: int):
        return [cmd, 0x00, host, motor]
    






    ######################################################   
    ##   VVVVVVVVVVVVVV Motor Message Builders  VVVVVVVVVV
    ######################################################
    @staticmethod
    def _start_motor_msg( motor_id: int):
        """Send 0x03 (start motor)."""
        eid = MotorMessages.build_eid(0x03, HOST_ID, motor_id)
        msg = eid+ [0x00] * 8
        return msg

    @staticmethod
    def _stop_motor_msg(motor_id: int):
        """Send 0x04 (stop)."""
        eid = MotorMessages.build_eid(0x04, HOST_ID, motor_id)
        msg = eid+ [0x00] * 8
        return msg

    def _zero_motor_msg(motor_id: int):
        """Send 0x06 (set current position to zero)."""
        eid = MotorMessages.build_eid(0x06, HOST_ID, motor_id)
        msg = eid+ [0x01] + [0x00] * 7
        return msg

    def _set_control_mode_msg(motor_id: int, mode: int):
        """
        Send 0x12 write-single-patan2arameter to index 0x7050.
        Modes:
          0: operation mode
          1: position mode (PP)
          2: velocity mode
          3: current mode
          5: position mode (CSP)
        """
        eid = MotorMessages.build_eid(0x12, HOST_ID, motor_id)
        data = [0x05, 0x70, 0x00, 0x00, mode & 0xFF, 0x00, 0x00, 0x00]
        msg = eid+ data
        return msg

    def _set_current_ref_msg(self, motor_id: int, CurrentRef: float):
        """
        Send 0x12 write-single-parameter to index 0x7006 (Vel_max).
        Constrained to [0, 20] 
        """
        eid = MotorMessages.build_eid(0x12, HOST_ID, motor_id)
        fb = struct.pack("<f", CurrentRef)  # little-endian float
        data = [0x06, 0x70, 0x00, 0x00] + list(fb)
        msg = eid+ data
        return msg
    
    def _set_loc_kp_msg(motor_id: int, loc_kp: float):
        """
        Send 0x12 write-single-parameter to index 0x701E ().
 
        """
        eid = MotorMessages.build_eid(0x12, HOST_ID, motor_id)
        fb = struct.pack("<f", loc_kp)  # little-endian float
        data = [0x1E, 0x70, 0x00, 0x00] + list(fb)
        msg = eid+ data
        return msg

    def _set_vel_ki_msg(motor_id: int, vel_ki: float):
        """
        Send 0x12 write-single-parameter to index 0x7020 ().
 
        """
        eid = MotorMessages.build_eid(0x12, HOST_ID, motor_id)
        fb = struct.pack("<f", vel_ki)  # little-endian float
        data = [0x20, 0x70, 0x00, 0x00] + list(fb)
        msg = eid+ data
        return msg

    def _set_OCM_ref_msg(motor_id: int,
                        angle_setpoints: float, 
                        vel_setpoint: float, 
                        feed_forward_torque: float,
                        kp: float, 
                        kd: float,
                        maxVel: float,
                        maxTorque:float,
                        kp_max: float,
                        kd_max: float):
        # eid : [bit 28-24: 0x01
        #        bits 23-8: feed_forward_torque (0-65535) --> (-max torque to max torque)
        #        bit 7-0: motor_id  ]
        # data: [bytes 0-1 :angle (0-65535) --> (-4pi to 4pi)
        #       bytes 2-3 : target ang vel (0-65535) --> (-max vel to max vel)
        #       bytes 4-5 : kp (0-65535) --> (0. to kp_max.)
        #       bytes 6-7 : kd (0-65535) --> (0. to kd_max.)]

        def float_to_uint(x: float, x_min: float, x_max: float, bits: int = 16) -> int:
            """
            Maps a float in [x_min, x_max] to an unsigned integer in [0, 2^bits - 1].
            Values outside the range are clipped.
            """
            x = max(min(x, x_max), x_min)

            span = x_max - x_min
            max_int = (1 << bits) - 1

            return int(round((x - x_min) * max_int / span))
        
        angle_uint16 = float_to_uint(angle_setpoints, -4*np.pi, 4*np.pi)
        vel_uint16 = float_to_uint(vel_setpoint, -maxVel, maxVel)
        torque_uint16 = float_to_uint(feed_forward_torque, -maxTorque, maxTorque)
        kp_uint16 = float_to_uint(kp, 0, kp_max)
        kd_uint16 = float_to_uint(kd, 0, kd_max)

        angle_bytes = struct.pack('>H', angle_uint16)
        vel_bytes = struct.pack('>H', vel_uint16)
        torque_bytes = struct.pack('>H', torque_uint16)
        kp_bytes = struct.pack('>H', kp_uint16)
        kd_bytes = struct.pack('>H', kd_uint16)
       
        eid = [0x01] + list(torque_bytes) + [motor_id]
        data = list(angle_bytes) + list(vel_bytes) + list(kp_bytes) + list(kd_bytes)

        msg = eid+ data
        return msg
    
    def _set_vel_kp_msg(motor_id: int, vel_kp: float):
        """
        Send 0x12 write-single-parameter to index 0x701F (Vel_max).
 
        """
        eid = MotorMessages.build_eid(0x12, HOST_ID, motor_id)
        fb = struct.pack("<f", vel_kp)  # little-endian float
        data = [0x1F, 0x70, 0x00, 0x00] + list(fb)
        msg = eid+ data
        return msg
    
    def _set_vel_filter_gain_msg(motor_id: int, vel_filter_gain: float):
        """
        Send 0x12 write-single-parameter to index 0x7021 
 
        """
        eid = MotorMessages.build_eid(0x12, HOST_ID, motor_id)
        fb = struct.pack("<f", vel_filter_gain)  # little-endian float
        data = [0x21, 0x70, 0x00, 0x00] + list(fb)
        msg = eid+ data
        return msg
    
    def _set_max_speed_msg(self,motor_id: int, max_speed: float):
        """
        Send 0x12 write-single-parameter to index 0x7024 (Vel_max).
        Constrained to [0, 20] 
        """
        eid = MotorMessages.build_eid(0x12, HOST_ID, motor_id)
        fb = struct.pack("<f", max_speed)  # little-endian float
        data = [0x24, 0x70, 0x00, 0x00] + list(fb)
        msg = eid+ data
        return msg

    def _set_max_acc_msg(motor_id: int, max_acc: float):
        """Send 0x12 write-single-parameter to index 0x7025 (acc_set)."""
        eid = MotorMessages.build_eid(0x12, HOST_ID, motor_id)
        fb = struct.pack("<f", max_acc)
        data = [0x25, 0x70, 0x00, 0x00] + list(fb)
        msg = eid+ data
        return msg
    
    def _set_max_torque_msg(self, motor_id: int, max_torque: float):
        """Send 0x12 write-single-parameter to index 0x700B."""
        eid = MotorMessages.build_eid(0x12, HOST_ID, motor_id)
        fb = struct.pack("<f", max_torque)
        data = [0x0B, 0x70, 0x00, 0x00] + list(fb)
        msg = eid+ data
        return msg
    
    def _set_limit_current_msg(motor_id: int, limit_Current: float):
        """Send 0x12 write-single-parameter to index 0x7018 (limit_torque)."""
        eid = MotorMessages.build_eid(0x12, HOST_ID, motor_id)
        fb = struct.pack("<f", limit_Current)
        data = [0x18, 0x70, 0x00, 0x00] + list(fb)
        msg = eid+ data
        return msg

    def _set_zero_sta_mode_msg(self, motor_id: int, Zero_sta_mode: int):
        """Send 0x12 write-single-parameter to index 0x7029 (limit_torque)."""
        if Zero_sta_mode != 0x01 and Zero_sta_mode != 0x00:
            self.logger.warning("WARNING: Zero_sta_mode must be 0x01 or 0x00. Defaulted to 0x01")
            Zero_sta_mode=0x01

        eid = MotorMessages.build_eid(0x12, HOST_ID, motor_id)

        data = [0x29, 0x70, 0x00, 0x00, Zero_sta_mode, 0x00, 0x00, 0x00]
        msg = eid+ data
        return msg
    
    def _set_position_ref_msg(motor_id: int, position: float):
        """
        Send 0x12 write-single-parameter to index 0x7016 (loc_ref).
        """
        eid = MotorMessages.build_eid(0x12, HOST_ID, motor_id)
        fb = struct.pack("<f", position)
        data = [0x16, 0x70, 0x00, 0x00] + list(fb)
        msg = eid+ data
        return msg

    def _set_CSP_max_speed_msg(self, motor_id: int, speed: float):
        """
        Send 0x12 write-single-parameter to index 0x7017 (speed_ref).
        Constrained to [0, 20] like your Arduino code.
        """
        eid = MotorMessages.build_eid(0x12, HOST_ID, motor_id)
        fb = struct.pack("<f", speed)
        data = [0x17, 0x70, 0x00, 0x00] + list(fb)
        msg = eid+ data
        return msg

    def _read_position_msg(motor_id: int):
        """
        Send 0x12 write-single-parameter to index 0x7016 (loc_ref).
        """
        eid = MotorMessages.build_eid(0x11, HOST_ID, motor_id)
        data = [0x19, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00] 
        msg = eid+ data
        return msg

   
    
    