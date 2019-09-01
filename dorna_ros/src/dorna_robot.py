#!/usr/bin/env python3
from dorna import Dorna
import dorna_cmd_utils 
from dorna_enums import DornaConnection, DornaMovement, DornaState
import json
import logging
import numpy as np 
import os
import time
import yaml

from dorna_exceptions import ConnectionException, HomingException

################################
####      Logging Info      ####
################################
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
formatter = logging.Formatter('[%(levelname)s]:[%(name)s]:%(message)s')
stream_handler = logging.StreamHandler()
stream_handler.setFormatter(formatter)
logger.addHandler(stream_handler)

################################
####       DornaRobot       ####
################################
class DornaRobot:
    def __init__(self):
        self._config_path = os.path.dirname(os.path.realpath(__file__))+str("/config.yaml")
        self.params = list(yaml.safe_load(open(self._config_path, 'r')).keys())
        self.joint_names = ['j0', 'j1', 'j2', 'j3', 'j4']
        self.axis_names = ['x', 'y', 'z', 'a', 'b']

        self._robot = Dorna(self._config_path)
        
        self._default_home = dict(zip(self.joint_names, [0,145,-90,0,0]))
        self.connection_status = None
        self._elevator_demo = False 
        self._hand = True
        self.triggered = False

    def init(self, home = True):
        '''
        Connect to the robot and then home or set the stepper motors
        '''
        try:
            self.connect()
        except ConnectionException:
            logger.error("ConnectionError [dorna_robot]: There was a connection issue. Check Robot and try again")
            raise
        else:
            if home:
                self.home()
            else:
                self.set_joint()
            logger.info("Robot startup success")
            
################################
#### Command calls to robot ####
################################
    def calibrate(self):
        '''
        Set angle offsets for homeing 
        '''
        joints = self.get_joint_angles()
        self._robot.calibrate(joints)
        self._robot.save_config(save_path= self._config_path)
        self.set_joint()

    def connect(self, usb = None):
        '''
        Connects to Dorna\n
        Note: Make sure to setup dorna udev rule
        '''
        logger.info('Connecting to Dorna...')

        if not usb:
            usb = "/dev/actuators/dorna" 
        
        response = json.loads(self._robot.connect(port_name=usb))
        self.connection_status = self.get_connection_status()


        while response["connection"] == DornaConnection.CONNECTING:
            time.sleep(0.5)
            response = json.loads(self._robot.device())
            
        # if response["connection"] == DornaConnection.CONNECTED:

        if response["connection"] == DornaConnection.DISCONNECTED:
            logger.error("ConnectionError")
            logger.info("There was a connection issue. Check Robot and try again")
            raise ConnectionException()

    def disconnect(self):
        '''
        Disconects from Arduino
        '''
        self._robot.disconnect()

    def halt(self):
        '''
        Stops Dorna and delets queue
        '''
        self._robot.halt()

    def force_fulfill(self, joint, desired_pose):
        '''
        Waits until joint angle equal to desired pose\n
        Note: only works for single joint and angle
        '''
        # TODO: Make this better
        not_in_position = True
        while not_in_position:
            joint_angles = self.get_joint_angles()
            if int(joint_angles[joint]) == desired_pose:
                not_in_position = False

    def force_fulfill_joints(self, desired):
        not_in_position = True
        while not_in_position and not self.triggered:
            current = self.get_joint_angles()
            check = np.linalg.norm(np.diff(np.row_stack((current, desired)), axis=0))
            not_in_position = check >= 0.1
    
    def force_fulfill_xyzab(self, desired):
        not_in_position = True
        while not_in_position and not self.triggered:
            current = self.get_cartesian_position()
            check = np.linalg.norm(np.diff(np.row_stack((current, desired)), axis=0))
            not_in_position = check >= 0.1
            

    def elevator_demo_safe_home(self, i):
        '''
        Turn the base 90 degrees before calibrating next motor\n 
        Then move base back to origin
        '''
        if i == 1:
            self.move_joints(
                path="joint", 
                movement = DornaMovement.GLOBAL.value, 
                speed = DornaMovement.DEFAULT_JOINT_SPEED.value, 
                joint_angles = {'j0': -90})

            self.force_fulfill(0, -90)
        elif i == 2: 
            self.move_joints(
                path="joint", 
                movement = DornaMovement.GLOBAL.value, 
                speed = DornaMovement.DEFAULT_JOINT_SPEED.value, 
                joint_angles = {'j0': 0})
            self.force_fulfill(0, 0)
        
    def home(self):
        '''
        Calibrate stepper motors
        '''
        logger.info('Homing Robot...')
        try:
            ## Only homing joints 0-2 and setting joints 3-4 so that tool head does not colide with robot
            if self._hand:
                n = 3
                self.set_joint({"j3": 0, "j4": 0})
            else:
                n = len(self.joint_names)

            for i in range(n):
                if self._elevator_demo:
                    self.elevator_demo_safe_home(i)

                motor_name = "j" + str(i)
                response = json.loads(self._robot.home(motor_name))
                if response[motor_name] != 1:
                    raise HomingException()

            logger.info('Finished Homing Robot...')
        except HomingException:
            logger.error('HomingError [dorna_robot]: Motor {} was unable to home'.format(i))

    def jog(self, path, cmd_dict):
        '''
        Jogs single motor or robot in single direction\n
        path = 'joint' or 'line'\n
        cmd_dict = dictionary of joint_name or axis_name as keys and angle or position as values
        '''
        logger.info('Jogging...')
        if path == "joint":
            speed = DornaMovement.DEFAULT_JOINT_SPEED
        elif path == "line":
            speed = DornaMovement.DEFAULT_XYZ_SPEED
        
        if list(cmd_dict.keys())[0] in self.joint_names:
            self.move_joints(
                path = path, 
                movement = DornaMovement.RELATIVE, 
                speed = speed, 
                joint_angles = cmd_dict
            )
       
        elif list(cmd_dict.keys())[0] in self.axis_names:
            self.move_xyzab(
                path = path, 
                movement = DornaMovement.RELATIVE, 
                speed = speed, 
                xyzab = cmd_dict
            )

    def move_xyzab(self, path, movement, speed, xyzab, fulfill=True):
        '''
        path = 'joint' or 'line'\n
        movement = relative or global\n
        speed = units/second\n
        xyzab = can be a list of all positions or a dict of single direction
        '''
        try:
            if type(xyzab) == list and len(xyzab) == 5:
                cmd_dict = dict(zip(self.axis_names, xyzab))
            elif type(xyzab) == dict:
                cmd_dict = xyzab
                pass
            else:
                raise TypeError
        except:
            raise

        parameters = dorna_cmd_utils.get_move_msg(
            path = path, 
            movement = movement, 
            speed = speed, 
            cmd_dict = cmd_dict)

        cmd = dorna_cmd_utils.get_play_msg(
            command = "move", 
            parameters=parameters, 
            fulfill=fulfill)
        logger.info(cmd)

        self.play(cmd)

    def move_joints(self, path, movement, speed, joint_angles, fulfill = True):
        '''
        path = 'joint' or 'line'\n
        movement = relative or global\n
        speed = units/second\n
        joint_angles = can be a list of all angles or a dict of some angles
        '''
        try:
            if type(joint_angles) == list and len(joint_angles) == 5:
                cmd_dict = dict(zip(self.joint_names, joint_angles))
                # print(cmd_dict)
            elif type(joint_angles) == dict:
                cmd_dict = joint_angles
            else:
                raise TypeError
        except:
            raise

        parameters = dorna_cmd_utils.get_move_msg(
            path = path, 
            movement = movement, 
            speed = speed, 
            cmd_dict = cmd_dict)

        cmd = dorna_cmd_utils.get_play_msg(
            command = "move", 
            parameters=parameters, 
            fulfill=fulfill)
        logger.info(cmd)

        self.play(cmd)

    def move_to_home(self):
        '''
        Moves Dorna to default home position at default speed
        '''
        self.move_joints(
            path = "joint", 
            movement = DornaMovement.GLOBAL, 
            speed = DornaMovement.DEFAULT_JOINT_SPEED, 
            joint_angles = self._default_home)

    def move_to_zeropose(self):
        '''
        Moves Dorna to joint angles all 0
        '''
        cmd_dict = dict.fromkeys(self.joint_names, 0)
        self.move_joints(
            path = "joint", 
            movement = DornaMovement.GLOBAL, 
            speed = DornaMovement.DEFAULT_JOINT_SPEED, 
            joint_angles = cmd_dict)
        
    def pause(self):
        '''
        Pauses Dorna command queue
        '''
        self._robot.pause()

    def play(self, path):
        '''
        Adds path to command queue
        '''
        logger.info('Playing Path...')
        
        try:
            self._robot.play(path, append= True)
        except KeyboardInterrupt:
            self.halt()
            pass
    
    def set_joint(self, joint_dict = None):
        '''
        Sets values for motor angle
        '''
        if joint_dict:
            self._robot.set_joint(joint_dict)
        else:
            logger.debug(self._default_home)
            self._robot.set_joint(self._default_home)

    def terminate(self):
        '''
        Kills threads in Dorna 
        '''
        self._robot.terminate()

    def xyz_to_joint(self, xyzab):
        '''
        Checks if xyz position is in workspace\n
        If true then returns inverse kinematics
        '''
        if self._robot._config["unit"]["length"] == "mm":
            xyzab[0:3] = [self._robot._mm_to_inch(xyz) for xyz in xyzab[0:3]]

        joint_response_dict = self._robot._xyz_to_joint(np.asarray(xyzab))

        if joint_response_dict['status'] == 0:
            logger.info("Can achieve xyz position and safe")
            return joint_response_dict['joint']
        elif joint_response_dict['status'] == 1:
            logger.info("Can achieve xyz position but NOT safe, USE WITH CAUTION")
            return joint_response_dict['joint']
        elif joint_response_dict['status'] == 2:
            logger.error("Cannot achieve xyz position")
            return None
    
####################################
#### Get information from robot ####
####################################                
    def get_cartesian_position(self):
        '''
        Get cartesian postion of end effector\n
        Return in units[mm, inch] set by params
        '''
        pos = json.loads(self._robot.position("xyz"))

        return pos
    
    def get_connection_status(self):
        '''
        Get connection statues
        '''
        status = json.loads(self._robot.device())
        return status["connection"] == DornaConnection.CONNECTED

    def get_homed_status(self):
        '''
        Returns if stepper motors calibrated
        '''
        homed = json.loads(self._robot.homed())

        return homed

    def get_joint_angles(self, units = 'deg'):
        '''
        Returns motor joint angles
        '''
        joints = json.loads(self._robot.position("joint"))
        if units == 'rad':
            joints = [joint*np.pi/180. for joint in joints]

        return joints
    
    def get_state(self):
        '''
        Returns if robot is moving
        '''
        state = json.loads(self._robot.device())['state']
        
        return state
    
    def get_param(self, param_name):
        '''
        Gets value of param from param_list
        '''
        if param_name in self.params:
            if param_name == 'calibrate':
                logger.warn('Dorna param calibrate is only a setting function')
                return
            if hasattr(self._robot, param_name):
                call = getattr(self._robot, param_name, None)
                response = json.loads(call())
                # logger.info("Get robot param: "+str(response))
                return response
            else: 
                logger.warn('Dorna does not have method: {}'.format(param_name))
        else: 
            logger.warn('[{}] not in param list'.format(param_name))

    def set_param(self, param_name, value):
        '''
        Sets value of param from param_list
        '''
        if param_name in self.params:
            attr = 'set_'+param_name
            if hasattr(self._robot, attr):
                call = getattr(self._robot, attr, None)
                response = call(value)
                logger.info("Set robot param: "+str(response))
            else: 
                logger.warn('Dorna does not have method: {}'.format(attr))
        else:
            logger.warn("[{}] not in param list".format(param_name))

        
if __name__ == "__main__":
    robot = DornaRobot()
    print(robot._config_path)
    robot.init(home=False)
    print(robot.set_param('motion', {'ct': 2.33444}))
    
    
    
    




