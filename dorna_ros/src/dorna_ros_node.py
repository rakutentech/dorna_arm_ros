#!/usr/bin/env python3

import threading
from threading import Thread
import re
import rospy

from dorna_ros.msg import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Bool

from dorna_ros.srv import *
from std_srvs.srv import Empty

from dorna_ros.cfg import DornaConfig

from dorna_cmd_utils import get_traj_msg #Useful functions for converting trajectory messages to Dorna format
from dorna_robot import DornaRobot #Instance of Dorna class with added utilities
from dorna_tool_head import DornaToolhead #Instance of ToolHead 
from dorna_exceptions import ConnectionException #Custome Exceptions for Dorna_ROS

from dynamic_reconfigure.server import Server

import dynamic_reconfigure.client

class DornaRos:
    def __init__(self, toolhead=False):
        self._ns = 'dorna'
        self._robot = DornaRobot()
        if toolhead:
            self._tool_head = DornaToolhead()
        else:
            self._tool_head = None
        self._config = None
        self._param_server = Server(DornaConfig, self.update_robot_params)
        
        self.connection_status = self._robot.get_connection_status()        
        ####################################
        ####    robot_info Publishers   ####
        ####################################
        self._pub_topics = ['cartesian_position', 'joint_angles',
                            'homed_status', 'joint_states', 'dorna_params', 'state', 'triggered']
        pub_msg_types = [DornaPos, DornaJoint,
                            DornaHomed, JointState, DornaInfo, Int32, Bool]
        self._publisher_list = {}
        for topic, msg_type in zip(self._pub_topics, pub_msg_types):
            self._publisher_list[topic] = rospy.Publisher(self._ns+'/robot_info/'+topic, msg_type, queue_size = 10)

        ####################################
        ####    robot_cmd Services      ####
        ####################################
        self._srv_topics = ['connect', 'disconnect',
                            'home', 'set_joint', 'play', 
                            'pause', 'halt', 'calibrate', 
                            'jog', 'xyz_check', 'xyz_to_joint',
                            'move_joints', 'move_xyzab', 'digital_out', 'move_to_home']
        srv_msg_types = [DornaConnect, DornaDisconnect,
                            DornaHome, DornaSet, DornaPlay,
                            DornaPause, DornaHalt, DornaCalibrate, 
                            DornaJog, DornaXYZCheck, DornaXYZtoJoint,
                            DornaMoveJoints, DornaMoveXYZ, DornaOutputPin, Empty]
        self._service_list = {}
        for topic, msg_type in zip(self._srv_topics, srv_msg_types):
            callback = getattr(self, topic+"_srv_callback", None)
            self._service_list[topic] = rospy.Service(self._ns+'/robot_cmd/'+topic, msg_type, callback)
                
        ########################
        ####    Ros Setup   ####
        ########################
        self._ros_rate = rospy.Rate(10)  # [hz]

        ################################
        ####    Dorna Threads       ####
        ################################
        self.publisher_thread = Thread(target=self.run_publisher)
        if self._tool_head is not None:
            self.toolhead_thread = Thread(target=self._tool_head.sensor_check)
       
####################
####    Init    ####
####################
    def init(self):
        '''
        Connects to robot, starts publisher and sensor threads, starts homing process for dorna
        '''
        self._robot.connect()
        self.start_threads()
        self._robot.home()

    def start_threads(self):
        '''
        Starts Publisher thread: publishes all useful dorna information\n
        Starts Toolhead Sensor check: If toolhead is connected will start sensor publishing
        '''
        self.connection_status = self._robot.get_connection_status()
        current_threads = threading.enumerate()
        if self.publisher_thread in current_threads: 
            rospy.loginfo('Publisher thread already started')
        else:
            self.publisher_thread.start()
            rospy.loginfo('Starting Publisher thread')

        if self.toolhead_thread not in current_threads and self._tool_head is not None:
            self.toolhead_thread.start()
            rospy.loginfo('Starting Hand sensor thread')

        else:
            rospy.loginfo('Hand sensor thread already started')
#######################
####    Threads    ####
#######################
    def run_publisher(self):
        '''
        Publisher thread main loop
        '''
        rospy.loginfo_once("Starting publisher Thread")
        try:
            while not rospy.is_shutdown() and self.connection_status:
                self.connection_status = self._robot.get_connection_status()
                self.publish()
                self._ros_rate.sleep()
        except KeyboardInterrupt:
            pass

    def publish(self):
        '''
        Publishes all dorna pub_topics 
        '''
        for topic in self._pub_topics:
            msg = getattr(self, "get_"+topic+"_msg", None)
            self._publisher_list[topic].publish(msg())

####################
####    Safety  ####
####################
    def disconnect(self):
        '''
        Disconnects from robot but does not stop Dorna API thread or shutdown ROS
        '''
        self._robot.disconnect()

    def terminate(self):
        '''
        Stops Dorna API thread
        '''
        self._robot.terminate()

    def quit(self):
        '''
        Disconnects and terminates Dorna robot and shutsdown ROS node
        '''
        self.disconnect()
        self.terminate()
        self.publisher_thread.join()
        
        if self._tool_head is not None:
            self.toolhead_thread.join()

        rospy.signal_shutdown('Shutting Down')

################################
####    Publisher Messages  ####
################################
    def get_cartesian_position_msg(self):
        pos = self._robot.get_cartesian_position()

        pos_msg = DornaPos()
        pos_msg.x = pos[0]
        pos_msg.y = pos[1]
        pos_msg.z = pos[2]
        pos_msg.a = pos[3]
        pos_msg.b = pos[4]

        return pos_msg

    def get_default_speed_msg(self):
        speed = self._robot.get_param('default_speed')

        speed_msg = DornaSpeed()
        speed_msg.xyz = speed['default_speed']['xyz']
        speed_msg.joint = speed['default_speed']['joint']
        
        return speed_msg

    def get_default_jerk_msg(self):
        jerk = self._robot.get_param('default_jerk')

        jerk_msg = DornaJerk()
        jerk_msg.xyz = jerk['default_jerk']['xyz']
        jerk_msg.joint = jerk['default_jerk']['joint']

        return jerk_msg

    def get_dorna_params_msg(self):
        dorna_params_msg = DornaInfo()
        dorna_params_msg.default_speed = self.get_default_speed_msg()
        dorna_params_msg.default_jerk = self.get_default_jerk_msg()
        dorna_params_msg.joint_limits = self.get_joint_limits_msg()
        dorna_params_msg.motion_params = self.get_motion_params_msg()
        dorna_params_msg.toolhead_length = self.get_toolhead_msg()
        dorna_params_msg.units = self.get_units_msg()

        return dorna_params_msg

    def get_homed_status_msg(self):
        homed = self._robot.get_homed_status()

        homed_msg = DornaHomed()
        homed_msg.j0 = homed['j0']
        homed_msg.j1 = homed['j1']
        homed_msg.j2 = homed['j2']
        homed_msg.j3 = homed['j3']
        homed_msg.j4 = homed['j4'] 

        return homed_msg

    def get_joint_angles_msg(self):
        joint = self._robot.get_joint_angles()

        joint_msg = DornaJoint()
        joint_msg.j0 = joint[0]
        joint_msg.j1 = joint[1]
        joint_msg.j2 = joint[2]
        joint_msg.j3 = joint[3]
        joint_msg.j4 = joint[4] 

        return joint_msg

    def get_joint_limits_msg(self):
        joint_limits = self._robot.get_param('limit')

        jl_msg = DornaLimits()
        jl_msg.j0 = joint_limits['limit']['j0']
        jl_msg.j1 = joint_limits['limit']['j1']
        jl_msg.j2 = joint_limits['limit']['j2']
        return jl_msg

    def get_joint_states_msg(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.header.frame_id = "dorna_base"

        joint_state_msg.name = ['dorna_base_to_dorna_arm_0', 
                                'dorna_arm_0_to_dorna_arm_1', 
                                'dorna_arm_1_to_dorna_arm_2',
                                'dorna_arm_2_to_dorna_arm_3',
                                'dorna_arm_3_to_dorna_arm_4']

        joint_angles = self._robot.get_joint_angles(units = 'rad')

        joint_state_msg.position = [joint_angles[0], 
                                    joint_angles[1], 
                                    joint_angles[2], 
                                    joint_angles[3], 
                                    joint_angles[4]]

        joint_state_msg.velocity = []
        joint_state_msg.effort = []
        return joint_state_msg

    def get_motion_params_msg(self):
        motion_params = self._robot.get_param('motion')

        motion_params_msg = DornaMotion()
        motion_params_msg.ct = motion_params['motion']['ct']
        motion_params_msg.gpa = motion_params['motion']['gpa']
        motion_params_msg.jt = motion_params['motion']['jt']

        return motion_params_msg

    def get_state_msg(self):
        state = self._robot.get_state()

        state_msg = Int32()
        state_msg.data = int(state)

        return state

    def get_toolhead_msg(self):
        toolhead = self._robot.get_param('toolhead')

        toolhead_msg = DornaToolHead()
        toolhead_msg.x = toolhead['toolhead']['x']

        return toolhead_msg

    def get_triggered_msg(self):
        triggered_msg = Bool()
        if self._tool_head is not None:
            triggered = self._tool_head.triggered
            self._robot.triggered = triggered

            
            triggered_msg.data = triggered
        else:
            triggered_msg.data = False

        return triggered_msg

    def get_units_msg(self):
        units = self._robot.get_param('unit')

        units_msg = DornaUnits()
        units_msg.unit = units['unit']['length']

        return units_msg   

################################
####    Service Callbacks   ####
################################
    def calibrate_srv_callback(self, req):
        rospy.loginfo("[Service Callback] Calibrate")
        self._robot.calibrate()
        return True

    def connect_srv_callback(self, req):
        rospy.loginfo("[Service Callback] Connect")
        if not self.connection_status:
            try:
                self._robot.connect()
                self.start_threads()
                return True
            except:
                return False
        else:
            rospy.loginfo("Already connected to the robot")
            return True

    def disconnect_srv_callback(self, req):
        rospy.loginfo("[Service Callback] Disconnect")
        if self.connection_status:
            self.disconnect()
        else:
            rospy.loginfo("Robot is not connected")
        return True

    def halt_srv_callback(self, req):
        rospy.loginfo("[Service Callback] Halt")
        self._robot.halt()
        return True

    def home_srv_callback(self, req):
        rospy.loginfo("[Service Callback] Home")
        self._robot.home()
        if self._tool_head is not None:
            self._tool_head.update_pitch()
            self._tool_head.update_roll()
            self._robot.set_joint({'j3': 0, 'j4': 0})
        return True

    def jog_srv_callback(self, req):
        rospy.loginfo("[Service Callback] Jog")
        self._robot.jog(req.path, {req.axis: req.step_size})
        return True
        
    def pause_srv_callback(self, req):
        rospy.loginfo("[Service Callback] Pause")
        self._robot.pause()
        return True

    def play_srv_callback(self, trajectory):
        rospy.loginfo("[Service Callback] Play")
        traj = get_traj_msg(trajectory)
        self._robot.play(traj)
        return True
    
    def set_joint_srv_callback(self, req):
        rospy.loginfo("[Service Callback] Set")
        if req.joint_angles == []:
            self._robot.set_joint()
        else:
            self._robot.set_joint(dict(zip(self._robot.joint_names, req.joint_angles)))
        return True

    def quit_srv_callback(self, req):
        self.quit()
        return True
    
    def xyz_to_joint_srv_callback(self, req):
        #TODO: make this better
        rospy.loginfo("[Service Callback] xyz_to_joint")
        xyzab = [req.xyz[0], req.xyz[1], req.xyz[2], 0, 0]
        joint_angles = self._robot.xyz_to_joint(xyzab)
        return [joint_angles]

    def xyz_check_srv_callback(self, req):
        #TODO: make this better
        rospy.loginfo("[Service Callback] XYZ_check")
        xyzab = [req.xyz[0], req.xyz[1], req.xyz[2], 0, 0]
        response = self._robot.xyz_to_joint(xyzab)
        print(response)
        if None not in response:
            return True
        else:
            return False

    def move_joints_srv_callback(self, req):
        #TODO: make this better
        rospy.loginfo("[Service Callback] move_joint")
        path = str(req.path)
        movement = req.movement
        speed = req.speed
        joint_angles = list(req.joint_angles)
        fulfill = req.fulfill
        self._robot.move_joints(path, movement, speed, joint_angles, fulfill)
        if fulfill:
            if movement:
                desired = [x+y for x,y in zip(self._robot.get_joint_angles(), joint_angles)]
            else: 
                desired = joint_angles  
            self._robot.force_fulfill_joints(desired)

        return True

    def move_xyzab_srv_callback(self, req):
        #TODO: make this better
        rospy.loginfo("[Service Callback] move_xyzab")
        path = req.path
        movement = req.movement
        speed = req.speed
        xyzab = list(req.xyzab)
        fulfill = req.fulfill
        self._robot.move_xyzab(path, movement, speed, xyzab, fulfill)
        if fulfill:
            if movement:
                desired = [x+y for x,y in zip(self._robot.get_cartesian_position(), xyzab)]
            else: 
                desired = xyzab  
            self._robot.force_fulfill_xyzab(desired)

        return True
        
    def move_to_home_srv_callback(self, req):
        rospy.loginfo("[Service Callback] move_to_home Open loop")
        self._robot.move_to_home()
        return {}

    def digital_out_srv_callback(self, req):
        rospy.loginfo("[Service Callback] digital out")
        self._robot.set_param('io', {req.pin: int(req.state)})
        return True

################################
####    Parameter Callback ####
################################
    def update_robot_params(self, config, level):
        if self._config == None:
           self._config = config
        
        for key in config.keys():
            if self._config[key] == config[key]:
                pass
            else:
                split = re.split("\B__", key)
                param_name = split[0]
                value_id = split[1]

                value = {}
                if len(split) == 2:
                    value[value_id] = config[key]
                else:
                    value_index = int(split[2])
                    prev_value = self._robot.get_param(param_name)[param_name][value_id]
    
                    value[value_id] = prev_value
                    value[value_id][value_index] = config[key]

                rospy.loginfo("""Reconfigure Request {}: {}""".format(param_name, value))

                self._robot.set_param(param_name, value)
        
        self._config = config

        return config


if __name__ == "__main__":    
    rospy.init_node('dorna')
    rospy.loginfo('Starting node "dorna"')
    robot = DornaRos()
    init = False
    try:
        if init:
            robot.init()
    except ConnectionException:
        raise
    else:
        try:
            rospy.spin()
        except (rospy.ROSInterruptException, ConnectionException, KeyboardInterrupt):
            raise
        finally:
            rospy.loginfo('Disconnecting and Terminating Robot Object')
            robot.disconnect()
    finally:
        robot.terminate()