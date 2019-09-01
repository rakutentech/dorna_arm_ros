#!/usr/bin/env python3

from enum import IntEnum
import time
import rospy
from threading import Thread

from dorna_enums import HumanResponse, DornaMovement

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import RobotState, RobotTrajectory, DisplayTrajectory, CollisionObject
from dorna_api.msg import DornaPos, DornaJoint, DornaLimits, DornaHomed, DornaToolHead, DornaSpeed, DornaJerk
from pix_2_world.msg import ButtonsPix, ButtonsWorld, Pix, World
from std_msgs.msg import Float64, Int32, Bool, Header, String, Empty

from dorna_api.srv import DornaConnect, DornaDisconnect, DornaHome, DornaSet, DornaPlay, DornaPause
from dorna_api.srv import DornaHalt, DornaCalibrate, DornaJog, DornaErase, DornaView
from dorna_api.srv import DornaPlanCmd, DornaXYZ, DornaXYZCheck, DornaXYZtoJoint, TfTransform
from dorna_api.srv import DornaMoveJoints, DornaMoveXYZ, DornaOutputPin
from pix_2_world.srv import Pix2World, NNService


class HumanRos:
    def __init__(self):
        self._ns = 'dorna_real'
        self._joint_names = ['j0', 'j1', 'j2', 'j3', 'j4']
        self._axis_names = ['x', 'y', 'z', 'a', 'b']

        self.trajectory = None
        self._button_data = None
        self.connection_status = False

        ##############################################
        ####        Human Service Proxies        #####
        ##############################################
        robot_srv_topics = ['connect', 'disconnect',
                            'home', 'set_joint', 'play', 
                            'pause', 'halt', 'calibrate',
                            'jog', 'xyz_check', 'xyz_to_joint',
                            'move_joints', 'move_xyzab', 'digital_out']
        robot_srv_msg_types = [DornaConnect, DornaDisconnect,
                            DornaHome, DornaSet, DornaPlay,
                            DornaPause, DornaHalt, DornaCalibrate,
                            DornaJog, DornaXYZCheck, DornaXYZtoJoint,
                            DornaMoveJoints, DornaMoveXYZ, DornaOutputPin]
        self._srv_prx_list = {}
        for topic, msg_type in zip(robot_srv_topics, robot_srv_msg_types):
            name_space = self._ns+'/robot_cmd/'+topic
            # rospy.wait_for_service(name_space)
            self._srv_prx_list[topic] = rospy.ServiceProxy(name_space, msg_type)

        moveit_srv_topics = ['view','move_home',
                            'move_extend', 'move_heart',
                            'move_rand', 'move_button',
                            'move_straight']
        moveit_srv_msg_types = [DornaView, DornaPlanCmd, 
                                DornaPlanCmd, DornaPlanCmd,
                                DornaPlanCmd, DornaXYZ, 
                                DornaPlanCmd]
        for topic, msg_type in zip(moveit_srv_topics, moveit_srv_msg_types):
            name_space = self._ns+'/moveit_cmd/'+topic
            # rospy.wait_for_service(name_space)
            self._srv_prx_list[topic] = rospy.ServiceProxy(name_space, msg_type)

        # rospy.wait_for_service('pix_2_world')
        self._srv_prx_list['pix_2_world'] = rospy.ServiceProxy('pix_2_world', Pix2World)
        self._srv_prx_list['tf_transform'] = rospy.ServiceProxy('get_tf_transform', TfTransform)
        ##############################################
        ####           Human Subscribers         #####
        ##############################################
        self._sub_topics = ['cartesian_position', 'joint_angles']
        pub_msg_types = [DornaPos, DornaJoint]
        self._subscriber_list = {}
        for topic, msg_type in zip(self._sub_topics, pub_msg_types):
            callback = getattr(self, topic+"_callback", None)
            self._subscriber_list[topic] = rospy.Subscriber(self._ns+'/robot_info/'+topic, msg_type, callback)
        
        callback = getattr(self, "sent_pic_callback", None)
        self._subscriber_list['sent_pic'] = rospy.Subscriber('/realsense/info/sent_pic', Empty, callback)
        callback = getattr(self, "buttons_world_callback", None)
        self._subscriber_list['buttons_world'] = rospy.Subscriber('buttons_world', ButtonsWorld, callback)
        # self._package_subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.package_callback)

        ##############################################
        ####           Human Publishers          #####
        ##############################################
        self._take_pic_pub = rospy.Publisher('/realsense/cmd/take_pic', Empty, queue_size=10)
        

    def init(self):
        self.connect()
        if self.connection_status:
            self.home()
            self.calibrate()
            try:
                response = input("Move to Home Pose enter 1:    ")
                if response == '1':
                    self.home_pose()
                else:
                    pass
            except rospy.ROSException:
                pass
        else:
            self.init()
        
    def connect(self):
        rospy.wait_for_service('/dorna_real/robot_cmd/connect')
        response = input("Press Enter to Connect to Robot:       ")
        if response == '':
            usb = "/dev/ttyACM0"
            response = self._srv_prx_list['connect'](usb)
            rospy.loginfo(response)
            if not response.connected:
                rospy.logerr('Error [Human_Ros]: Did not get proper response when connecting to robot')

            self.connection_status = response.connected
        else:
            self.connection_status = False

    def home(self):
        response = input("NOTE: Please check robot surroundings before homing!\n\rTo home the robot motors input 1, else the robot will set the motors to the home position\n\r\n\rHere:    ")
        if response == "1":
            srv_response = self._srv_prx_list['home']()
        else:
            srv_response = self._srv_prx_list['set_joint']()

        if srv_response.homed:
            return
        else:
            rospy.logerr('Error [Human_Ros]: Did not get proper response when homing to robot')

    def calibrate(self):
        response = input("Do you want to calibrate the motors? Default in NO, Enter y to override\n\rHere:      ")
        if response == 'y':
            for i in range(len(self._joint_names)):
                jog_response = input("Do you want to jog joint{}? y/n       ".format(i))
                if jog_response == 'n':
                    pass
                elif jog_response == 'y' or jog_response == '':
                    self.jog_motor(i)

            # response = self._srv_prx_list['calibrate']()
        else:
            pass
    
    def home_pose(self):
        self.service_check('/dorna_real/robot_cmd/move_joints')
        response = self._srv_prx_list['move_joints']("joint", DornaMovement.GLOBAL.value, DornaMovement.DEFAULT_JOINT_SPEED.value, [0, 145, -90, 0, 0])
        if response.response:
            rospy.loginfo("Arrived at home location")
        else:
            rospy.logwarn("Did not get good response")
        # trajectory = response.trajectory.joint_trajectory
        # print(trajectory)
        # response = self._srv_prx_list['play'](trajectory)

    def tuck_away(self):
        self.service_check('/dorna_real/robot_cmd/move_joints')
        response = self._srv_prx_list['move_joints']('joint', DornaMovement.GLOBAL.value, DornaMovement.DEFAULT_XYZ_SPEED.value, [-90., 170., 0., 0., 0.])
        if response.response:
            rospy.loginfo("Arrived at tuck_away location")
        else:
            rospy.logwarn("Did not get good response")

    def move_forward_test(self):
        self.service_check('/dorna_real/robot_cmd/move_xyzab')
        ## TODO:Fix this function
        speed = int(DornaMovement.DEFAULT_XYZ_SPEED.value/2)
        response = self._srv_prx_list['move_xyzab']("line", DornaMovement.RELATIVE.value, speed, [30, 0, 0, 0, 0])
        if response.response:
            rospy.loginfo("Arrived at desired location")
        else:
            rospy.logwarn("Did not get good response")

    def move_xyz_test(self):
        self.service_check('/dorna_real/robot_cmd/move_xyzab')
        response = self._srv_prx_list['move_xyzab']("line", DornaMovement.GLOBAL.value, DornaMovement.DEFAULT_XYZ_SPEED.value, [300, 0, 500, 0, 0])
        if response.response:
            rospy.loginfo("Arrived at tuck_away location")
        else:
            rospy.logwarn("Did not get good response")


    #######################################
    ####       Button Commands         ####
    #######################################
    def get_button_dict(self, response):
        availible_buttons = []
        for i in range(len(response.msg)):
            availible_buttons.append("BUTTON_{}".format(response.msg[i].id))
            # availible_buttons[response[i].id] = self.find_transform("dorna_base", child_frame)

        return availible_buttons

    def show_availible_buttons(self, availible_buttons):
        rospy.loginfo("These are the buttons we can see:    ")
        for button in availible_buttons:
            rospy.loginfo(button)

    def button_input(self, availible_buttons):
        self.show_availible_buttons(availible_buttons)
        button_id = input("Which button do you want to move to?\n\rEnter Here:       ")
        try:
            if button_id in availible_buttons:
                if self.can_reach_button(button_id):
                    return button_id
                else:
                    rospy.logerr("Robot cant reach that button safely")
                    self.button_input(availible_buttons)
            else: 
                rospy.logerr("[Error]: Response was not found in availible buttons. Try again.")
                self.button_input(availible_buttons)
        except KeyboardInterrupt:
            raise
    
    def get_xyz(self, button_id):
        child_frame = button_id
        parent_frame = 'dorna_base'
        self.service_check('get_tf_transform')
        xyz_transform_meters = self._srv_prx_list['tf_transform']([child_frame, parent_frame])
        xyz_transform_mm = []
        for val in xyz_transform_meters.transform:
            xyz_transform_mm.append(val*1000.)
        
        # xyz_transform_mm = [300, 0, 360]
        return xyz_transform_mm
    
    def can_reach_button(self, button_id):
        xyz = self.get_xyz(button_id)
        
        print(xyz)
        response = self._srv_prx_list['xyz_check'](xyz)
        print(response.can_reach)
        if response.can_reach: 
            rospy.loginfo("Can reach button location")
            return True
        else:
            rospy.logerr("Cannot reach button location")
            return False

    def pick_button(self, response):
        availible_buttons = self.get_button_dict(response)
        button_id = self.button_input(availible_buttons)
        xyz = self.get_xyz(button_id)

        return xyz

    def xyz_to_joint(self, xyz, x_offset = None):
        if x_offset:
            xyz[0] -= x_offset
        response = self._srv_prx_list['xyz_to_joint'](xyz)
        return response.joint_angles

    #######################################
    ####          Jog Commands         ####
    #######################################
    def jog_cmd(self, axis, axis_value, step_size, max_step, min_step, units):
        rospy.loginfo("Axis {} current value [{}]:  {}".format(axis, units, axis_value))
        rospy.loginfo("Current jog step size is {} [{}]".format(step_size, units))
        command = input("Enter w to jog up\n\rEnter s to jog down\n\rEnter e to change step size\n\rEnter y when ok\n\rHere:    ")
        if command == 'w':
            response = self._srv_prx_list['jog']("joint", axis, step_size)
            return True
        elif command == 's':
            response = self._srv_prx_list['jog']("joint", axis, -step_size)
            return True
        elif command == 'e':
            step_size = int(input("Enter value for step size in {} (Max is {}, Min is {}):        ".format(units, max_step, min_step)))
            step_size = min(max(step_size, min_step), max_step)
            self.jog_cmd(axis, axis_value, step_size, max_step, min_step, units)
        elif command == 'y':
            return False
        else:
            rospy.logerr('Invalid Entry')
            pass
        return True

    def jog_axis(self, axis):
        not_ok = True 
        units = rospy.get_param('/dorna_real/units')           
        index = self._axis_names.index(axis)
        while not_ok:
            axis_value = self.robot_xyz_pos[index]
            if axis == "a" or axis == "b":
                step_size = 5
                not_ok = self.jog_cmd(axis, axis_value, step_size, 10, 1, "degrees")
            elif units == "mm":      
                step_size = 25
                not_ok = self.jog_cmd(axis, axis_value, step_size, 50, 10, "millimeters")
            elif units == "inch":
                step_size = 1
                not_ok = self.jog_cmd(axis, axis_value, step_size, 2, 0.4, "inches")

    def jog_motor(self, motor): 
        not_ok = True
        step_size = 5 # [deg]
        name = self._joint_names[motor]
        while not_ok:
            joint_value = self.robot_joint_angles[motor]
            not_ok = self.jog_cmd(name, joint_value, step_size, 10, 1, "degrees")

    def jog_input(self):
        jogging = True
        while jogging:
            mode = int(input("To move each motor enter 0\n\rTo move end effector enter 1\n\rOr 2 to quit\n\rHere:       "))
            if mode == 1: 
                axis = input("Enter Axis [x, y, z, pitch, roll] or q to quit jogging:   ")
                if axis not in self._axis_names and axis != "q":
                    rospy.logerr("Invalid Entry")
                    pass
                elif axis == "q":
                    jogging = False
                else:
                    self.jog_axis(axis)
            elif mode == 0:
                motor = input("Enter Motor number (0-4) or q to quit jogging:      ")
                if motor not in ['0','1','2','3','4','q']:
                    rospy.logerr("Invalid Entry")
                    pass
                elif motor == 'q':
                    jogging = False
                else:
                    self.jog_motor(int(motor))
            else:
                jogging = False

    ###########################################
    ####        Robot Info Callback        ####
    ###########################################
    def cartesian_position_callback(self, pos):
        self.robot_xyz_pos = [pos.x, pos.y, pos.z, pos.a, pos.b]

    def joint_angles_callback(self, joints):
        self.robot_joint_angles = [joints.j0, joints.j1, joints.j2, joints.j3, joints.j4]

    ###########################################
    ####              Thread               ####
    ###########################################
    def service_check(self, name):
        try:
            rospy.wait_for_service(name, timeout=5.0)
        except rospy.ROSException:
            rospy.logerr('{} service is not responding'.format(name))
            self.run()


    def run(self):
        try:
            while not rospy.is_shutdown():
                if self.trajectory is not None:
                    play = input("Press s to view path again in Rvis\n\rPress p to play current trajectory\n\r Press e to erase current trajectory\n\r Here:  ")
                    if play == 'p':
                        response = self._srv_prx_list['play'](self.trajectory)
                    elif play == 'e':
                        self.trajectory = None
                    elif play == 's':
                        response = self._srv_prx_list['view']()
                    else:
                        rospy.logerr("You did not pick one of the options try again.")
                        pass
                elif self._button_data is not None:
                    button_xyz = self.pick_button(self._button_data)  #Returns selected button xyz in robot frame (only returns if robot can reach)
                    joint_angles = self.xyz_to_joint(button_xyz, x_offset=30.0) #Returns joint_angles at end position with 10 mm offset

                    # name = 'move_button'
                    # self.service_check(self._ns+'/moveit_cmd/'+name)
                    # response = self._srv_prx_list[name](list(joint_angles)) #Returns trajectory for button press
                    # self.trajectory = response.trajectory.joint_trajectory
                    # rospy.loginfo("Sending trajectory to Robot")
                    # response = self._srv_prx_list['play'](self.trajectory)

                    self.service_check('/dorna_real/robot_cmd/move_joints')
                    response = self._srv_prx_list['move_joints']('joint', DornaMovement.GLOBAL.value, DornaMovement.DEFAULT_XYZ_SPEED.value, list(joint_angles))
                    if response.response:
                        rospy.loginfo("Arrived at button location")
                    else:
                        rospy.logwarn("Did not get good response")

                    response = input("Does robot hand seem to be aligned correctly?     ")
                    if response == '' or response == 'y':
                        self.move_forward_test()
                    else:
                        rospy.logerr("Hand does not seem to be aligned correctly.")
                        response = input("Would you like to jog the robot into the correct place?   ")
                        if response == '' or response == 'y':
                            self.jog_input()


                        
                        response = input("Would you like to return robot to home position?      ")
                        if response == '' or response == 'y':
                            self.home_pose()

                    response = input("Was demo successful, ready to quit and disconnect?        ")
                    if response == '' or response == 'y':
                        self.home_pose()
                        rospy.loginfo("CONGRATS")
                    elif response == 'n':
                        self.button_demo()

                else:
                    path = input("[Note] These are commands for Moveit Node make sure dorna_real_moveit.py has started.\n\rEnter h to move to Home position\n\rEnter e to move to Extended position\n\rEnter d to draw heart\n\rEnter r to move to random valid position\n\rEnter s to move in a straight line\n\rEnter b for buttons \n\rEnter j to jog joints\n\rEnter q to quit\n\rHere:      ")
                    if path == "h":
                        name = "move_home"
                        self.service_check(self._ns+'/moveit_cmd/'+name)
                        response = self._srv_prx_list[name]()
                        self.trajectory = response.trajectory.joint_trajectory
                    elif path == "e":
                        name = 'move_extend'
                        self.service_check(self._ns+'/moveit_cmd/'+name)
                        response = self._srv_prx_list[name]()
                        self.trajectory = response.trajectory.joint_trajectory
                    elif path == "d":
                        name = 'move_heart'
                        self.service_check(self._ns+'/moveit_cmd/'+name)
                        response = self._srv_prx_list[name]()
                        self.trajectory = response.trajectory.joint_trajectory
                    elif path == "r":
                        name = 'move_rand'
                        self.service_check(self._ns+'/moveit_cmd/'+name)
                        response = self._srv_prx_list[name]()
                        self.trajectory = response.trajectory.joint_trajectory
                    elif path == "b":
                        self.button_demo()
                    elif path == 'j':
                        self.jog_input()
                    elif path == 't':
                        self.tuck_away()
                    elif path == 'f':
                        self.move_forward_test()
                    elif path == 'x':
                        self.move_xyz_test()
                    elif path == 'p':
                        self.pick_and_place(300, 0, -250, 0)
                    elif path == 'q':
                        rospy.loginfo("Quiting Program")
                        if self.connection_status:
                            response = self._srv_prx_list['quit']()
                        rospy.signal_shutdown('Quit')
        except KeyboardInterrupt:
            pass

    def sent_pic_callback(self, data):
        rospy.loginfo("Picture was sent to nn.")
        self.home_pose()

    def buttons_world_callback(self, data):
        self._button_data = data
        

    def button_demo(self):
        self.tuck_away()
        time.sleep(5) ##TODO: Fix this to be dependant on robot state
        # name = 'pix_2_world'
        # self.service_check(name)
        # response = self._srv_prx_list[name]() #Starts button demo response is dictionary of buttons and xyz in camera frame
        self._take_pic_pub.publish(Empty())
        rospy.loginfo("Take_picture was requested.")

        # self.home_pose()
    def pick_and_place(self, xi, yi, xf, yf):
        tool_head = 0
        package_height = 150
        safety = 50
        above = package_height+tool_head+safety
        child_frame = 'tag_2'
        parent_frame = 'dorna_base'
        pin = 2
        response = self._srv_prx_list['digital_out'](pin, False)
        response = self._srv_prx_list['move_xyzab']("joint", DornaMovement.GLOBAL.value, DornaMovement.DEFAULT_XYZ_SPEED.value, [xi,yi, 300, -90, 0])
        time.sleep(2)
        response = self._srv_prx_list['tf_transform']([child_frame, parent_frame])
        location = response.transform
        x = location[0]*1000.
        y = location[1]*1000.
        z = round(location[2]*1000., 1)
        response = self._srv_prx_list['move_xyzab']("joint", DornaMovement.GLOBAL.value, DornaMovement.DEFAULT_XYZ_SPEED.value, [x+30, y, safety, -90, 0])
        response = self._srv_prx_list['digital_out'](pin, True)
        response = self._srv_prx_list['move_xyzab']("line", DornaMovement.RELATIVE.value, DornaMovement.DEFAULT_XYZ_SPEED.value, [0, 0, -60, 0, 0])
        response = self._srv_prx_list['move_xyzab']("line", DornaMovement.RELATIVE.value, DornaMovement.DEFAULT_XYZ_SPEED.value, [0,0, 100, 0, 0])
        response = self._srv_prx_list['move_xyzab']("joint", DornaMovement.GLOBAL.value, DornaMovement.DEFAULT_XYZ_SPEED.value, [0, -300, 100, -90, 0])
        response = self._srv_prx_list['move_xyzab']("joint", DornaMovement.RELATIVE.value, DornaMovement.DEFAULT_XYZ_SPEED.value, [xf, 300, 0, 0, 0])
        response = self._srv_prx_list['move_xyzab']("line", DornaMovement.RELATIVE.value, DornaMovement.DEFAULT_XYZ_SPEED.value, [0, 0, -100, 0, 0])
        response = self._srv_prx_list['digital_out'](pin, False)
        time.sleep(2)
        response = self._srv_prx_list['move_xyzab']("line", DornaMovement.RELATIVE.value, DornaMovement.DEFAULT_XYZ_SPEED.value, [0, 0, safety, 0, 0])
        response = self.home_pose()        
 

                            



if __name__=="__main__":
    rospy.init_node('human_ros')
    rospy.loginfo('Starting node "human_ros"')
    human = HumanRos()
    try:
        human.init()
    except (rospy.service.ServiceException, KeyboardInterrupt):
        rospy.logerr('Check if the robot node has started')
        raise        
    try:
        runner = Thread(target=human.run)
        runner.start()
        rospy.spin()
        runner.join()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        raise
    # finally:
    #     human.quit
    
