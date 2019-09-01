#!/usr/bin/env python2

import copy
from enum import IntEnum
import numpy as np
import pickle
import sys
from threading import Thread
import time

import rosnode
import rospy
import tf
import tf2_ros

import moveit_commander
from moveit_python import PlanningSceneInterface

from apriltags2_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import RobotState, RobotTrajectory, DisplayTrajectory, CollisionObject
from pix_2_world.msg import ButtonsPix, ButtonsWorld, Pix, World
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Int32, String, Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from dorna_api.srv import DornaView, DornaXYZ, DornaPlanCmd, HumanOpCmd
from pix_2_world.srv import Pix2World, NNService

# sys.path.append('/home/rit/catkin_ws/src/dorna_robot')  #### bad
import DornaUtils


class HumanOperator(IntEnum):
    MOVE_HOME           = 4
    MOVE_EXTEND         = 5
    MOVE_HEART          = 6
    MOVE_RANDOM         = 7
    MOVE_STRAIGHT       = 8
    MOVE_BUTTON         = 9
    GO                  = 10
    
    @classmethod
    def has_value(cls, value):
        return any(value == item.value for item in cls)

class MoveitPathService():
    def __init__(self, loop_rate=10, pose_target = None, debug=False, gazebo=False):
        self._listener = tf.TransformListener()
        self.trajectory = None
        ns = '/dorna_real'
        self._gazebo = gazebo
        if gazebo: 
            ns = '/dorna'
        #############################################################
        ################# Useful Constants ##########################
        #############################################################
        self._n_joints = 5
        self._joint_names = ["dorna_base_to_dorna_arm_0", 
                            "dorna_arm_0_to_dorna_arm_1", 
                            "dorna_arm_1_to_dorna_arm_2", 
                            "dorna_arm_2_to_dorna_arm_3", 
                            "dorna_arm_3_to_dorna_arm_4"]

        ###############################################################
        ################# MoveGroupCommander ##########################
        ###############################################################
        # self.has_move_group_launched() 
        # time.sleep(20)
        self._group = moveit_commander.MoveGroupCommander("manipulator")
        self._group.set_planner_id("SBL")
        self._group.set_planning_time(10.0)
        self._group.set_goal_tolerance(0.1)

        self._planners = ['SBL', 'EST','LBKPIECE','BKPIECE','KPIECE',
                        'RRT','RRTConnect','RRTstar','TRRT','PRM',
                        'PRMstar','FMT','BFMT','PDST','STRIDE','BiTRRT',
                        'LBTRRT','BiEST','ProjEST','LazyPRM','LazyPRMstar',
                        'SPARS','SPARStwo']

        self._target_names = ["allZeros", "home", "doorPose", "ElbowDown"]
        ###################################################################
        ################# PlanningSceneInterface ##########################
        ###################################################################
        self._scene = PlanningSceneInterface(frame="dorna_base")
        self.create_scene()

        ######################################################
        ################# Ros Setup ##########################
        ######################################################
        self._rate = rospy.Rate(loop_rate)

        ######################################################
        ################# Publishers #########################
        ######################################################
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size = 10)
        
        ######################################################
        #################  Services  #########################
        ######################################################
        srv_topics = ['view','move_home', 'move_extend', 'move_heart',
                            'move_rand', 'move_button', 'move_straight']
        srv_msg_types = [DornaView, DornaPlanCmd, DornaPlanCmd, DornaPlanCmd,
                         DornaPlanCmd, DornaXYZ, DornaPlanCmd]
        self._srv_list = {}
        for topic, msg_type in zip(srv_topics, srv_msg_types):
            callback = getattr(self, topic+"_srv_callback", None)
            self._srv_list[topic] = rospy.Service(ns+'/moveit_cmd/'+topic, msg_type, callback)

        self._plan_service = rospy.Service(ns+'/human_op/cmd_input', HumanOpCmd, self.input_callback)
        self._view_service = rospy.Service(ns+'/human_op/view_path', DornaView, self.view_callback)
        ######################################################
        ################# Subscriber #########################
        ######################################################
        self._button_subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.button_callback)

        self.avalible_buttons = {}
        if debug:
            rospy.loginfo("here")
            self.debug_button()

    ####################################################################
    ################### Gazebo Human Interface #########################
    ####################################################################
    def run_human_interface(self):
		try:
			while not rospy.is_shutdown():
				self._move = str(raw_input('Enter r for random valid position\n\rEnter d for open door beahvior\n\rEnter h to draw heart\n\rHere:		'))
				if self._move == "r":
					pass
					# self.random_pose()
				elif self._move == "d":
					self.open_door()
				elif self._move == "h":
					# self.draw_heart()
					pass
				else:
					rospy.loginfo('Invalid Input')
					pass
		except KeyboardInterrupt:
			pass
    
    def run_testing_methods(self):
        try:
            rospy.loginfo("GO_JOINT_VALUE_TARGET")
            time.sleep(2.0)
            self.go_joint_value_target([np.pi/2., 0, 0, 0, 0])
            time.sleep(2.0)
            rospy.loginfo("GO_SHIFT_POSE_TARGET")
            self.go_shift_pose_target(3, np.pi/2.)
            time.sleep(2.0)

            # rospy.loginfo("TESTING ORIENTATION_TARGET: roll by 90 deg")
            # des_pose = self._group.get_current_pose()
            # des_pose.pose.orientation.x = np.sqrt(0.5)
            # des_pose.pose.orientation.w = np.sqrt(0.5)
            response = str(raw_input("Did test work? 	"))
            # self.go_pose_target(des_pose)
            if response == '':
                pass
            else:
                raise KeyboardInterrupt
                
            # self.go_home()
            # rospy.loginfo("TESTING ORIENTATION_TARGET: roll by 90 deg")
            # self.go_orientation_target([np.sqrt(0.5), 0, 0, np.sqrt(0.5)])
            # rospy.loginfo("TESTING ORIENTATION_TARGET: pitch by 90 deg")
            # self.go_orientation_target([0, np.sqrt(0.5), 0, np.sqrt(0.5)])
            # response = str(raw_input("Did ORIENTATION_TARGET test work? 	"))
            # if response == '':
            # 	pass
            # else:
            # 	raise KeyboardInterrupt

        except KeyboardInterrupt:
            raise

    ####################################################################
    ####################### Helper Methods #############################
    ####################################################################
    def create_scene(self):
        self._scene.addBox(name= "cart", size_x = 0.5, size_y = 0.3, size_z = 0.7,  x = -0.175, y = 0.0, z = -0.362)
        self._scene.setColor(name = "cart", r = 255, g = 0, b = 0)
        self._scene.addBox(name= "wood_base", size_x = 0.15, size_y = 0.235, size_z = 0.060, x = 0.0, y = 0.0, z = -0.03)
        self._scene.setColor(name = "wood_base", r = 0, g = 255, b = 0)
        self._scene.addBox(name = "right_wall", size_x = 0.85, size_y = 0.1, size_z = 2.0, x = -0.15, y = -0.22, z = 0.275)
        self._scene.setColor(name = "right_wall", r = 0, g = 0, b = 0, a = 0.5)
        self._scene.addBox(name = "back_wall", size_x = 0.1, size_y = 1.4, size_z = 2.0, x = -0.600, y = .53, z = 0.275)
        self._scene.setColor(name = "back_wall", r = 0, g = 0, b = 0, a = 0.9)
        # self._scene.addBox(name = "front_wall", size_x = 0.1, size_y = 0.6, size_z = 2.0, x = 0.300, y = 0.13, z = 0.275)
        # self._scene.setColor(name = "front_wall", r = 0, g = 0, b = 0, a = 0.9)
        self._scene.addBox(name = "front_wall2", size_x = 0.1, size_y = 0.2, size_z = 2.0, x = 0.300, y = 1.1, z = 0.275)
        self._scene.setColor(name = "front_wall2", r = 0, g = 0, b = 0, a = 0.9)
        self._scene.addBox(name = "left_wall", size_x = 0.85, size_y = 0.1, size_z = 2.0, x = -0.15, y = 1.28, z = 0.275)
        self._scene.setColor(name = "left_wall", r = 0, g = 0, b = 0, a = 0.5)
        self._scene.sendColors()
        # self._scene.addBox(name="table", size_x=1.0, size_y=1., size_z=0.1, x=-0.45, y=0., z=-0.05)
        # self._scene.addBox(name="wall", size_x=0.1, size_y=1., size_z=1., x=0.4, y=0., z=0.5)

    def debug_button(self):
        for i in range(2):
            tag_name = 'tag_' + str(i)
            trans = self.find_transform('dorna_base', tag_name)
            if trans:
                self.avalible_buttons[tag_name] = trans

    def find_transform(self, parent_frame, child_frame):
        now = rospy.Time(0)

        # Wait for tf transform
        self._listener.waitForTransform(parent_frame, child_frame,  now, rospy.Duration(1.))
        if self._listener.canTransform(parent_frame, child_frame, now):
            (trans,rot) = self._listener.lookupTransform(parent_frame, child_frame, now)
        
            return trans
        return None

    def has_move_group_launched(self):
        available_nodes = rosnode.get_node_names()
        if '/move_group' in available_nodes:
            rospy.loginfo("/move_group has launched")
            pass
        else:
            rospy.loginfo("waiting for /move_group")
            time.sleep(1)
            self.has_move_group_launched()

    def publish_traj(self):
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory.append(self.trajectory)
        self._display_trajectory_publisher.publish(display_trajectory)

    ##########################################################################
    ####################### Subscriber Callbacks #############################
    ##########################################################################
    def button_callback(self, data):
        self.avalible_buttons = {}
        for tag in data.detections:
            tag_name = 'tag_' + str(tag.id[0])
            trans = self.find_transform('dorna_base', tag_name)
            if trans:
                self.avalible_buttons[tag_name] = trans


    def input_callback(self, data):
        path = data.data
        if HumanOperator.has_value(path): 
            if path == HumanOperator.MOVE_HOME:
                self.home()
                return self.trajectory
            elif path == HumanOperator.MOVE_EXTEND:
                self.extend()
                return self.trajectory
            elif path == HumanOperator.MOVE_RANDOM:
                self.random_pose()
                return self.trajectory
            elif path == HumanOperator.MOVE_STRAIGHT:
                self.straight()
                return self.trajectory
            # elif path == HumanOperator.MOVE_BUTTON:
            #     self.push_button(buttons)
            #     return self.trajectory
            elif path == HumanOperator.MOVE_HEART:
                self.draw_heart()
                return self.trajectory
        else:
            rospy.loginfo("Not valid input.")
            pass

    def view_callback(self, data):
        self.publish_traj()

    #######################################################################
    ####################### Service Callbacks #############################
    #######################################################################
    def view_srv_callback(self, req):
        rospy.loginfo("[Service Callback] View")
        self.publish_traj()
        return True
    
    def move_home_srv_callback(self, req):
        rospy.loginfo("[Service Callback] MoveHome")
        self.home()
        return self.trajectory

    def move_extend_srv_callback(self, req):
        rospy.loginfo("[Service Callback] MoveExtend")
        self.extend()
        return self.trajectory

    def move_heart_srv_callback(self, req):
        rospy.loginfo("[Service Callback] MoveHeart")
        self.draw_heart()
        return self.trajectory

    def move_rand_srv_callback(self, req):
        rospy.loginfo("[Service Callback] MoveRand")
        self.random_pose()
        return self.trajectory

    def move_button_srv_callback(self,req):
        rospy.loginfo("[Service Callback] MoveButton")
        # button_location = self.find_transform("world", req.child_frame)
        print(req)
        self.push_button(req)
        return self.trajectory

    def move_straight_srv_callback(self, req):
        rospy.loginfo("[Service Callback] MoveStraight")
        self.straight()
        return self.trajectory

    def button_demo_srv_callback(self, req):
        rospy.loginfo("TODO")

    ###############################################################
    ################# Planning Actions ############################
    ###############################################################
    def draw_heart(self):
        thetas = pickle.load(open('/home/rit/catkin_ws/src/dorna_robot/dorna_nodes/src/draw_heart_thetas.p','rb')) ##@TODO Bad need to fix 
        traj = JointTrajectory()
        traj.joint_names = self._joint_names
        for theta in thetas:
            point = JointTrajectoryPoint()
            # improve this
            if theta[0] > np.pi:
                theta[0] -= 2*np.pi
            if theta[1] > np.pi:
                theta[1] -= 2*np.pi
            if theta[2] > np.pi:
                theta[2]-= np.pi
            point.positions = theta
            traj.points.append(point)
        self.trajectory = RobotTrajectory()
        self.trajectory.joint_trajectory = traj

        self.publish_traj()

    def extend(self):
        traj = JointState()
        traj.name = self._joint_names
        traj.position = [0, 0, 0, 0, 0]
        self.trajectory = self._group.plan(joints=traj)
        self.publish_traj()

    def home(self):
        traj = JointState()
        traj.name = self._joint_names
        traj.position = [0, 2.5, -1.57, 0, 0]
        self.trajectory = self._group.plan(joints=traj)
        self.publish_traj()

    def push_button(self, joint_angles_deg):  # this gets the button to push
        joint_angles_rad = []
        for joint in joint_angles_deg.joint_angles:
            joint_angles_rad.append(joint*np.pi/180.)
        self.go_joint_value_target(joint_angles_rad)
        self.publish_traj()

        # offset_x = 0.05 #1
        # offset_y = 0. #015
        # offset_z = 0.

        # waypoints = []

        # # start with the current pose
        # pose_0 = self._group.get_current_pose().pose
        # waypoints.append(pose_0)

        # # move just short of button
        # pose_1 = Pose()
        # pose_1.position.x = -button_location[0]+offset_x
        # pose_1.position.y = button_location[1]
        # pose_1.position.z = button_location[2]
        # pose_1.orientation.y = 1
        # waypoints.append(pose_1)

        # # move forward to button
        # pose_2 = Pose()
        # pose_2.position.x = -button_location[0]
        # pose_2.position.y = button_location[1]
        # pose_2.position.z = button_location[2]
        # pose_2.orientation.y = 1
        # waypoints.append(pose_2)

        # (self.trajectory, fraction) = self._group.compute_cartesian_path(
        #                     waypoints,   # waypoints to follow
        #                     0.01,        # eef_step
        #                     0.0)         # jump_threshold
        
        # self.publish_traj()

    def random_pose(self):
        goal = self._group.get_random_joint_values()
        self.trajectory = self._group.plan(joints=goal)
        self.publish_traj()

    def straight(self, dir="x", dist="0.05"):
        waypoints = []

        # start with the current pose
        waypoints.append(self._group.get_current_pose().pose)

        # first orient gripper
        wpose = Pose()
        wpose.orientation.y = 1.0
        if dir == 'x':
            wpose.position.x = waypoints[0].position.x - dist
        elif dir == 'y':
            wpose.position.y = waypoints[0].position.y - dist
        elif dir == 'z':
            wpose.position.z = waypoints[0].position.z + dist

        waypoints.append(copy.deepcopy(wpose))

        (self.trajectory, fraction) = self._group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold
        self.publish_traj()

    def trajectories(self, joints):
        trajectory = []
        for joint in joints:
            traj = JointState()
            traj.name = self._joint_names
            traj.position = [0, 2.5, -1.57, 0, 0]
            trajectory.append(self._group.plan(joints = traj))

        
    #################################
    #### Set movegroup_commander ####
    #################################
    def plan(self):
        self.trajectory = self._group.plan()
        rospy.loginfo(self.trajectory)

    def go(self):
        if self._gazebo:
            self._group.go(wait=True)

    def go_home(self):
        self.go_named_target('home')

    def go_joint_value_target(self, arg1, arg2 = None, arg3 = None):
        ##Works##
        """
        Specify a target joint configuration for the group.
        - if the type of arg1 is one of the following: dict, list, JointState message, then no other arguments should be provided.
        The dict should specify pairs of joint variable names and their target values, the list should specify all the variable values
        for the group. The JointState message specifies the positions of some single-dof joints.
        - if the type of arg1 is string, then arg2 is expected to be defined and be either a real value or a list of real values. This is
        interpreted as setting a particular joint to a particular value.
        - if the type of arg1 is Pose or PoseStamped, both arg2 and arg3 could be defined. If arg2 or arg3 are defined, their types must
        be either string or bool. The string type argument is interpreted as the end-effector the pose is specified for (default is to use
        the default end-effector), and the bool is used to decide whether the pose specified is approximate (default is false). This situation
        allows setting the joint target of the group by calling IK. This does not send a pose to the planner and the planner will do no IK.
        Instead, one IK solution will be computed first, and that will be sent to the planner.
        """

        self._group.set_joint_value_target(arg1, arg2, arg3)
        self.plan()
        self.go()


    def go_rpy_target(self, rpy):
        ##Doesnt Works##
        """rpy should be a list"""
        try:
            if type(rpy) == list and len(rpy) == 3:
                self._group.set_rpy_target(rpy)
                self.plan()
                self.go()
            else:
                raise TypeError
        except TypeError:
            raise

    def go_orientation_target(self, q):
        ##Doesnt Works##
        """q should be a list"""
        try:
            if type(q) == list and len(q) == 4:
                self._group.set_orientation_target(q)
                self.plan()
                self.go()
            else:
                raise TypeError
        except TypeError:
            raise

    def go_position_target(self, xyz):
        ##Doesnt Works##
        """xyz should be a list"""
        try:
            if type(xyz) == list and len(xyz) == 3:
                self._group.set_position_target(xyz)
                self.plan()
                self.go()
            else:
                raise TypeError
        except TypeError:
            raise

    def go_pose_target(self, pose):
        """ Set the pose of the end-effector, if one is available. The expected input is a Pose message, a PoseStamped message or a list of 6 floats:"""
        """ [x, y, z, rot_x, rot_y, rot_z] or a list of 7 floats [x, y, z, qx, qy, qz, qw] """
        try:
            if type(pose) == Pose or type(pose) == list or type(pose) == PoseStamped:
                self._group.set_pose_target(pose)
                self.plan()
                self.go()
            else:
                raise TypeError
        except TypeError:
            raise

    def go_pose_targets(self, poses):
        try:
            if type(poses) == list and type(type(poses[0])) == Pose:
                self._group.set_pose_targets(poses)
                self.plan()
                self.go()
            else:
                raise TypeError
        except TypeError:
            raise

    def go_shift_pose_target(self, axis, value):
        ## Did not work ##
        """ Get the current pose of the end effector, add value to the corresponding axis (0..5: X, Y, Z, R, P, Y) and set the new pose as the pose target """
        try:
            if type(axis) == int:
                self._group.shift_pose_target(axis, value)
                self.plan()
                self.go()
            else:
                raise TypeError
        except TypeError:
            raise

    def go_random_target(self):
        self._group.set_random_target()
        self.plan()
        self.go()

    def go_named_target(self, name):
        ##Works##
        rospy.loginfo("Called go {}".format(name))
        if name in self._target_names:
            self._group.set_named_target(name)
            self.plan()
            self.go()

if __name__=="__main__":
    rospy.init_node('moveit_path_service', anonymous = True)
    rospy.loginfo('Starting node "moveit_path_service"')
    moveit_commander.roscpp_initialize(sys.argv)
    gazebo = False
    MPS = MoveitPathService(gazebo=gazebo)
    if gazebo: 
        runner = Thread(target= MPS.run_testing_methods)
        runner.start()
        rospy.spin()
        runner.join
    else:
        rospy.spin()
