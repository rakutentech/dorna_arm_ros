#!/usr/bin/env python2

import sys
from threading import Thread

import rospy
import tf2_ros

import moveit_commander
from moveit_python import PlanningSceneInterface

from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import RobotState, RobotTrajectory, DisplayTrajectory, CollisionObject
from pix_2_world.msg import ButtonsPix, ButtonsWorld, Pix, World
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Int32, String, Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from dorna_api.srv import DornaView, DornaXYZ, DornaPlanCmd, HumanOpCmd, DornaPlay, DornaPlan, DornaExecute
from pix_2_world.srv import Pix2World, NNService

class MoveitPathService():
    def __init__(self, gazebo=False):
        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Time(1))
        self._listener = tf2_ros.TransformListener(self._tf_buffer)
        self.trajectory = None
        ns = '/dorna_real'
        self._gazebo = gazebo
        if gazebo:
            ns = '/dorna'
        #############################################################
        ################# Useful Constants ##########################
        #############################################################
        joint_param = '/move_group/hardware_interface/joints'
        if rospy.has_param(joint_param):
            self._joint_names = rospy.get_param(joint_param)
            self._n_joints = len(self._joint_names)
        else:
            rospy.logwarn('No joints found.')

        ###############################################################
        ################# MoveGroupCommander ##########################
        ###############################################################
        self._group = moveit_commander.MoveGroupCommander("dorna")
        # self._group.set_planner_id("SBL")
        # self._group.set_planning_time(10.0)
        # self._group.set_goal_tolerance(0.1) 

        planner_param = '/move_group/dorna/planner_configs'
        if rospy.has_param(planner_param):
            self._planners = rospy.get_param(planner_param)

        self._target_names = ["allZeros", "home", "doorPose", "ElbowDown"]
        ###################################################################
        ################# PlanningSceneInterface ##########################
        ###################################################################
        #TODO: look into moveit_python vs moveit_commander
        # self._scene = PlanningSceneInterface(frame="dorna_base")
        # self.create_scene()

        ######################################################
        ################# Ros Setup ##########################
        ######################################################
        self._rate = rospy.Rate(10)

        ######################################################
        ################# Publishers #########################
        ######################################################
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size = 10)
        
        ######################################################
        #################  Services  #########################
        ######################################################
        srv_topics = ['view','move_home', 'move_extend', 'execute', 'plan']
        srv_msg_types = [DornaView, DornaPlanCmd, DornaPlanCmd, DornaExecute, DornaPlan]
        self._srv_list = {}
        for topic, msg_type in zip(srv_topics, srv_msg_types):
            callback = getattr(self, topic+"_srv_callback", None)
            self._srv_list[topic] = rospy.Service(ns+'/moveit_cmd/'+topic, msg_type, callback)

        self._play_service = rospy.ServiceProxy(ns+'/robot_cmd/play', DornaPlay)



    ####################################################################
    ####################### Helper Methods #############################
    ####################################################################
    # def create_scene(self):
    #     self._scene.addBox(name= "cart", size_x = 0.5, size_y = 0.3, size_z = 0.7,  x = -0.175, y = 0.0, z = -0.362)
    #     self._scene.setColor(name = "cart", r = 255, g = 0, b = 0)
    #     self._scene.addBox(name= "wood_base", size_x = 0.15, size_y = 0.235, size_z = 0.060, x = 0.0, y = 0.0, z = -0.03)
    #     self._scene.setColor(name = "wood_base", r = 0, g = 255, b = 0)
    #     self._scene.addBox(name = "right_wall", size_x = 0.85, size_y = 0.1, size_z = 2.0, x = -0.15, y = -0.22, z = 0.275)
    #     self._scene.setColor(name = "right_wall", r = 0, g = 0, b = 0, a = 0.5)
    #     self._scene.addBox(name = "back_wall", size_x = 0.1, size_y = 1.4, size_z = 2.0, x = -0.600, y = .53, z = 0.275)
    #     self._scene.setColor(name = "back_wall", r = 0, g = 0, b = 0, a = 0.9)
    #     # self._scene.addBox(name = "front_wall", size_x = 0.1, size_y = 0.6, size_z = 2.0, x = 0.300, y = 0.13, z = 0.275)
    #     # self._scene.setColor(name = "front_wall", r = 0, g = 0, b = 0, a = 0.9)
    #     self._scene.addBox(name = "front_wall2", size_x = 0.1, size_y = 0.2, size_z = 2.0, x = 0.300, y = 1.1, z = 0.275)
    #     self._scene.setColor(name = "front_wall2", r = 0, g = 0, b = 0, a = 0.9)
    #     self._scene.addBox(name = "left_wall", size_x = 0.85, size_y = 0.1, size_z = 2.0, x = -0.15, y = 1.28, z = 0.275)
    #     self._scene.setColor(name = "left_wall", r = 0, g = 0, b = 0, a = 0.5)
    #     self._scene.sendColors()
        # self._scene.addBox(name="table", size_x=1.0, size_y=1., size_z=0.1, x=-0.45, y=0., z=-0.05)
        # self._scene.addBox(name="wall", size_x=0.1, size_y=1., size_z=1., x=0.4, y=0., z=0.5)

    def find_transform(self, parent_frame, child_frame):
        transform = self._tf_buffer.lookup_transform(
            parent_frame,
            child_frame, #source frame
            rospy.Time(0), #get the tf at first available time
            rospy.Duration(1.0)) #wait for 1 second

        return transform

    def publish_traj(self):
        display_trajectory = DisplayTrajectory()
        if self.trajectory[0]:
            display_trajectory.trajectory.append(self.trajectory[1])
            self._display_trajectory_publisher.publish(display_trajectory)
        else:
            rospy.logwarn("No trajectory to view")

    #######################################################################
    ####################### Service Callbacks #############################
    #######################################################################
    def plan_srv_callback(self, req):
        rospy.loginfo("[Service Callback] Plan")
        try:
            self.go_joint_value_target(req.joint_angles)
            if self.trajectory[0]:
                return self.trajectory[1]
        except moveit_commander.exception.MoveItCommanderException:
            rospy.logwarn("Moveit failed to find a trajectory")
            return RobotTrajectory()

    def execute_srv_callback(self, req):
        rospy.loginfo("[Service Callback] Execute")
        self._play_service(req.trajectory.joint_trajectory)
        return 

    def view_srv_callback(self, req):
        rospy.loginfo("[Service Callback] View")
        self.publish_traj()
        return True
    
    def move_home_srv_callback(self, req):
        rospy.loginfo("[Service Callback] MoveHome")
        self.go_named_target('home')
        return self.trajectory

    def move_extend_srv_callback(self, req):
        rospy.loginfo("[Service Callback] MoveExtend")
        self.go_named_target("allZeros")
        return self.trajectory

    #################################
    #### Set movegroup_commander ####
    #################################
    def plan(self):
        try:
            self.trajectory = self._group.plan()
            rospy.loginfo(self.trajectory)
        except moveit_commander.exception.MoveItCommanderException:
            rospy.logwarn("Moveit failed to find a trajectory")
            self.trajectory = RobotTrajectory()

    def go(self):
        if self._gazebo:
            self._group.go(wait=True)

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
