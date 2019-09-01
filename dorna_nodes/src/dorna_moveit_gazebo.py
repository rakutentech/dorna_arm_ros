#!/usr/bin/env python2

import copy
import moveit_commander
import numpy as np
import pickle
import rospy
import sys
from threading import Thread
import time

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import RobotTrajectory, DisplayTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Int32
from trajectory_msgs.msg import JointTrajectory

class MoveitGazeboInterface():
	def __init__(self, loop_rate=10, pose_target = None):
		self._robot = moveit_commander.RobotCommander()
		self._scene = moveit_commander.PlanningSceneInterface()
		self._group = moveit_commander.MoveGroupCommander("manipulator")
		self._group.set_planner_id("SBL")
		self._planners = ['SBL', 'EST','LBKPIECE','BKPIECE','KPIECE',
                        'RRT','RRTConnect','RRTstar','TRRT','PRM',
                        'PRMstar','FMT','BFMT','PDST','STRIDE','BiTRRT',
                        'LBTRRT','BiEST','ProjEST','LazyPRM','LazyPRMstar',
                        'SPARS','SPARStwo']
		self._group.set_planning_time(10.0)
		self._group.set_goal_tolerance(0.1)
		self._joint_names = ['dorna_base_to_dorna_arm_0', 'dorna_arm_0_to_dorna_arm_1', 'dorna_arm_1_to_dorna_arm_2', 'dorna_arm_2_to_dorna_arm_3', 'dorna_arm_3_to_dorna_arm_4']
		
		self._rate = rospy.Rate(loop_rate)
		self._n_joints = 5
		self._target_names = ["allZeros", "home", "doorPose", "ElbowDown"]

		self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size = 10)
		
		#TODO: add input agrs when running this node
		# self.pub = rospy.Publisher('/dorna/dorna_effort_controller/command', JointTrajectory, queue_size=10)
		self.plan = None
		self.pub = rospy.Publisher('/dorna/dorna_position_controller/command', JointTrajectory, queue_size=10)

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
			rospy.loginfo(self._group.has_end_effector_link())
			self.go_home()
			time.sleep(2.0)
			self.go_named_target(name='ElbowDown')
			time.sleep(2.0)
			rospy.loginfo("TESTING ORIENTATION_TARGET: roll by 90 deg")
			des_pose = self._group.get_current_pose()
			des_pose.pose.orientation.x = np.sqrt(0.5)
			des_pose.pose.orientation.w = np.sqrt(0.5)
			response = str(raw_input("Did test work? 	"))
			self.go_pose_target(des_pose)
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
			


	# def draw_heart(self):
	# 	thetas = pickle.load(open('/home/rit/catkin_ws/src/dorna_robot/draw_heart_thetas.p','rb'))
	# 	for theta in thetas:
	# 		if theta[0] > np.pi:
	# 			theta[0] -= 2*np.pi
	# 		if theta[1] > np.pi:
	# 			theta[1] -= 2*np.pi
	# 		if theta[2] > np.pi:
	# 			theta[2]-= np.pi
	# 		# theta = [theta[0]-2.*np.pi, theta[1], theta[2]-np.pi, theta[3], theta[4]]
	# 		rospy.loginfo(theta)
	# 		traj = JointState()
	# 		traj.name = self._joint_names
	# 		traj.position = theta
	# 		self._group.plan(joints=traj)
	# 		self._group.go(wait=True)


	def open_door(self):
		traj = JointState()
		traj.name = self._joint_names
		positions = [[0, 1.57, 1.57, 0, 0], [0, -1, 0, 0, 0]]
		for position in positions:
			traj.position = position
			self._group.plan(joints=traj)
			self._group.go(wait=True)
		
	
	def visualize_plan_rviz(self):
		display_trajectory = DisplayTrajectory()
		display_trajectory.trajectory_start = self._robot.get_current_state()
		display_trajectory.trajectory.append(self.plan)
		self._display_trajectory_publisher.publish(display_trajectory)
		rospy.sleep(5)

	#################################
	#### Set movegroup_commander ####
	#################################
	# def set_joint_value_target(self, arg1, arg2 = None, arg3 = None):
	def go(self):
		plan = self._group.plan()
		print(plan)
		self._group.go(wait=True)

	def go_home(self):
		self.go_named_target('home')
	
	def go_rpy_target(self, rpy):
		"""rpy should be a list"""
		try:
			if type(rpy) == list and len(rpy) == 3:
				self._group.set_rpy_target(rpy)
				self.go()
			else:
				raise TypeError
		except TypeError:
			raise

	def go_orientation_target(self, q):
		"""q should be a list"""
		try:
			if type(q) == list and len(q) == 4:
				self._group.set_orientation_target(q)
				self.go()
			else:
				raise TypeError
		except TypeError:
			raise

	def go_position_target(self, xyz):
		"""xyz should be a list"""
		try:
			if type(xyz) == list and len(xyz) == 3:
				self._group.set_position_target(xyz)
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
				self.go()
			else:
				raise TypeError
		except TypeError:
			raise

	def go_pose_targets(self, poses):
		try:
			if type(poses) == list and type(type(poses[0])) == Pose:
				self._group.set_pose_targets(poses)
				self.go()
			else:
				raise TypeError
		except TypeError:
			raise

	def go_shift_pose_target(self, axis, value):
		""" Get the current pose of the end effector, add value to the corresponding axis (0..5: X, Y, Z, R, P, Y) and set the new pose as the pose target """
		try:
			if type(axis) == int:
				self._group.shift_pose_target(axis, value)
				self.go()
			else:
				raise TypeError
		except TypeError:
			raise
	
	def go_random_target(self):
		self._group.set_random_target()
		self.go()

	def go_named_target(self, name):
		rospy.loginfo("Called go {}".format(name))
		if name in self._target_names:
			self._group.set_named_target(name)
			self.go()
		



if __name__=="__main__":
	try:
		rospy.init_node('moveit_gazebo_wrapper', anonymous = True)
		rospy.loginfo('Starting node "moveit_gazebo_wrapper"')
		moveit_commander.roscpp_initialize(sys.argv)
		m_g_w = MoveitGazeboInterface()
		m_g_w.run_testing_methods()
		# m_g_w.run()
		# runner = Thread(target = m_g_w.run_human_interface)
		# runner.start()
		# rospy.spin()
		# runner.join()
	except rospy.ROSInterruptException: pass
