#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, Int32
import numpy as np
from threading import Thread

class SimpleController():
	def __init__(self, loop_rate=10):
		self._subscriber_list = {}
		self._rate = rospy.Rate(loop_rate)
		self._n_joints = 5

		self._publisher_list = []
		for i in range(self._n_joints):
			self._publisher_list.append(rospy.Publisher('/dorna/joint'+str(i)+'_position_controller/command', Float64, queue_size=10))
		
		self._command = np.zeros((self._n_joints,))
		self._itter = 0

	def run(self):
		while not rospy.is_shutdown():
			self._rate.sleep()
			self.publish()
			self.update_command()
			self._itter+=1

	def publish(self):
		for i in range(self._n_joints):
			self._publisher_list[i].publish(self._command[i])
		
	def update_command(self):
		for i in range(self._n_joints):
			self._command[i] = np.sin(self._itter)
	


if __name__=="__main__":
	try:
		rospy.init_node('simple_controller', anonymous = True)
		rospy.loginfo('Starting node "simple_controller"')
		s_c = SimpleController()
		runner = Thread(target=s_c.run)
		runner.start()
		rospy.spin()
		runner.join()
	except rospy.ROSInterruptException: pass
