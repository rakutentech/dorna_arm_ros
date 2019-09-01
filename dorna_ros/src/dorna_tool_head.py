#!/usr/bin/env python3

import rospy
import numpy as np
import transforms3d
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
from dorna_ros.srv import DornaMoveXYZ, DornaHalt, DornaMoveJoints
from std_srvs.srv import Empty


class DornaToolhead: 
    def __init__(self):
        self.hand_pin = "in2" #input id for hand sensor
        self.force_data = 0
        self.light_data = 0

        self.accel_goal = [0.0, np.cos(55.*np.pi/180.0)*9.8, np.sin(55.*np.pi/180.0)*9.8]
        self.p_roll = 5.0
        self.d_roll = 0.01
        self.p_pitch = 5.0
        self.d_pitch = 0.01
        self.epsilon = 0.05
        self.time = rospy.get_rostime()

        self.triggered = False

        self.move_back = rospy.ServiceProxy('dorna/robot_cmd/move_xyzab', DornaMoveXYZ)
        self.halt = rospy.ServiceProxy('dorna/robot_cmd/halt', DornaHalt)
        self.move_joints = rospy.ServiceProxy('dorna/robot_cmd/move_joints', DornaMoveJoints)
        self.move_to_home = rospy.ServiceProxy('dorna/robot_cmd/move_to_home', Empty)

        self.reset_trigger_srv = rospy.Service('robot_hand/reset_trigger', Empty, self.reset_trigger)

        ####################################
        ####    robot_hand Services      ####
        ####################################
        self.force_data_sub = rospy.Subscriber("/robot_hand/force_reading", Int32, self.force_data_cb)
        self.light_data_sub = rospy.Subscriber("/robot_hand/light_reading", Int32, self.light_data_cb)
        self.imu_data_sub = rospy.Subscriber("/camera/accel/sample", Imu, self.imu_cb)

    def sensor_check(self):
        try:
            while not rospy.is_shutdown():
                rospy.loginfo_once("Starting sensor_check Thread")
                self.sensor_check_callback()
        except KeyboardInterrupt:
            raise

    def force_data_cb(self, force):
        self.force_data = force.data
        
    def light_data_cb(self, light):
        self.light_data = light.data

    def sensor_check_callback(self):
        LIGHT_TRIGGER = 675
        FORCE_TRIGGER = 130
        FORCE_SAFETY_MAX = 170

        if not self.triggered:

            if self.force_data >= FORCE_TRIGGER and self.light_data >= LIGHT_TRIGGER:
                self.triggered = True
                rospy.loginfo("Force and Light sensors detect button push.")
                rospy.loginfo("Stopping robot and returning to home position.")
                self.halt()
                self.move_joints('joint', 0, 7000, [0, 145, -120, -25, 0], True)
                

            elif self.force_data >= FORCE_SAFETY_MAX:
                self.triggered = True
                rospy.loginfo("Force over the safety limit.")
                rospy.loginfo("Stopping robot and returning to home position.")
                self.halt()
                self.move_joints('joint', 0, 7000, [0, 145, -120, -25, 0], True)


            # elif self.force_data < FORCE_TRIGGER and self.light_data >= LIGHT_TRIGGER:
            #     rospy.loginfo("Light was triggered but Force trigger was not reached.")
            #     rospy.loginfo("Stopping robot and returning to home position.")
            #     self._robot.halt()
            #     self._robot.move_to_home()

    def reset_trigger(self, req):
        self.triggered = False
        return {}

    def imu_cb(self, data):
        self.prev_time = self.time
        self.accel_data = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]
        self.time = rospy.get_rostime()

    def home_hand(self):
        self.update_roll()
        self.update_pitch()

    def get_rotation(self):
        axis = np.cross(self.accel_data, self.accel_goal)
        angle = np.arccos(np.dot(self.accel_data, self.accel_goal)/(np.linalg.norm(self.accel_goal)*np.linalg.norm(self.accel_data)))
        roll, pitch, yaw = transforms3d.euler.axangle2euler(axis, angle)
        rospy.loginfo("roll: {}".format(roll))
        rospy.loginfo("pitch: {}".format(pitch))
        rospy.loginfo("yaw: {}".format(yaw))
        # return roll, pitch, yaw

    def update_roll(self):
        error = self.accel_goal[0]-self.accel_data[0] 
        prev_error = 0
        while np.sqrt(error**2) >= self.epsilon: 
            rospy.loginfo("error = {}".format(error))
            prev_error = error
            error = self.accel_goal[0]-self.accel_data[0]
            d_error = (error - prev_error)/(self.time.to_sec()-self.prev_time.to_nsec())

            theta = self.p_roll * error + self.d_roll*d_error
            self.move_joints('joint', 1, 1000, [0, 0, 0, 0, theta], True)

    def update_pitch(self):
        error = self.accel_goal[1]-self.accel_data[1] 
        prev_error = 0
        while np.sqrt(error**2) >= self.epsilon: 
            rospy.loginfo("error = {}".format(error))
            prev_error = error
            error = self.accel_goal[1]-self.accel_data[1] 
            d_error = (error - prev_error)/(self.time.to_sec()-self.prev_time.to_nsec())
            theta = self.p_pitch * error + self.d_pitch * d_error
            self.move_joints('joint', 1, 1000, [0, 0, 0, -theta, 0], True)


               