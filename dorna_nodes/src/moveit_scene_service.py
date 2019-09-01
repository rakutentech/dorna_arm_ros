#!/usr/bin/env python

import sys
import copy
import rospy
import numpy as np
from threading import Thread
import pickle

import tf
import tf2_ros
import moveit_commander

from moveit_msgs.msg import RobotTrajectory, DisplayTrajectory, CollisionObject
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Float64, Int32, String, Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from apriltags2_ros.msg import AprilTagDetectionArray
from shape_msgs.msg import MeshTriangle, Mesh, SolidPrimitive, Plane

sys.path.append('/home/rit/catkin_ws/src/dorna_robot')
import DornaUtils

class MoveitSceneService():
    def __init__(self, loop_rate=10, pose_target = None):
        self._scene = moveit_commander.PlanningSceneInterface()
        self.create_scene()


    def add_object(self):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))


    

if __name__=="__main__":
    try:
        rospy.init_node('moveit_scene_service', anonymous = True)
        rospy.loginfo('Starting node "moveit_scene_service"')
        moveit_commander.roscpp_initialize(sys.argv)

        MSS = MoveitSceneService()
        runner = Thread()
        runner.start()
        rospy.spin()
        runner.join()   
    except rospy.ROSInterruptException: pass
