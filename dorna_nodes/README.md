## Dorna Nodes
    Author: Mitchell Fogelson
    Maintainer: Mitchell Fogelson

## About
This package is for nodes to interface with the rest of the packages.  

For example it contains a nodes to allow Moveit_ros to be used with [dorna_ros](http://git.fut.rit/robotics/code/dorna_robot/dorna_ros).

## Nodes
[dorna_moveit_ros.cpp](http://git.fut.rit/robotics/code/dorna_robot/dorna_nodes/blob/master/src/dorna_moveit_ros.cpp): A planning and executing interface between dorna_ros and Moveit_ros

[dorna_moveit_real.py](http://git.fut.rit/robotics/code/dorna_robot/dorna_nodes/blob/master/src/dorna_moveit_real.cpp): A planning and executing interface between dorna_ros and Moveit_ros in Python
    
[dorna_moveit_gazebo.py](http://git.fut.rit/robotics/code/dorna_robot/dorna_nodes/blob/master/src/dorna_moveit_gazebo.cpp): This node is very similar to dorna_moveit_real.py, but creates an instance of the MoveitCommander for Dorna model in Gazebo.


        