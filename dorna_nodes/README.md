## Dorna Nodes
    Author: Mitchell Fogelson
    Maintainer: Mitchell Fogelson

## About
    Description:  This package is for nodes to interface with the rest of the packages.  For example it contains a nodes to allow for a human interface to communicate to moveit and send execution to gazebo.

## Nodes
    dorna_moveit_real.py:
        This node interfaces with the real robot through dorna_main and moveit_commander
    
    dorna_moveit_gazebo.py
        This node is very similar to dorna_moveit_real.py, but creates an instance of the MoveitCommander for Dorna model in Gazebo.
        