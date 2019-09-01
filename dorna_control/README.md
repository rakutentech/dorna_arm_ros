## Dorna Control
    Author: Mitchell Fogelson
    Maintainer: Mitchell Fogelson

## About
    Description: This package contains the files which allow the robot to interface with ros_control.  

## Launch
    roslaunch dorna_control.launch [args]

## Args
    controller_type -> Default = True
        If true, dorna_control.launch will load and spawn the dorna_position_controller and joint_state_controller description from dorna_controller.yaml in the config file.  

        Else, dorna_control.launch will load and spawn the dorna_effort_controller and joint_state_controller description from dorna_controller.yaml in the config file.
