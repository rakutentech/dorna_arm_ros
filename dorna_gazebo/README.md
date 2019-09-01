## Dorna Gazebo
    Author: Mitchell Fogelson
    Maintainer: Mitchell Fogelson
    
## About
    Desciption: Contains the launch files robot in gazebo

## Quick Start:
    roslaunch dorna_gazebo dorna.launch [args]

## Args
    controller_trype ->  Default = True
        If set to true, dorna.launch will load the dorna description with with the ros_control position controller hardware interface

        Else, it will load the dorna model with ros_control effort controller hardware interface

       
    rviz_mode -> Default = True
        If set to true, dorna.launch will launch the dorna_planning_execution.launch with the rviz gui.  This is helpful for debuging and showing the trajectory path prior to executing.

        Else, it will not launch the rviz gui

        
    door_sim -> Default = False
        If set to true, dorna.launch will include door.launch from the door_description package.  It will also change the initial position of the dorna robot to be at the appropriate height and distance to the door.

    agv_model -> Default = False
        If set to true, dorna.launch will include agv.launch from the agv_description package. 


