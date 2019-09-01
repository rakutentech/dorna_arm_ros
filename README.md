## Dorna Robot Rakuten
    Author: Mitchell Fogelson
    Email: mitchell.fogelson@rakuten.com
    
## Dorna_robot Installation Guide

Clone:
   
 ```git clone http://git.rit/robotics/code/dorna_robot/dorna_robot.git```

Move to directory:
  
```cd dorna_robot/```

Update submodules:
   
```git submodule update --init --recursive```

## RoS Dependencies
Gazebo 9
gazebo_ros
ros_control
ros_controllers
moveit
joint_state_publisher
robot_state_publisher

## Quick Start

Dorna Robot:
    See dorna_api for more information

    Launching: 
    roslaunch dorna_api dorna_api.launch
    rosrun dorna_api dorna_main.py
    rosrun dorna_nodes dorna_moveit_real.py

Dorna Gazebo:
    See dorna_gazebo for more information

    Launching:
    roslaunch dorna_gazebo dorna.launch
    rosrun dorna_nodes dorna_moveit_gazebo.py

    [Warning: If the above launching sequence fails it is probably due to dorna_planning_execution.launch being launched too soon.  You may need to use this launch sequence instead and comment out the include in the dorna.launch.]

    Launching:
    roslaunch dorna_gazebo dorna.launch
    roslaunch dorna_moveit_config dorna_planning_execution.launch
    rosrun dorna_nodes dorna_moveit_gazebo.py

## Packages

dorna_api:
    This package interfaces with the actual robot.
    dorna_main.py --> Is the ROS wrapper for the robot
    dorna_robot.py --> Is the API interface for the robot
    dorna_planner.py --> Is a message converter for the robot

dorna_description:
    This package is Xacro and URDF descriptions that interface with Rviz and Gazebo.

dorna_control:
    This package interfaces with ros_control to allow moveit to control the Gazebo model

dorna_gazebo:
    This package simply launches the various models both Dorna and enviornment related.

dorna_moveit_config:
    This package controls all the moveit interface with both gazebo and the real robot

dorna_nodes: 
    This package contains any nodes that may not directly fit within the other packages mentioned above
