## Dorna Arm ROS 
    Author: Mitchell Fogelson
    Email: mfogelson630@gmail.com
    
**Note** I am not affiliated in any way the developement of Dorna Arm
## Dorna_robot Installation Guide

Clone:
   
 ```
 cd ~/catkin_ws/src
 git clone https://github.com/rakutentech/dorna_arm_ros.git
 ```

Download Dependencies:
```
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=ROSDISTRO -r -y
catkin_make
```

**Note:** See [dorna_ros](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_ros) for more detailed instruction including how to install Dorna arm API and udev rule


## Quick Start

**Dorna Robot ROS**:

See [dorna_ros](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_ros) for more information

```
roslaunch dorna_ros dorna_ros.launch
```
**Dorna Gazebo Simulation**:

See [dorna_gazebo](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_gazebo) for more information

```
roslaunch dorna_gazebo dorna.launch
rosrun dorna_nodes dorna_moveit_gazebo.py
```

## Packages
[dorna_control](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_control):
This package interfaces with ros_control to allow moveit to control the Gazebo model

[dorna_description](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_description):
This package is Xacro and URDF descriptions that interface with Rviz and Gazebo.

[dorna_gazebo](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_gazebo):
This package simply launches the various models both Dorna and enviornment related.

[dorna_moveit_config](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_moveit_config):
This package controls all the moveit interface with both gazebo and the real robot

[dorna_nodes](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_nodes): 
This package contains any nodes that may not directly fit within the other packages mentioned above

[dorna_ros](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_ros):
This package interfaces with the actual robot.

