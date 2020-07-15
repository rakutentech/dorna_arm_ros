# Dorna Ros
    Author: Mitchell Fogelson
    Maintainer: Mitchell Fogelson

## Requirements
    Python: 3.5+
    Ubuntu: 18.04
    RoS: Melodic
    
Dependencies:
*  [Dorna API](https://github.com/dorna-robotics/dorna)
*  [transform3D](https://pypi.org/project/transforms3d/)

## About
    This package has several tools that allows you to easily interface RoS 
    with the Dorna arm. It attemps to mirror the API format and command names for ease of use.
    
## Dorna API Installation
Clone Dorna API
  ```
   cd
   git clone https://github.com/dorna-robotics/dorna
   cd dorna
   sudo python3 setup.py install
  ```
**Test Install**
   ```
   python3
   from dorna import Dorna
   robot = Dorna()
   robot.connect()
   ```
Notes:
* If python3 fails try specifying version (eg. python3.5, python3.6 python3.7)
* May need to install pip3
* May need to run python in sudo (eg. run sudo -su)
* Check USB ports 
* May need to install setuptools (sudo apt-get install python3-setuptools)

## Dorna ROS Installation
1. Install ROS Melodic 
[Melodic Install](http://wiki.ros.org/melodic/Installation/Ubuntu)
2. Clone repository
```
cd ~/catkin_ws/src
git clone https://github.com/rakutentech/dorna_arm_ros.git
pip3 install catkin_pkg
pip3 install rospkg
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
catkin_make
```
## udev_rule Installation
This will allow your computer to recognize the Dorna Robotic Arm when it is plugged in as /dev/actuators/dorna


  *Note: This can cause issues if you have more then 1 Dorna robot or Arduino Due plugged in*
```
cd ~/catkin_ws/src/dorna_arm_ros/dorna_ros
./dorna_udev_install.sh
```
Unplug then replug in device

**Test**
```
ls /dev/actuators/dorna
```
Returns
```
/dev/actuators/dorna
```

## Quick Start
```
roslaunch dorna_ros dorna_ros.launch
```


## Publishes
* /dorna/robot_info/cartesian_position [[dorna_ros/DornaPos]](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_ros/msg/DornaPos.msg): Position of end effector [x,y,z, roll, pitch] 
* /dorna/robot_info/dorna_params [[dorna_ros/DornaInfo]](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_ros/msg/DornaInfo.msg): Loaded parameters of dorna (Depricated please use parameter server)
* /dorna/robot_info/homed_status [[dorna_ros/DornaHomed]](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_ros/msg/DornaHomed.msg): Joint homed status [j0, j1, j2, j3, j4] 1 = True, 0 = False
* /dorna/robot_info/joint_angles [[dorna_ros/DornaJoint]](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_ros/msg/DornaJoint.msg): Joint angles in degrees
* /dorna/robot_info/joint_states [[sensor_msgs/JointState]](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html): Joint state message angles in radians
* /dorna/robot_info/state [[std_msgs/Int32]](http://docs.ros.org/melodic/api/std_msgs/html/msg/Int32.html): Dorna state 0 = Ready, 1 = Moving, 2 = Complete
* /dorna/robot_info/triggered [[std_msgs/Bool]](http://docs.ros.org/melodic/api/std_msgs/html/msg/Bool.html): Dorna toolhead trigger (For application with force sensitive finger)

## Services
* /dorna/robot_cmd/calibrate [[dorna_ros/DornaCalibrate]](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_ros/srv/DornaCalibrate.srv): [Dorna calibration](https://github.com/dorna-robotics/dorna/wiki/api#calibration)
* /dorna/robot_cmd/connect [[dorna_ros/DornaConnect]](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_ros/srv/DornaConnect.srv): Connects to Dorna Robot
* /dorna/robot_cmd/digital_out [[dorna_ros/DornaDigitalOut]](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_ros/srv/DornaDigitalOut.srv): Set IO pins
* /dorna/robot_cmd/disconnect [[dorna_ros/DornaDisconnect]](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_ros/srv/DornaDisconnect.srv): Disconnect from Robot
* /dorna/robot_cmd/halt [[dorna_ros/DornaHalt]](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_ros/srv/DornaHalt.srv): Stop all commands and clear que
* /dorna/robot_cmd/home [[dorna_ros/DornaHome]](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_ros/srv/DornaHome.srv): Homing process (Homes all joints)
* /dorna/robot_cmd/jog [[dorna_ros/DornaJog]](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_ros/srv/DornaJog.srv): Jog joints or xyz
* /dorna/robot_cmd/move_joints [[dorna_ros/DornaMoveJoints]](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_ros/srv/DornaMoveJoints.srv): Send Joint command
* /dorna/robot_cmd/move_to_home [[dorna_ros/DornaMoveHome]](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_ros/srv/DornaMoveHome.srv): Move to home position
* /dorna/robot_cmd/move_xyzab [[dorna_ros/DornaMoveXYZAB]](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_ros/srv/DornaMoveXYZAB.srv): Send XYZAB command
* /dorna/robot_cmd/pause [[dorna_ros/DornaPause]](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_ros/srv/DornaPause.srv): Pause Dorna commands
* /dorna/robot_cmd/play [[dorna_ros/DornaPlay]](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_ros/srv/DornaPlay.srv): Play Dorna commands
* /dorna/robot_cmd/set_joint [[dorna_ros/DornaSetJoint]](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_ros/srv/DornaSetJoint.srv): Set value for dorna joint angles
* /dorna/robot_cmd/xyz_check [[dorna_ros/DornaXYZCheck]](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_ros/srv/DornaXYZCheck.srv): See if position is valid for workspace
* /dorna/robot_cmd/xyz_to_joint [[dorna_ros/DornaXYZToJoint]](https://github.com/rakutentech/dorna_arm_ros/tree/master/dorna_ros/srv/DornaXYZToJoint.srv): Get IK for XYZ position

