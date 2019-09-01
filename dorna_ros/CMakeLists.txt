cmake_minimum_required(VERSION 2.8.3)
project(dorna_ros)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs moveit_msgs trajectory_msgs message_generation dynamic_reconfigure)

add_message_files(
FILES 
	DornaPos.msg 
	DornaHomed.msg 
	DornaJerk.msg
	DornaJoint.msg
	DornaLimits.msg
	DornaSpeed.msg
	DornaToolHead.msg
	Job.msg
	DornaJob.msg
	DornaUnits.msg
	DornaMotion.msg
	DornaInfo.msg
	DornaGoal.msg
)

add_service_files(
FILES
	DornaCalibrate.srv
	DornaConnect.srv
	DornaDisconnect.srv
	DornaErase.srv
	DornaExecute.srv
	DornaHalt.srv
	DornaHome.srv
	DornaJog.srv
	DornaPause.srv
	DornaPlanCmd.srv
	DornaPlan.srv
	DornaPlay.srv
	DornaSet.srv
	DornaView.srv
	HumanOpCmd.srv
	DornaXYZ.srv
	DornaXYZCheck.srv
	DornaXYZtoJoint.srv
	TfTransform.srv
	DornaMoveJoints.srv
	DornaMoveXYZ.srv
	DornaOutputPin.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  trajectory_msgs
  moveit_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Dorna.cfg
)

catkin_package(CATKIN_DEPENDS message_runtime)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
