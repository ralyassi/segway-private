cmake_minimum_required(VERSION 3.8)
project(segway_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find dependencies used in .msg/.srv/.action files
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Message files
set(msg_files
  msg/ErrorCodeFb.msg
  msg/MotorWorkModeFb.msg
  msg/SpeedFb.msg
  msg/TicksFb.msg
  msg/BmsFb.msg
  msg/ChassisCtrlSrcFb.msg
  msg/ChassisMileageMeterFb.msg
  msg/ChassisModeFb.msg
  msg/FrontAxleAngleFb.msg
)

# Service files
set(srv_files
  srv/RosEnableChassisRotateCmd.srv
  srv/RosGetChassisRotateSwitchCmd.srv
  srv/RosClearChassisErrorCodeCmd.srv
  srv/RosGetChassisSnCmd.srv
  srv/RosGetLoadParamCmd.srv
  srv/RosGetSwVersionCmd.srv
  srv/RosGetVelMaxFeedbackCmd.srv
  srv/RosSetChassisEnableCmd.srv
  srv/RosSetChassisPoweroffCmd.srv
  srv/RosSetLoadParamCmd.srv
  srv/RosSetVelMaxCmd.srv
  srv/ChassisSendEvent.srv
  srv/RosResetHostPowerCmd.srv
  srv/RosStartChassisLeftRotateCmd.srv
  srv/RosStartChassisRightRotateCmd.srv
  srv/RosStopChassisRotateCmd.srv
  srv/RosGetRotateFunctionCfgCmd.srv
  srv/RosSetCfgRotateFunctionCmd.srv
  srv/RosGetHostAndChassisMatchCmd.srv
  srv/RosSetChassisBuzzerCmd.srv
)

# Action files
set(action_files
  action/RosSetIapCmd.action
)

# Register interfaces for build
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs nav_msgs action_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
