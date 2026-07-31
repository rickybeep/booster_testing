cmake_minimum_required(VERSION 3.15)
project(booster_interface)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ButtonEventMsg.msg"
  "msg/ImuState.msg"
  "msg/MotorState.msg"
  "msg/MotorCmd.msg"
  "msg/LowCmd.msg"
  "msg/LowState.msg"
  "msg/RemoteControllerState.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
