cmake_minimum_required(VERSION 3.5)
project(cf_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# List of message files
set(msg_files
  "msg/ControllerRPYRate.msg"
  "msg/ControllerRPYT.msg"
  "msg/StateEstimate.msg"
  "msg/KalmanPositionEst.msg"
  "msg/MotorPower.msg"
  "msg/PosCtl.msg"
  "msg/Stabilizer.msg"
  "msg/Fly.msg"
  "msg/FlyStamped.msg"
  "msg/Hover.msg"
  "msg/HoverStamped.msg"
  "msg/PositionStamped.msg"
)

set(action_files
  # "action/CheckForObjects.action"
)

set(srv_files
  "srv/Land.srv"
  "srv/SendHoverSetpoint.srv"
  "srv/ResetPositionEstimator.srv"
  "srv/SetParam.srv"
  "srv/TakeOff.srv"
  "srv/VelocityControl.srv"
  "srv/PositionControl.srv"
  "srv/PoseTransform.srv"
  )

# Generate messages & services
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()