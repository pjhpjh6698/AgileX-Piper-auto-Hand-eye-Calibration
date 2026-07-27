cmake_minimum_required(VERSION 3.8)
project(auto_handeye_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # --- messages ---
  "msg/MarkerDetection.msg"
  "msg/RobotState.msg"
  "msg/CalibrationStatus.msg"
  # --- services ---
  "srv/ResetCalibration.srv"
  "srv/SaveCalibration.srv"
  "srv/LoadCalibration.srv"
  "srv/PublishCalibrationTf.srv"
  "srv/AddManualSample.srv"
  # --- actions ---
  "action/RunCalibration.action"
  "action/MoveToCalibrationPose.action"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
