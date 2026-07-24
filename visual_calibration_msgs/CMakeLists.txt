cmake_minimum_required(VERSION 3.8)
project(visual_calibration_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlanningFailure.msg"
  "msg/AutoCalibrateStatus.msg"
  "msg/Detection2D.msg"
  "msg/Detection2DArray.msg"
  "srv/TracePath.srv"
  "srv/GetPolygonWaypoints.srv"
  "srv/GetStandoffPose.srv"
  "srv/GetPresetPose.srv"
  "srv/MoveToPreset.srv"
  "srv/SetDetectorMode.srv"
  "action/Calibrate.action"
  "action/AutoCalibrate.action"
  DEPENDENCIES geometry_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()