cmake_minimum_required(VERSION 3.8)
project(orchestrator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visual_calibration_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(calibration_orchestrator_node
  src/orchestrator/calibration_orchestrator_node.cpp
  src/orchestrator/main.cpp
)
target_include_directories(calibration_orchestrator_node PRIVATE
  include
)
ament_target_dependencies(calibration_orchestrator_node
  rclcpp
  rclcpp_action
  std_srvs
  geometry_msgs
  visual_calibration_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  calibration_orchestrator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()