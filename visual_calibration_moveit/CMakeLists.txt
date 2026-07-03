cmake_minimum_required(VERSION 3.8)
project(visual_calibration_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(planning_scene_setup
  src/planning_scene_setup/planning_scene_setup.cpp
  src/planning_scene_setup/scene_object_types.cpp
  src/planning_scene_setup/main.cpp
)
target_include_directories(planning_scene_setup PRIVATE
  include
)
ament_target_dependencies(planning_scene_setup
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometric_shapes
  shape_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  planning_scene_setup
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
