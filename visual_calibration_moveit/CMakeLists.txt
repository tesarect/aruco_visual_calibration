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
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

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

add_executable(simple_trajectory
  src/simple_trajectory/simple_trajectory.cpp
  src/simple_trajectory/main.cpp
)
target_include_directories(simple_trajectory PRIVATE
  include
)
ament_target_dependencies(simple_trajectory
  rclcpp
  moveit_ros_planning_interface
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

# MoveIt Task Constructor: deactivated, not just unbuildable-and-skipped.
# Unresolved upstream packaging bug (py_binding_tools vs. py_bindings_tools,
# see moveit/moveit_task_constructor#629) — see README.md. Files are left in
# place; uncomment this block (and the matching depend in package.xml) to
# reactivate once that's fixed upstream.
# find_package(moveit_task_constructor_core QUIET)
# if(moveit_task_constructor_core_FOUND)
#   add_executable(mtc_trajectory
#     src/mtc_trajectory/mtc_trajectory.cpp
#     src/mtc_trajectory/main.cpp
#   )
#   target_include_directories(mtc_trajectory PRIVATE
#     include
#   )
#   ament_target_dependencies(mtc_trajectory
#     rclcpp
#     moveit_task_constructor_core
#   )
# endif()

install(TARGETS
  planning_scene_setup
  simple_trajectory
  DESTINATION lib/${PROJECT_NAME}
)

# if(moveit_task_constructor_core_FOUND)
#   install(TARGETS
#     mtc_trajectory
#     DESTINATION lib/${PROJECT_NAME}
#   )
# endif()

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
