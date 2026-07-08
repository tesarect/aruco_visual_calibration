cmake_minimum_required(VERSION 3.8)
project(aruco_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(visual_calibration_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS aruco calib3d)

add_executable(image_subscriber_node
  src/image_subscriber/image_subscriber_node.cpp
  src/image_subscriber/main.cpp
)
target_include_directories(image_subscriber_node PRIVATE
  include
)
ament_target_dependencies(image_subscriber_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

add_executable(aruco_detector_node
  src/aruco_detector/aruco_detector_node.cpp
  src/aruco_detector/main.cpp
)
target_include_directories(aruco_detector_node PRIVATE
  include
)
ament_target_dependencies(aruco_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  tf2
)
target_link_libraries(aruco_detector_node ${OpenCV_LIBS})

add_executable(calibration_broadcaster_node
  src/calibration_broadcaster/calibration_broadcaster_node.cpp
  src/calibration_broadcaster/orientation_averaging.cpp
  src/calibration_broadcaster/main.cpp
)
target_include_directories(calibration_broadcaster_node PRIVATE
  include
)
ament_target_dependencies(calibration_broadcaster_node
  rclcpp
  rclcpp_action
  geometry_msgs
  visual_calibration_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  image_subscriber_node
  aruco_detector_node
  calibration_broadcaster_node
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
