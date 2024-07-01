cmake_minimum_required(VERSION 3.5)
project(robocamp_rosbot)

set(CMAKE_PREFIX_PATH "/usr/local" ${CMAKE_PREFIX_PATH})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(depthai REQUIRED)

include_directories(
  include
)

add_executable(robot_controller_node src/robot_controller_node.cpp)
ament_target_dependencies(robot_controller_node rclcpp sensor_msgs geometry_msgs)

add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node rclcpp sensor_msgs cv_bridge image_transport)

add_executable(motor_driver_node src/motor_driver_node.cpp)
ament_target_dependencies(motor_driver_node rclcpp geometry_msgs nav_msgs)

target_link_libraries(camera_node
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  robot_controller_node
  camera_node
  motor_driver_node
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/sensor_node.py
  scripts/color_sensor_node.py
  scripts/oak_d_lite_node.py
  ../ldlidar_stl_ros2/launch/ld19.launch.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()