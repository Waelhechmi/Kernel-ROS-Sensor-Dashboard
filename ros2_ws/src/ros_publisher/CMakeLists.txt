cmake_minimum_required(VERSION 3.5)
project(ros_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(sensor_publisher
  src/sensor_publisher.cpp
  src/RawSensorsReader.cpp
)

ament_target_dependencies(sensor_publisher
  rclcpp
  std_msgs
)

install(TARGETS sensor_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
