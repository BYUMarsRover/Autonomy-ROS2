cmake_minimum_required(VERSION 3.5)
project(heartbeat)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
# find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# install Python executables
install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
)

# install launch file
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "../rover_msg/Heartbeat.msg"
  "../rover_msg/HeartbeatStatusRover.msg"
  "../rover_msg/HeartbeatStatusBase.msg"
  DEPENDENCIES std_msgs
)

ament_package()
