cmake_minimum_required(VERSION 3.8)
project(aruco_detect)

# Set C++ standard (optional, adjust if needed)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) 
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)

# Include directories
include_directories(
  include  # Your local include directory
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Include directories (for OpenCV headers)
include_directories(${OpenCV_INCLUDE_DIRS})

# Add the FiducialsNode library
add_library(fiducials_node
  src/FiducialsNode.cpp
)

# Add the executable for aruco_detect
add_executable(aruco_detect
  src/aruco_detect.cpp
)


# Link OpenCV libraries to the executable
target_link_libraries(aruco_detect ${OpenCV_LIBS})

# Include OpenCV header files
target_include_directories(aruco_detect PRIVATE ${OpenCV_INCLUDE_DIRS})

# Link dependencies for fiducials_node
ament_target_dependencies(fiducials_node
  rclcpp
  rover_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  sensor_msgs
  std_srvs
  std_msgs
  cv_bridge
  image_transport
)
target_link_libraries(fiducials_node
  ${OpenCV_LIBS}
)



# Link dependencies for aruco_detect
ament_target_dependencies(aruco_detect
  rclcpp
  rover_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  sensor_msgs
  std_srvs
  std_msgs
  cv_bridge
  image_transport
)
target_link_libraries(aruco_detect
  fiducials_node
  ${OpenCV_LIBS}
)

# Install targets
install(TARGETS 
  aruco_detect
  fiducials_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install scripts
install(PROGRAMS scripts/create_markers.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
