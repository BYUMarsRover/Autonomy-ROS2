cmake_minimum_required(VERSION 3.8)
project(aruco_detect)

# Check for compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ament_Cmake first: 
find_package(ament_cmake REQUIRED)

# Find dependencies
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(fiducial_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) 
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Add executable for the main node
add_executable(aruco_detect src/aruco_detect.cpp)
add_executable(fiducials_node src/FiducialsNode.cpp)

# Link libraries and dependencies
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
  # fiducial_msgs
)

# Link OpenCV libraries
target_link_libraries(aruco_detect ${OpenCV_LIBS})

#############
## Install ##
#############

# Mark the executable for installation
install(TARGETS 
  aruco_detect
  fiducials_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install Python scripts if necessary
install(PROGRAMS scripts/create_markers.py
  DESTINATION lib/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED) 
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Add the C++ test executable with gtest
  ament_add_gtest(aruco_images_test test/aruco_images_test.cpp)
  if(aruco_images_test) # Check if the gtest executable was created successfully
    ament_target_dependencies(aruco_images_test rclcpp OpenCV)

    # Link the test executable with OpenCV
    target_link_libraries(aruco_images_test ${OpenCV_LIBS})

    # Install test files
    install(FILES test/aruco_images.test
      DESTINATION share/${PROJECT_NAME}/test
    )
  endif()
endif()

# ROS2 package
ament_package()
