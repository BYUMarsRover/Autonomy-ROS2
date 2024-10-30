cmake_minimum_required(VERSION 3.8)
project(aruco_detect)

# Check for compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) 
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(OpenCV REQUIRED)
<<<<<<< HEAD

# Include directories
include_directories(include) # Adjust this if your headers are in a different directory

# Add the executable
add_executable(aruco_detect src/aruco_detect.cpp) 
=======
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Add executable for the main node
add_executable(aruco_detect src/aruco_detect.cpp)

# Link libraries and dependencies
>>>>>>> Aruco-Detect-Parent
ament_target_dependencies(aruco_detect 
  rclcpp 
  rover_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs 
  visualization_msgs 
<<<<<<< HEAD
  image_transport 
  cv_bridge 
  sensor_msgs 
  std_msgs 
  std_srvs 
  OpenCV
)

# Install targets
=======
  sensor_msgs 
  std_srvs 
  std_msgs 
  OpenCV
  cv_bridge
  image_transport
)

# Link OpenCV libraries
target_link_libraries(aruco_detect ${OpenCV_LIBS})

#############
## Install ##
#############

# Mark the executable for installation
>>>>>>> Aruco-Detect-Parent
install(TARGETS aruco_detect
  DESTINATION lib/${PROJECT_NAME}
)

<<<<<<< HEAD
install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
=======
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
  find_package(ament_lint_auto REQUIRED)
>>>>>>> Aruco-Detect-Parent
  ament_lint_auto_find_test_dependencies()

  # Add the C++ test executable
  add_executable(aruco_images_test test/aruco_images_test.cpp)
  ament_target_dependencies(aruco_images_test rclcpp OpenCV)

  # Link the test executable
  target_link_libraries(aruco_images_test ${OpenCV_LIBS})

  # Add a gtest for this test file
  ament_add_gtest(aruco_images_test test/aruco_images_test.cpp)

  # Install test files
  install(FILES test/aruco_images.test
    DESTINATION share/${PROJECT_NAME}/test
  )
endif()

# ROS2 package
ament_package()
