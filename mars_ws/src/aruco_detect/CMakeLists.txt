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
find_package(visualization_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include) # Adjust this if your headers are in a different directory

# Add the main executable
add_executable(aruco_detect src/aruco_detect.cpp) 
ament_target_dependencies(aruco_detect 
  rclcpp 
  rover_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs 
  visualization_msgs 
  image_transport 
  cv_bridge 
  sensor_msgs 
  std_msgs 
  std_srvs 
  OpenCV
)

# Install targets
install(TARGETS aruco_detect
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
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
