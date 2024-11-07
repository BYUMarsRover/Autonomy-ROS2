cmake_minimum_required(VERSION 3.8)
project(aruco_detect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
find_package(OpenCV REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${rclcpp_INCLUDE_DIRS}      # JM added this
  ${rover_msgs_INCLUDE_DIRS}  # JM added this
  ${OpenCV_INCLUDE_DIRS}
  ${ament_cmake_INCLUDE_DIRS}
  ${rover_msgs_INCLUDE_DIRS}  # Include rover_msgs directory
)

# Add executables
add_executable(aruco_detect src/aruco_detect.cpp)
add_executable(fiducials_node src/FiducialsNode.cpp)

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
target_link_libraries(aruco_detect ${OpenCV_LIBS})

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
target_link_libraries(fiducials_node ${OpenCV_LIBS})

install(TARGETS 
  aruco_detect
  fiducials_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS scripts/create_markers.py
  DESTINATION lib/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()

#   ament_add_gtest(aruco_images_test test/aruco_images_test.cpp)
#   if(aruco_images_test)
#     ament_target_dependencies(aruco_images_test rclcpp OpenCV)
#     target_link_libraries(aruco_images_test ${OpenCV_LIBS})

#     install(FILES test/aruco_images.test
#       DESTINATION share/${PROJECT_NAME}/test
#     )
#   endif()
# endif()

ament_package()
