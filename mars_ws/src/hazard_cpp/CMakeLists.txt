cmake_minimum_required(VERSION 3.8)
project(hazard_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rover_msgs REQUIRED)

# Include directories
include_directories(
  ${PCL_INCLUDE_DIRS}
)

# Declare executable
add_executable(pcl_processor src/pcl_processor.cpp)

# Link dependencies
ament_target_dependencies(pcl_processor
  rclcpp
  sensor_msgs
  pcl_conversions
  visualization_msgs
  std_srvs
  rover_msgs
)

# Link PCL libraries
target_link_libraries(pcl_processor
  ${PCL_LIBRARIES}
)

# Install and export
install(TARGETS pcl_processor
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
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
