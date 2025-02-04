cmake_minimum_required(VERSION 3.5)
project(science)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scripts
ament_python_install_package(${PROJECT_NAME})

# Install resource files
install(
  DIRECTORY science/presentation/resources/
  DESTINATION share/${PROJECT_NAME}/presentation/resources
)

# Install launch files (if any)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()