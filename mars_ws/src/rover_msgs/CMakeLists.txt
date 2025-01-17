cmake_minimum_required(VERSION 3.8)
project(rover_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AutonomyTaskInfo.msg"
  "msg/Battery.msg"
  "msg/Clicker.msg"
  "msg/Fiducial.msg"
  "msg/FiducialArray.msg"
  "msg/FiducialData.msg"
  "msg/FiducialTransform.msg"
  "msg/FiducialTransformArray.msg"
  "msg/Gripper.msg"
  "msg/MobilityDriveCommand.msg"
  "msg/Laser.msg"
  "msg/MobilityArucoAutopilotCommand.msg"
  "msg/MobilityAutopilotCommand.msg"
  "msg/MobilityGPSWaypoint2Follow.msg"
  "msg/MobilityVelocityCommands.msg"
  "msg/NavStatus.msg"
  "msg/ObjectDetection.msg"
  "msg/ObjectDetections.msg"
  "msg/ObstacleInfo.msg"
  "msg/PositionVelocityTime.msg"
  "msg/RawBattery.msg"
  "msg/RoverState.msg"
  "msg/RoverStateSingleton.msg"
  "msg/WaypointNav.msg"
  "msg/ZedObstacles.msg"
  "msg/Camera.msg"
  "msg/DeviceList.msg"
  "msg/ScienceFADIntensity.msg"
  "msg/Heartbeat.msg"
  "msg/HeartbeatStatusRover.msg"
  "msg/HeartbeatStatusBase.msg"
  "msg/ScienceAugerOn.msg"
  "msg/ScienceCacheDoor.msg"
  "msg/ScienceLinearActuatorDirection.msg"
  "msg/ScienceSaveNotes.msg"
  "msg/ScienceSaveSecondaryCache.msg"
  "msg/ScienceSaveSensor.msg"
  "msg/ScienceSecondaryCachePosition.msg"
  "msg/ScienceSensorValues.msg"
  "msg/ScienceSwitchTool.msg"
  "msg/ScienceToolPosition.msg"
  "msg/IWCMotors.msg"
  "msg/Elevator.msg"
  "msg/PointList.msg"
  "srv/PointList.srv"
  "srv/CameraControl.srv"
  "srv/CameraControlResponse.srv"
  "srv/AutonomyAbort.srv"
  "srv/AutonomyWaypoint.srv"
  "srv/AutonomyWaypointResponse.srv"
  "srv/SetFloat32.srv"
  "srv/PlanPath.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs builtin_interfaces # Add packages that above messages depend on
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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
