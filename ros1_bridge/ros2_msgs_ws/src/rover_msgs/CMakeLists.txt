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

install(
  FILES mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AntennaStatus.msg"
  "msg/Arm.msg"
  "msg/ArmFeedback.msg"
  "msg/ArmHardware.msg"
  "msg/ArmHardwareFeedback.msg"
  "msg/ArmPosition.msg"
  "msg/ArmSetPosition.msg"
  "msg/ArmState.msg"
  "msg/AutonomyTaskInfo.msg"
  "msg/BatStatGripElevArduino.msg"
  "msg/Battery.msg"
  "msg/Camera.msg"
  "msg/CameraObjectList.msg"
  "msg/CameraSetValues.msg"
  "msg/CameraValue.msg"
  "msg/Clicker.msg"
  "msg/ConnectionInfo.msg"
  "msg/DeviceList.msg"
  "msg/DynamixelState.msg"
  "msg/Elevator.msg"
  "msg/Feedback.msg"
  "msg/Fiducial.msg"
  "msg/FiducialArray.msg"
  "msg/FiducialData.msg"
  "msg/FiducialTransform.msg"
  "msg/FiducialTransformArray.msg"
  "msg/FPVServo.msg"
  "msg/Gripper.msg"
  "msg/Heading.msg"
  "msg/Heartbeat.msg"
  "msg/HeartbeatStatusBase.msg"
  "msg/HeartbeatStatusRover.msg"
  "msg/IWC_feedback.msg"
  "msg/IWC_motors.msg"
  "msg/IWCMotors.msg"
  "msg/KeyboardHomography.msg"
  "msg/KeyLocations.msg"
  "msg/Laser.msg"
  "msg/MobilityArucoAutopilotCommand.msg"
  "msg/MobilityAutopilotCommand.msg"
  "msg/MobilityDriveCommands.msg"
  "msg/MobilityDriveControl.msg"
  "msg/MobilityDriveSpeedMultiplier.msg"
  "msg/MobilityDriveState.msg"
  "msg/MobilityGPSWaypoint2Follow.msg"
  "msg/MobilityIWCFeedback.msg"
  "msg/MobilityIWCMotors.msg"
  "msg/MobilityJoyDriveEnable.msg"
  "msg/MobilityVelocityCommands.msg"
  "msg/MobilityWaypoint2Follow.msg"
  "msg/NavStatus.msg"
  "msg/ObjectDetection.msg"
  "msg/ObjectDetections.msg"
  "msg/ObstacleInfo.msg"
  "msg/Orientation.msg"
  "msg/PathPlanningWaypoints.msg"
  "msg/RawBattery.msg"
  "msg/RoverState.msg"
  "msg/RoverStateSingleton.msg"
  "msg/ScienceAugerOn.msg"
  "msg/ScienceCacheDoor.msg"
  "msg/ScienceFADIntensity.msg"
  "msg/ScienceLinearActuatorDirection.msg"
  "msg/ScienceSaveNotes.msg"
  "msg/ScienceSaveSecondaryCache.msg"
  "msg/ScienceSaveSensor.msg"
  "msg/ScienceSecondaryCachePosition.msg"
  "msg/ScienceSensorValues.msg"
  "msg/ScienceSwitchTool.msg"
  "msg/ScienceToolPosition.msg"
  "msg/Temperature.msg"
  "msg/WallObstacles.msg"
  "msg/WaypointNav.msg"
  "msg/Xbox.msg"
  "msg/ZedObstacles.msg"
  "srv/AutonomyAbort.srv"
  "srv/AutonomyWaypoint.srv"
  "srv/AutonomyWaypointResponse.srv"
  "srv/CameraControl.srv"
  "srv/CameraControlResponse.srv"
  "srv/CameraNamesList.srv"
  "srv/CameraObjectList.srv"
  "srv/DebugMessages.srv"
  "srv/DistHeadingConv.srv"
  "srv/GimbalCommand.srv"
  "srv/GuiCommand.srv"
  "srv/KeyPress.srv"
  "srv/LaunchCamera.srv"
  "srv/LLToNE.srv"
  "srv/PositionReturn.srv"
  "srv/Screenshot.srv"
  "srv/SetFloat32.srv"
  "srv/WaypointSend.srv"
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
