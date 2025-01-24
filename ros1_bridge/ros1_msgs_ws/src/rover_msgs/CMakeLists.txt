cmake_minimum_required(VERSION 2.8.3)
project(rover_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED std_msgs message_generation sensor_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   AntennaStatus.msg
   Arm.msg
   ArmFeedback.msg
   ArmHardware.msg
   ArmHardwareFeedback.msg
   ArmPosition.msg
   ArmSetPosition.msg
   ArmState.msg
   AutonomyTaskInfo.msg
   Battery.msg
   BatStatGripElevArduino.msg
   Camera.msg
   CameraSetValues.msg
   CameraValue.msg
   Clicker.msg
   Laser.msg
   ConnectionInfo.msg
   DeviceList.msg
   DynamixelState.msg
   Feedback.msg
   Fiducial.msg
   FiducialArray.msg
   FiducialData.msg
   FiducialTransform.msg
   FiducialTransformArray.msg
   Heading.msg
   Heartbeat.msg
   HeartbeatStatusBase.msg
   HeartbeatStatusRover.msg
   MobilityArucoAutopilotCommand.msg
   MobilityAutopilotCommand.msg
   MobilityDriveControl.msg
   MobilityDriveState.msg
   MobilityIWCFeedback.msg
   MobilityDriveCommands.msg
   IWC_feedback.msg
   IWC_motors.msg
   IWCMotors.msg
   KeyboardHomography.msg
   KeyLocations.msg
   MobilityJoyDriveEnable.msg
   MobilityVelocityCommands.msg
   MobilityWaypoint2Follow.msg
   MobilityGPSWaypoint2Follow.msg
   NavStatus.msg
   ObjectDetection.msg
   ObjectDetections.msg
   ObstacleInfo.msg
   PathPlanningWaypoints.msg
   RawBattery.msg
   RoverState.msg
   ScienceAugerOn.msg
   ScienceCacheDoor.msg
   ScienceFADIntensity.msg
   ScienceLinearActuatorDirection.msg
   ScienceSaveNotes.msg
   ScienceSaveSecondaryCache.msg
   ScienceSaveSensor.msg
   ScienceSecondaryCachePosition.msg
   ScienceSensorValues.msg
   ScienceSwitchTool.msg
   ScienceToolPosition.msg
   Temperature.msg
   WallObstacles.msg
   WaypointNav.msg
   Xbox.msg
   ZedObstacles.msg
   Gripper.msg
   Elevator.msg
   ArmPosition.msg
   Orientation.msg
   RoverStateSingleton.msg
   FPVServo.msg
 )

# Generate services in the 'srv' folder
add_service_files(
   FILES
   AutonomyAbort.srv
   AutonomyWaypoint.srv
   CameraControl.srv
   CameraNamesList.srv
   CameraObjectList.srv
   DebugMessages.srv
   DistHeadingConv.srv
   GimbalCommand.srv
   GuiCommand.srv
   KeyPress.srv
   LaunchCamera.srv
   LLToNE.srv
   PositionReturn.srv
   Screenshot.srv
   SetFloat32.srv
   WaypointSend.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rover_msgs
  CATKIN_DEPENDS sensor_msgs
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
 include_directories(
  ${catkin_INCLUDE_DIRS}
 )
## Declare a C++ library
# add_library(rover_msgs
#   src/${PROJECT_NAME}/rover_msgs.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(rover_msgs ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(rover_msgs_node src/rover_msgs_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(rover_msgs_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(rover_msgs_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS rover_msgs rover_msgs_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rover_msgs.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
