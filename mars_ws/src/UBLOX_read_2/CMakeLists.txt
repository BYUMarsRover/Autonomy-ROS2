cmake_minimum_required(VERSION 3.5)
project(ublox_read_2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Boost)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


add_subdirectory(lib/async_comm)
set(async_comm_LIBRARIES async_comm)
set(async_comm_INCLUDE_DIRS lib/async_comm/include;${Boost_INCLUDE_DIRS})


message("async_comm_INCLUDE_DIRS: ${async_comm_INCLUDE_DIRS}\n")

add_library(UBLOX SHARED
  src/ublox.cpp
  src/parsers/ubx_maps.cpp
  src/parsers/ubx.cpp
  src/parsers/rtcm.cpp
  src/parsers/nav.cpp
  src/utctime.cpp
)

message("UBLOX Boost include dirs: ${Boost_INCLUDE_DIRS}")
target_link_libraries(UBLOX ${async_comm_LIBRARIES} ${gnss_utils_LIBRARIES})
target_include_directories(UBLOX PUBLIC include ${async_comm_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

option(BUILD_ROS "Build ROS Node" ON)
if (${BUILD_ROS})
  message(STATUS "BUILDING UBLOX ROS PACKAGE")
  find_package(ament_cmake REQUIRED)
  find_package(rosbag2 REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/CfgValGetType.msg"
    "msg/Ephemeris.msg"
    "msg/GlonassEphemeris.msg"
    "msg/Observation.msg"
    "msg/ObsVec.msg"
    "msg/PositionVelocityTime.msg"
    "msg/PosVelEcef.msg"
    "msg/RelPos.msg"
    "msg/RelPosFlags.msg"
    "msg/RTCMInput.msg"
    "msg/Satellite.msg"
    "msg/SatelliteStatus.msg"
    "msg/SurveyStatus.msg"
    "srv/CfgValGet.srv"
    "srv/CfgValDel.srv"
    "srv/CfgValSet.srv"
    "srv/CfgReset.srv"
    "srv/CfgValGetAll.srv"
    "srv/GetVersion.srv"
    "srv/InitModule.srv"
    DEPENDENCIES builtin_interfaces std_msgs
  )

  ament_export_libraries(UBLOX)
  ament_export_include_directories(include)
  ament_export_dependencies(std_msgs rosidl_default_runtime)


  add_executable(ublox_ros src/ros/ublox_node.cpp src/ros/ublox_ros.cpp src/ros/callback_ros.cpp src/ros/services_ros.cpp src/ros/init_ros.cpp)
  ament_target_dependencies(ublox_ros rclcpp std_msgs sensor_msgs)

  rosidl_target_interfaces(ublox_ros ${PROJECT_NAME} "rosidl_typesupport_cpp")

  # ament_target_dependencies(ublox_ros UBLOX)
  target_link_libraries(ublox_ros UBLOX)
  target_include_directories(ublox_ros PUBLIC ../../devel/include)

  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
  )
  install(DIRECTORY
    params
    DESTINATION share/${PROJECT_NAME}/
  )

  install(TARGETS ublox_ros
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(
  TARGETS UBLOX async_comm
  DESTINATION lib/
)

ament_package()
