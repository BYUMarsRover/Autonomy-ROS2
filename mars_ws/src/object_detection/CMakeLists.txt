cmake_minimum_required(VERSION 3.8)
project(object_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

SET(SPECIAL_OS_LIBS "")

option(ENABLE_INT8_CALIBRATOR "Enable int8 calibrator class for quantification calibration, requires OpenCV DNN" OFF)

if (ENABLE_INT8_CALIBRATOR)
    add_definitions(-DENABLE_INT8_CALIBRATOR)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED) #DO WE STILL NEED CV _ BRIDGE 2?
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(TensorRT REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
find_package(ZED 4 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

# Install other files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})


# INCLUDES and LIBS
include_directories(
  ${CUDA_INCLUDE_DIRS}
  ${ZED_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
include_directories(include)
include_directories(${TensorRT_INCLUDE_DIRS})

# # Find source files
file(GLOB_RECURSE SRC_FILES src/*.cpp)
file(GLOB_RECURSE HDR_FILES include/*.h)

# Add executable
# add_executable(${PROJECT_NAME}_node ${SRC_FILES} ${HDR_FILES})
add_executable(object_detection src/main.cpp src/yolo.cpp)
target_include_directories(object_detection PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Set compile definitions and optimizations
add_definitions(-O3 -D_MWAITXINTRIN_H_INCLUDED -Wno-deprecated-declarations)

# Link libraries
if (LINK_SHARED_ZED)
set(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
set(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()

set(TRT_LIBS nvinfer)

target_link_libraries(object_detection
  ${TRT_LIBS}
  ${SPECIAL_OS_LIBS}
  ${ZED_LIBS}
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(object_detection
  rclcpp
  cv_bridge
  rover_msgs
  image_transport
  sensor_msgs
)

# Install targets
install(TARGETS object_detection
  DESTINATION lib/${PROJECT_NAME})


install(
  DIRECTORY params
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY include/
  DESTINATION include
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


# ament_export_dependencies(rclcpp cv_bridge rover_msgs image_transport sensor_msgs)
# ament_export_include_directories(include)

ament_package()
