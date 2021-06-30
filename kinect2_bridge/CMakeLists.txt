cmake_minimum_required(VERSION 2.8.3)
project(kinect2_bridge C CXX)

cmake_policy(SET CMP0057 NEW)

# Additional cmake modules
LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBT_USE_DOUBLE_PRECISION -Wall")
# Unused warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wuninitialized -Winit-self -Wunused-function -Wunused-label -Wunused-variable -Wunused-but-set-variable -Wunused-but-set-parameter")
# Additional warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Warray-bounds -Wtype-limits -Wreturn-type -Wsequence-point -Wparentheses -Wmissing-braces -Wchar-subscripts -Wswitch -Wwrite-strings -Wenum-compare -Wempty-body -Wlogical-op")

# Check for C++17 support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
IF(COMPILER_SUPPORTS_CXX17)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
ENDIF()

find_package(freenect2 REQUIRED HINTS "$ENV{HOME}/freenect2")

find_package(ament_cmake REQUIRED)
find_package(compressed_depth_image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(kinect2_registration REQUIRED)
find_package(LibUSB REQUIRED) # TODO: This should come from libfreenect2
#find_package(nodelet REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rcl_interfaces REQUIRED) # TODO
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED) # TODO
find_package(rmw REQUIRED) # TODO
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(TurboJPEG REQUIRED) # TODO: This should come from libfreenect2

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

###########
## Build ##
###########

# Display additional files in qtcreator
execute_process(COMMAND find ${PROJECT_SOURCE_DIR} -type f OUTPUT_VARIABLE FILES_STRING)
string(REPLACE "\n" ";" FILES_LIST ${FILES_STRING})
add_custom_target(additional_files_${PROJECT_NAME}
  SOURCES
  ${FILES_LIST}
  ${PROJECT_SOURCE_DIR}/../README.md
)

include_directories(include
  ${OpenCV_INCLUDE_DIRS}
  ${freenect2_INCLUDE_DIRS}
  ${kinect2_registration_INCLUDE_DIRS}
  # TODO
  ${kinect2_registration_DIR}/../../../include
  ${compressed_depth_image_transport_INCLUDE_DIRS}
  # TODO
  ${compressed_depth_image_transport_DIR}/../../../include
  ${cv_bridge_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS} # TODO
  ${rclcpp_INCLUDE_DIRS}
  ${rcutils_INCLUDE_DIRS}
  ${rmw_INCLUDE_DIRS} # TODO
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
)

if(DEPTH_REG_OPENCL)
  add_definitions(-DDEPTH_REG_OPENCL)
endif()
if(DEPTH_REG_CPU)
  add_definitions(-DDEPTH_REG_CPU)
endif()
add_definitions(-DK2_CALIB_PATH="${PROJECT_SOURCE_DIR}/data/")

add_library(kinect2_bridge_nodelet SHARED
  src/utils/time_utils.cpp
  src/kinect2_bridge.cpp
)
target_link_libraries(kinect2_bridge_nodelet
  ${OpenCV_LIBRARIES}
  ${freenect2_LIBRARY}
  ${kinect2_registration_LIBRARY}
  ${LibUSB_LIBRARIES}
  ${compressed_depth_image_transport_LIBRARY}
  ${cv_bridge_LIBRARY}
  ${rclcpp_LIBRARY}
  ${rcutils_LIBRARY}
  ${sensor_msgs_LIBRARY}
  ${std_msgs_LIBRARY}
  ${tf2_LIBRARY}
  ${TurboJPEG_LIBRARIES}
  pthread
)
ament_target_dependencies(kinect2_bridge_nodelet
  compressed_depth_image_transport
  cv_bridge
  kinect2_registration
  #nodelet
  rclcpp
  rcutils
  sensor_msgs
  std_msgs
  tf2
)

add_executable(kinect2_bridge
  src/utils/time_utils.cpp
  src/kinect2_bridge.cpp
)
target_link_libraries(kinect2_bridge
  ${OpenCV_LIBRARIES}
  ${freenect2_LIBRARY}
  ${kinect2_registration_LIBRARY}
  # TODO
  ${kinect2_registration_DIR}/../../../lib/kinect2_registration/libkinect2_registration.so
  ${LibUSB_LIBRARIES}
  ${compressed_depth_image_transport_LIBRARY}
  ${cv_bridge_LIBRARY}
  ${rclcpp_LIBRARY}
  ${rcutils_LIBRARY}
  ${sensor_msgs_LIBRARY}
  ${std_msgs_LIBRARY}
  ${tf2_LIBRARY}
  ${TurboJPEG_LIBRARIES}
  pthread
)
ament_target_dependencies(kinect2_bridge
  compressed_depth_image_transport
  cv_bridge
  kinect2_registration
  #nodelet
  #rcl_interfaces
  rclcpp
  rcutils
  #rosidl_typesupport_interface # TODO
  #rmw # TODO
  sensor_msgs
  std_msgs
  tf2
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION bin/${PROJECT_NAME}
# )

## Mark executables and/or libraries for installation
install(TARGETS kinect2_bridge kinect2_bridge_nodelet
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  # myfile2
  DESTINATION share/${PROJECT_NAME}
)
install(FILES
  nodelet_plugins.xml
  # myfile2
  DESTINATION share/${PROJECT_NAME}
)

# Install data files
install(DIRECTORY
  data
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Copyright and license must be present in all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # The following line skips cpplint (only works in a git repo)
  # Uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

##############################
## Declare an ament Package ##
##############################

ament_package()
