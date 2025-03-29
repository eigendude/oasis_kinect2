cmake_minimum_required(VERSION 3.5)
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

################################################################################
# Dependencies
################################################################################

find_package(ament_cmake REQUIRED)
find_package(compressed_depth_image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3)
find_package(freenect2 REQUIRED HINTS "$ENV{HOME}/freenect2")
find_package(kinect2_registration REQUIRED)
find_package(LibUSB REQUIRED) # TODO: This should come from libfreenect2
find_package(OpenCL)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(TurboJPEG REQUIRED) # TODO: This should come from libfreenect2

################################################################################
# Soures
################################################################################

set(KINECT2_BRIDGE_SOURCES
  src/bridge/kinect2_bridge.cpp
  src/utils/time_utils.cpp
)

################################################################################
# Libraries
################################################################################

set(KINECT2_BRIDGE_LIBRARIES
  ${compressed_depth_image_transport_LIBRARY}
  ${cv_bridge_LIBRARY}
  ${freenect2_LIBRARY}
  ${kinect2_registration_LIBRARIES}
  ${LibUSB_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${rclcpp_LIBRARY}
  ${rcutils_LIBRARY}
  ${sensor_msgs_LIBRARY}
  ${std_msgs_LIBRARY}
  ${tf2_LIBRARY}
  ${TurboJPEG_LIBRARIES}
  pthread
)

################################################################################
# Ament dependencies
################################################################################

set(KINECT2_BRIDGE_AMENT_DEPENDS
  compressed_depth_image_transport
  cv_bridge
  kinect2_registration
  rclcpp
  rcutils
  sensor_msgs
  std_msgs
  tf2
)

################################################################################
# Build Configuration
################################################################################

if(OpenCL_FOUND)
  add_definitions(-DDEPTH_REG_OPENCL)
endif()
if(EIGEN3_FOUND)
  add_definitions(-DDEPTH_REG_CPU)
endif()

# TODO: Modify path for target access
add_definitions(-DK2_CALIB_PATH="${PROJECT_SOURCE_DIR}/data/")

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${compressed_depth_image_transport_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${freenect2_INCLUDE_DIRS}
  ${kinect2_registration_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rcutils_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
)

################################################################################
# Nodes
################################################################################

#
# kinect2_bridge node
#

add_executable(kinect2_bridge
  ${KINECT2_BRIDGE_SOURCES}
  src/cli/kinect2_bridge_node.cpp
)
target_link_libraries(kinect2_bridge
  ${KINECT2_BRIDGE_LIBRARIES}
)
ament_target_dependencies(kinect2_bridge
  ${KINECT2_BRIDGE_AMENT_DEPENDS}
)

################################################################################
# Components
################################################################################

#
# kinect2_bridge component
#

add_library(kinect2_bridge_component SHARED
  ${KINECT2_BRIDGE_SOURCES}
  src/components/kinect2_bridge_component.cpp
)
target_link_libraries(kinect2_bridge_component
  ${KINECT2_BRIDGE_LIBRARIES}
)
ament_target_dependencies(kinect2_bridge_component
  ${KINECT2_BRIDGE_AMENT_DEPENDS}
  rclcpp_components
)

# Register the component
rclcpp_components_register_node(
  kinect2_bridge_component
  PLUGIN "kinect2_bridge::Kinect2BridgeComponent"
  EXECUTABLE kinect2_bridge_component_node
)

################################################################################
# Install
################################################################################

# Mark executables and/or libraries for installation
install(
  TARGETS
    kinect2_bridge
    kinect2_bridge_component
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Install data files
install(DIRECTORY
  data
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Testing
################################################################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Copyright and license must be present in all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # The following line skips cpplint (only works in a git repo)
  # Uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Declare an ament Package
################################################################################

# Export the component so that the plugin XML is generated
ament_export_dependencies(rclcpp_components)

ament_package()
