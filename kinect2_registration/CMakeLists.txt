cmake_minimum_required(VERSION 3.5)
project(kinect2_registration C CXX)

cmake_policy(SET CMP0057 NEW)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBT_USE_DOUBLE_PRECISION -Wall")
# Unused warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wuninitialized -Winit-self -Wunused-function -Wunused-label -Wunused-variable -Wunused-but-set-variable -Wunused-but-set-parameter")
# Additional warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Warray-bounds -Wtype-limits -Wreturn-type -Wsequence-point -Wparentheses -Wmissing-braces -Wchar-subscripts -Wswitch -Wwrite-strings -Wenum-compare -Wempty-body")# -Wlogical-op")

# Check for c++17 support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
IF(COMPILER_SUPPORTS_CXX17)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
ENDIF()

# additional cmake modules
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# enable ament exports
find_package(ament_cmake REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Eigen3)
#find_package(OpenCL) # TODO
find_package(OpenCV REQUIRED)
find_package(OpenMP)

# ROS 2 is used for logging
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)

if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

############################################################
## Check for modules being build based on found libraries ##
############################################################

if(EIGEN3_FOUND)
  message(STATUS "CPU based depth registration enabled")
  include_directories(${EIGEN3_INCLUDE_DIR})
  set(DEPTH_REG_CPU ON)
  add_definitions(-DDEPTH_REG_CPU)
else()
  message(STATUS "CPU based depth registration disabled")
  set(DEPTH_REG_CPU OFF)
endif()

if(OpenCL_FOUND)
  message(STATUS "OpenCL based depth registration enabled")
  set(EXPORTED_DEPENDENCIES OpenCL)

  if(UNIX AND NOT APPLE)
    include(CheckOpenCLICDLoader)
    if(OpenCL_C_WORKS AND NOT OpenCL_CXX_WORKS)
      set(KINECT2_OPENCL_ICD_LOADER_IS_OLD 1)
      message(WARNING "Your libOpenCL.so is incompatible with CL/cl.h. Install ocl-icd-opencl-dev to update libOpenCL.so?")
    endif()
  endif()
  include_directories(${OpenCL_INCLUDE_DIRS})

  set(DEPTH_REG_OPENCL ON)
  add_definitions(-DDEPTH_REG_OPENCL)
else()
  message(STATUS "OpenCL based depth registration disabled")
  set(DEPTH_REG_OPENCL OFF)
endif()

if(NOT DEPTH_REG_CPU AND NOT DEPTH_REG_OPENCL)
  message(FATAL_ERROR "No registration method available!")
endif()

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
)

include_directories(include
  ${OpenCV_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rosidl_typesupport_interface_INCLUDE_DIRS}
  ${rmw_INCLUDE_DIRS}
  "${CMAKE_CURRENT_SOURCE_DIR}/include/internal"
)

if(DEPTH_REG_CPU)
  set(MODULES ${MODULES} src/depth_registration_cpu.cpp)
endif()

if(DEPTH_REG_OPENCL)
  add_definitions(-DREG_OPENCL_FILE="${PROJECT_SOURCE_DIR}/src/depth_registration.cl")
  set(MODULES ${MODULES} src/depth_registration_opencl.cpp)
  set(MODULE_LIBS ${MODULE_LIBS} ${OpenCL_LIBRARIES})
endif()

add_library(kinect2_registration SHARED src/kinect2_registration.cpp ${MODULES})
target_link_libraries(kinect2_registration
  ${OpenCV_LIBRARIES}
  ${MODULE_LIBS}
)

# Namespaced target for modern CMake
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS kinect2_registration)

# Add ROS 2 dependencies
ament_target_dependencies(kinect2_registration
  rclcpp
  rcutils
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS kinect2_registration
  EXPORT ${PROJECT_NAME}Targets
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

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenCV rclcpp Eigen3)

##############################
## Declare an ament Package ##
##############################

ament_package()
