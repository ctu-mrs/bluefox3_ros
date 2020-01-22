cmake_minimum_required(VERSION 2.8.3)
project(bluefox3)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O0 -g -fvar-tracking -fvar-tracking-assignments -DEIGEN_INITIALIZE_MATRICES_BY_NAN")
set(CMAKE_CXX_FLAGS_DEBUG "-std=c++17 -Wall -O0 -g -fvar-tracking -fvar-tracking-assignments -DEIGEN_INITIALIZE_MATRICES_BY_NAN")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++17 -Wall -O0 -g -fvar-tracking -fvar-tracking-assignments -DEIGEN_INITIALIZE_MATRICES_BY_NAN")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-std=c++17 -Wall -O0 -g -fvar-tracking -fvar-tracking-assignments -DEIGEN_INITIALIZE_MATRICES_BY_NAN")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  image_transport
  camera_info_manager 
  sensor_msgs
  mrs_lib
  )

find_package(mvIMPACT_Acquire REQUIRED)

generate_dynamic_reconfigure_options(
  config/Bluefox3Params.cfg
  )

catkin_package(
  INCLUDE_DIRS include ${mvIMPACT_Acquire_INCLUDE_DIRS}
  LIBRARIES bluefox3 ${mvIMPACT_Acquire_LIBRARIES}
  CATKIN_DEPENDS camera_info_manager dynamic_reconfigure image_transport roscpp sensor_msgs mrs_lib
  #  DEPENDS system_lib
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${mvIMPACT_Acquire_INCLUDE_DIRS}
  )

## Declare a C++ library
add_library(Bluefox3
  src/bluefox3.cpp
  )
add_dependencies(Bluefox3
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${mvIMPACT_Acquire_EXPORTED_TARGETS}
  )

## Declare a C++ executable
add_executable(bluefox3_list_cameras
  src/list_devices.cpp
  )
add_dependencies(bluefox3_list_cameras
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${mvIMPACT_Acquire_EXPORTED_TARGETS}
  )
## Specify libraries to link a library or executable target against
target_link_libraries(bluefox3_list_cameras
  ${catkin_LIBRARIES}
  ${mvIMPACT_Acquire_LIBRARIES}
  )
