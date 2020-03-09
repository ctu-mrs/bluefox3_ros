cmake_minimum_required(VERSION 2.8.3)
project(bluefox3)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O0 -g")
set(CMAKE_CXX_FLAGS_DEBUG "-std=c++17 -Wall -O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++17 -Wall -O3")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-std=c++17 -Wall -O3 -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  image_transport
  camera_info_manager 
  sensor_msgs
  )

find_package(mvIMPACT_Acquire REQUIRED)

generate_dynamic_reconfigure_options(
  config/Bluefox3Params.cfg
  )

catkin_package(
  INCLUDE_DIRS include ${mvIMPACT_Acquire_INCLUDE_DIRS}
  LIBRARIES bluefox3 ${mvIMPACT_Acquire_LIBRARIES}
  CATKIN_DEPENDS camera_info_manager dynamic_reconfigure image_transport roscpp sensor_msgs
  )

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${mvIMPACT_Acquire_INCLUDE_DIRS}
  )

add_library(Bluefox3
  src/bluefox3.cpp
  )
add_dependencies(Bluefox3
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${mvIMPACT_Acquire_EXPORTED_TARGETS}
  )
target_link_libraries(Bluefox3
  ${catkin_LIBRARIES}
  ${mvIMPACT_Acquire_LIBRARIES}
  )

add_executable(list_cameras
  src/list_devices.cpp
  )
add_dependencies(list_cameras
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${mvIMPACT_Acquire_EXPORTED_TARGETS}
  )
target_link_libraries(list_cameras
  ${catkin_LIBRARIES}
  ${mvIMPACT_Acquire_LIBRARIES}
  )
