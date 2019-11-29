cmake_minimum_required(VERSION 2.8.3)
project(cost_map_server)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")


##############################################################################
# Find Packages
##############################################################################
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  cost_map_ros
  cost_map_visualisations
  nav_msgs
  message_generation
  roscpp
)
find_package(PkgConfig REQUIRED)


##############################################################################
# Catkin
##############################################################################

add_service_files(
  FILES
  GetCost.srv
  ClearLayer.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        geometry_msgs
        cost_map_ros
        cost_map_visualisations
        nav_msgs
        roscpp
    )



##############################################################################
# Project Configuration
##############################################################################

ecl_enable_cxx11_compiler()
add_definitions(${catkin_DEFINITIONS})
include_directories(include ${catkin_INCLUDE_DIRS})

###########
## Build ##
###########

add_executable(cost_map_server 
src/cost_map_server.cpp
include/cost_map_server.h 
)
target_link_libraries(cost_map_server 
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
