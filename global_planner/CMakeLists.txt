cmake_minimum_required(VERSION 2.8.3)
project(global_planner)

# set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS_DEBUG}")
##############################################################################
# Find Packages
##############################################################################
find_package(catkin REQUIRED COMPONENTS
  rosmsg  
  message_generation
  cost_map_ros
  cost_map_visualisations
  nav_msgs
  std_msgs
  roscpp
  )

add_service_files(
  FILES
  Enable.srv
  Goal.srv
  Mode.srv
  PathCost.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

##############################################################################
# Catkin
##############################################################################



catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
  # LIBRARIES
  # ${PROJECT_NAME}
  # costmap_2d
  # cost_map_ros
  # cost_map_visualisations
  # nav_msgs
  # roscpp
)

##############################################################################
# Project Configuration
##############################################################################

include_directories(include ${catkin_INCLUDE_DIRS})

###########
## Build ##
###########

add_executable(global_planner 
src/path_planner.cpp 
src/path_simplifier.cpp
src/global_planner.cpp
src/develop.cpp
)
target_link_libraries(global_planner 
  ${catkin_LIBRARIES}
)
add_executable(mover 
src/mover.cpp
)
target_link_libraries(mover
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Copy launch files
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#   FILES_MATCHING PATTERN "*.launch"
# )
