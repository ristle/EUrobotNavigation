cmake_minimum_required(VERSION 2.8.3)
project(local_planning)

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -O3 -g0")

#set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS_DEBUG}")
##############################################################################
# Find Packages
##############################################################################



find_package(catkin REQUIRED COMPONENTS
  rosmsg
  message_generation
  cost_map_core
  cost_map_ros
  nav_msgs
  std_msgs
  roscpp
)

# add_service_files(
#   FILES
#   LocalPlanner.srv
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
#   nav_msgs
#   cost_map_msgs
# )

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

add_executable(local_planner
src/local_planner.cpp
)
target_link_libraries(local_planner
  ${catkin_LIBRARIES}
)
