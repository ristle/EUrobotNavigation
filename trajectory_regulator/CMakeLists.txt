cmake_minimum_required(VERSION 2.8.3)
project(trajectory_regulator)

# set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -fopenmp")
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS_DEBUG} -fopenmp")
##############################################################################
# Find Packages
##############################################################################
find_package(catkin REQUIRED COMPONENTS
        rosmsg
        # message_generation
        grid_map_ros
        cost_map_core
        cost_map_ros
        nav_msgs
        std_msgs
        roscpp
        global_planner
        )


##############################################################################
# Catkin
##############################################################################

# add_service_files(
#   FILES
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
#   nav_msgs
# )


catkin_package(
        CATKIN_DEPENDS roscpp std_msgs global_planner
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

add_executable(trajectory_regulator
        src/trajectory_regulator.cpp
        )
target_link_libraries(trajectory_regulator
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
