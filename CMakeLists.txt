cmake_minimum_required(VERSION 2.8.3)
project(waypoint_saver)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11 -Wall -Wextra)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/waypoint_saver_node.cpp
        )

############
# BINARIES #
############
cs_add_executable(waypoint_saver_node
        src/waypoint_saver_node.cpp
        )
target_link_libraries(waypoint_saver_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
