cmake_minimum_required(VERSION 2.8.3)
project(sblox)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14 -Wall -Wextra)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/segment_server.cc
)

############
# BINARIES #
############

cs_add_executable(segment_server
  src/segment_server_node.cc
)
target_link_libraries(segment_server ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
