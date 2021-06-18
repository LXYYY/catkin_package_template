cmake_minimum_required(VERSION 2.8.3)
project(template)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14 -Wall -Wextra)

#############
# LIBRARIES #
#############
#cs_add_library(${PROJECT_NAME}
#        src/template.cc
#)

############
# BINARIES #
############

cs_add_executable(template
  src/template_node.cc
)
target_link_libraries(template ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
