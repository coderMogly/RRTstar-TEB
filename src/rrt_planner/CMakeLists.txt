cmake_minimum_required(VERSION 2.8.3)
project(rrt_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    COMPONENTS
    angles
    costmap_2d
#    dynamic_reconfigure, base_local_planner
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf
    base_local_planner
    rospy
    move_base_msgs
    std_msgs
)

find_package(OMPL)
##If Eigen is needed
#add_definitions(${EIGEN_DEFINITIONS})
#FIND_PACKAGE(Eigen REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS}

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rrt_planner
  CATKIN_DEPENDS 
    costmap_2d
#    dynamic_reconfigure, base_local_planner
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf
    base_local_planner
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(rrt_planner ~/glocal/src/rrt_planner/src/rrt_planner.cpp)
# add_library(a
#   src/${PROJECT_NAME}/a.cpp
# )

target_link_libraries(rrt_planner ${catkin_LIBRARIES})
target_link_libraries(rrt_planner ${OMPL_LIBRARIES})

install(TARGETS rrt_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(rrt_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(rrt_planner_node src/rrt_planner_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(rrt_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(rrt_planner_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS rrt_planner rrt_planner_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rrt_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
