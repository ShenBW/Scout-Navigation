cmake_minimum_required(VERSION 3.0.2)
project(scout_navigation)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
  tf2_ros
  tf2_geometry_msgs
  roscpp
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(INCLUDE_DIRS include)

# include directories
include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
)

# link directories
link_directories(
  include
)

add_library(slam_odom_library
  src/slam_odom.cpp
)

target_link_libraries(slam_odom_library
  ${catkin_LIBRARIES} 
)

# Slam Odom
add_executable(slam_odom_node src/slam_odom_node.cpp)
add_dependencies(slam_odom_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(slam_odom_node ${catkin_LIBRARIES} slam_odom_library)

roslaunch_add_file_check(launch)

#############
## Install ##
#############
## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_m_navigation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
