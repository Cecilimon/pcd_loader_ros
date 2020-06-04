cmake_minimum_required(VERSION 2.8.3)
project(pcd_loader_ros)

# -mavx causes a lot of errors!!
add_definitions(-std=c++11)
# add_definitions(-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
# set(CMAKE_CXX_FLAGS "-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(catkin REQUIRED COMPONENTS
  nodelet
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  roslaunch
)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  LIBRARIES hdl_scan_matching_odometry
#  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# nodelets
add_executable(pcd_map_server src/pcd_map_server.cpp)
target_link_libraries(pcd_map_server
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

install(DIRECTORY  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

