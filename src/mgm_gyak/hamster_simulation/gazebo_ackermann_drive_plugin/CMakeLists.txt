cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ackermann_drive_plugin)

add_compile_options(-std=c++17)

find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ackermann_msgs
  gazebo_ros
  random_numbers
)

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#
# Build
#

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
    random_numbers
)

include_directories(
  include 
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


add_library(gazebo_ackermann_drive SHARED 
src/AckermannDrivePlugin.cpp
)

add_library(ackermann_kinematic 
src/ackermann_kinematic.cpp
)

target_link_libraries(ackermann_kinematic
  ${catkin_LIBRARIES}
)

target_link_libraries(gazebo_ackermann_drive ackermann_kinematic
  ${catkin_LIBRARIES}
)

install(TARGETS gazebo_ackermann_drive
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


