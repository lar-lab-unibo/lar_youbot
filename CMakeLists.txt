cmake_minimum_required(VERSION 2.8.3)
project(lar_youbot)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp     
  actionlib  
  control_msgs
  trajectory_msgs  
  brics_actuator 
)

#KDL
find_package(orocos_kdl REQUIRED)

#EIGEN
find_package( Eigen3 REQUIRED )
include_directories( /usr/include/eigen3 )

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES lar_youbot
 CATKIN_DEPENDS joy roscpp actionlib control_msgs trajectory_msgs brics_actuator
 DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#YOUBOT LIBRARY
include_directories(
    include/
    include/lar_youbot/
)
add_library(
    lar_youbot  
    src/lar_youbot/PIDController.cpp
    src/lar_youbot/YouBot.cpp
)


#NODE youbot_playground
add_executable(youbot_playground src/nodes/youbot_playground.cpp)
target_link_libraries(youbot_playground ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} lar_youbot )


