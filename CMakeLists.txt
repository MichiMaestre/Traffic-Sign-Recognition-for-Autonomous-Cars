cmake_minimum_required(VERSION 2.8.3)
project(traffic_sign_recognition)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg geometry_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(walker src/walker.cpp src/robot.cpp)
#target_link_libraries(walker ${catkin_LIBRARIES})

