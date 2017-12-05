cmake_minimum_required(VERSION 2.8.3)
project(traffic_sign_recognition)

add_definitions(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS cv_bridge roscpp rospy std_msgs genmsg geometry_msgs)
find_package(OpenCV REQUIRED)

## Declare ROS messages and services
add_message_files(FILES sign.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Executables
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(vision src/vision_node.cpp src/classifier.cpp)
target_link_libraries(vision ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(vision traffic_sign_recognition_generate_messages_cpp)

add_executable(robot src/navig_node.cpp src/robot.cpp)
target_link_libraries(robot ${catkin_LIBRARIES})
add_dependencies(robot traffic_sign_recognition_generate_messages_cpp)

