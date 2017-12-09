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

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")

  add_rostest_gtest(test_vision test/test_vision.launch test/test_vision.cpp src/classifier.cpp)
  target_link_libraries(test_vision ${catkin_LIBRARIES} ${OpenCV_LIBS})
  ##add_dependencies(test_vision vision ${catkin_EXPORTED_TARGETS})

  add_rostest_gtest(test_robot test/test_robot.launch test/test_robot.cpp src/robot.cpp)
  target_link_libraries(test_robot ${catkin_LIBRARIES} ${OpenCV_LIBS})
  ##add_dependencies(test_robot vision ${catkin_EXPORTED_TARGETS})

  add_rostest_gtest(test_ros test/test_ros.launch test/test_ros.cpp test/testclass.cpp)
  target_link_libraries(test_ros ${catkin_LIBRARIES} ${OpenCV_LIBS})
  #add_dependencies(test_robot vision ${catkin_EXPORTED_TARGETS})

endif()
