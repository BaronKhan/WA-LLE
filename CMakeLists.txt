# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(hcr_walker)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
#add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
#add_dependencies(listener beginner_tutorials_generate_messages_cpp)

add_executable(hcr_gait src/hcr_gait.cpp)
target_link_libraries(hcr_gait ${catkin_LIBRARIES})

add_executable(hcr_gait_test_listener src/hcr_gait_test_listener.cpp)
target_link_libraries(hcr_gait_test_listener ${catkin_LIBRARIES})

add_executable(hcr_imu src/hcr_imu.cpp)
target_link_libraries(hcr_imu ${catkin_LIBRARIES})

add_executable(dummy_listener src/dummy_listener.cpp)
target_link_libraries(dummy_listener ${catkin_LIBRARIES})
# %EndTag(FULLTEXT)%
