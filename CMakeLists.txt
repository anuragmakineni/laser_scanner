cmake_minimum_required(VERSION 2.8.3)
project(laser_scanner)

add_definitions(-Wall -std=c++11)
find_package(catkin REQUIRED COMPONENTS
    roscpp rospy sensor_msgs tf
    message_generation
    )

find_package(PCL 1.7 REQUIRED COMPONENTS common io)
catkin_package(
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )
link_directories(
    ${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(laser_scanner src/merge.cpp)
target_link_libraries(laser_scanner
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    )
add_service_files(
  FILES
  Trigger.srv)
