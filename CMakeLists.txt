cmake_minimum_required(VERSION 2.8.3)
project(laser_scanner)

add_definitions(-Wall -std=c++11)
find_package(catkin REQUIRED COMPONENTS
    roscpp rospy sensor_msgs tf
    )

catkin_package(
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )

add_executable(laser_scanner src/merge.cpp)
target_link_libraries(laser_scanner
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )
