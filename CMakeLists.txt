cmake_minimum_required(VERSION 2.8.3)
project(laser_scanner)

add_definitions(-Wall -std=c++11)
find_package(catkin REQUIRED COMPONENTS
    roscpp rospy sensor_msgs tf
    message_generation laser_geometry
    )

find_package(PCL 1.7 REQUIRED COMPONENTS common io)
catkin_package(
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )
link_directories(
    ${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(merge src/merge.cpp)
add_executable(project src/project.cpp)
target_link_libraries(merge
    ${catkin_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    )
target_link_libraries(project
    ${catkin_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    )
