cmake_minimum_required(VERSION 2.8.3)
project(laser_scanner)

add_definitions(-Wall -std=c++11)
find_package(catkin REQUIRED COMPONENTS
    roscpp rospy sensor_msgs tf
    message_generation laser_geometry
    
    )

find_package(PCL 1.7.1 REQUIRED COMPONENTS common io kdtree surface features)
catkin_package(
)


include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${PCL_LIBRARY_DIRS}
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
    ${PCL_KDTREE_LIBRARIES}
    ${PCL_SURFACE_LIBRARIES}
    ${PCL_FEATURES_LIBRARIES}
    )
target_link_libraries(project
    ${catkin_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCL_KDTREE_LIBRARIES}
    )

link_libraries(pcl17_2d ; pcl17_surface ; pcl_io_ply  ;   pcl_people  ; pcl_stereo ;pcl17_common ; pcl17_tracking.so ; pcl_io ;        pcl_people  ;  pcl_stereo; pcl17_features;pcl17_visualization.so  ;pcl_io;pcl_recognition   ;        pcl_stereo ;pcl17_filters    ;       pcl_2d    ;           pcl_io   ;      pcl_recognition;        pcl_surface;pcl17_io_ply    ;       pcl_2d      ;     pcl_kdtree;     pcl_recognition   ;    pcl_surface ;pcl17_io    ;            pcl_2d    ;     pcl_kdtree ;     pcl_registration ;           pcl_surface ;pcl17_kdtree  ;         pcl_common   ;       pcl_kdtree  ;   pcl_registration ;      pcl_tracking ;pcl17_keypoints ;       pcl_common   ;   pcl_keypoints  ;     pcl_registration   ;  pcl_tracking;pcl17_ml    ;    pcl_common;     pcl_keypoints ;   pcl_sample_consensus  ;     pcl_tracking; pcl17_octree  ;         pcl_features ;     pcl_keypoints ; pcl_sample_consensus ;  pcl_visualization; pcl17_people   ;         pcl_features;  pcl_ml  ;           pcl_sample_consensus ; pcl_visualization ;pcl17_recognition   ;   pcl_features  ; pcl_ml   ;     pcl_search  ;               pcl_visualization ;pcl17_registration  ;   pcl_filters   ;     pcl_ml;       pcl_search   ;    pcl17_sample_consensus;  pcl_filters;     pcl_octree   ;        pcl_search; pcl17_search      ;   pcl_filters ;   pcl_octree ;     pcl_segmentation ;pcl17_segmentation;     pcl_io_ply ;        pcl_octree ;    pcl_segmentation; pcl17_stereo;          pcl_io_ply;      pcl_people ;         pcl_segmentation ) 
