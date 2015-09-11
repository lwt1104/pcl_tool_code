cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(correspondence_grouping)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (correspondence_grouping correspondence_grouping.cpp)
target_link_libraries (correspondence_grouping ${PCL_LIBRARIES})

add_executable (print_point print_point.cpp)
target_link_libraries (print_point ${PCL_LIBRARIES})

add_executable (pcd_recorder pcd_recorder.cpp)
target_link_libraries (pcd_recorder ${PCL_LIBRARIES})

add_executable (remove_plane planar_seg.cpp)
target_link_libraries (remove_plane ${PCL_LIBRARIES})

add_executable (cluster_seg cluster_seg.cpp)
target_link_libraries (cluster_seg ${PCL_LIBRARIES})

add_executable (online_match online_match.cpp)
target_link_libraries (online_match ${PCL_LIBRARIES})

add_executable (global_hypothesis_verification global_hypothesis_verification.cpp)
target_link_libraries (global_hypothesis_verification ${PCL_LIBRARIES})

