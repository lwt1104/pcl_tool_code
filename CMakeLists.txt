cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

project(correspondence_grouping)

find_package(PCL 1.7 REQUIRED)
find_package(PNG REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${PNG_INCLUDE_DIR})

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

add_executable (alignment_prerejective alignment_prerejective.cpp)
target_link_libraries (alignment_prerejective ${PCL_LIBRARIES})

add_executable (plane_cluster_seg plane_cluster_seg.cpp)
target_link_libraries (plane_cluster_seg ${PCL_LIBRARIES})

add_executable (pcd2png pcd2png.cpp)
target_link_libraries (pcd2png ${PCL_LIBRARIES} ${PNG_LIBRARY})

find_package(HDF5)
if(HDF5_FOUND)
# find FLANN with pkg-config 
 find_package(PkgConfig REQUIRED) 
 pkg_check_modules(FLANN REQUIRED flann) 
 link_directories(${FLANN_LIBRARY_DIRS}) 
 include_directories(${FLANN_INCLUDE_DIRS}) 
 if(FLANN_CFLAGS) 
   add_definitions(${FLANN_CFLAGS}) 
 endif(FLANN_CFLAGS)

#  find_package(FLANN)
#  include_directories(${FLANN_INCLUDE_DIRS})

  include_directories(${HDF5_INCLUDE_DIR})

  add_executable(build_tree build_tree.cpp)
  target_link_libraries(build_tree ${PCL_LIBRARIES} ${Boost_LIBRARIES}
                                 ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY})

  add_executable(nearest_neighbors nearest_neighbors.cpp)
  target_link_libraries(nearest_neighbors ${PCL_LIBRARIES} 
                                        ${Boost_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY} 
                                        )
endif(HDF5_FOUND)

