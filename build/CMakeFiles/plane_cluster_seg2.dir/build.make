# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lwt1104/correspondence_grouping

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lwt1104/correspondence_grouping/build

# Include any dependencies generated for this target.
include CMakeFiles/plane_cluster_seg2.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/plane_cluster_seg2.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/plane_cluster_seg2.dir/flags.make

CMakeFiles/plane_cluster_seg2.dir/plane_cluster_seg2.cpp.o: CMakeFiles/plane_cluster_seg2.dir/flags.make
CMakeFiles/plane_cluster_seg2.dir/plane_cluster_seg2.cpp.o: ../plane_cluster_seg2.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lwt1104/correspondence_grouping/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/plane_cluster_seg2.dir/plane_cluster_seg2.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/plane_cluster_seg2.dir/plane_cluster_seg2.cpp.o -c /home/lwt1104/correspondence_grouping/plane_cluster_seg2.cpp

CMakeFiles/plane_cluster_seg2.dir/plane_cluster_seg2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/plane_cluster_seg2.dir/plane_cluster_seg2.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lwt1104/correspondence_grouping/plane_cluster_seg2.cpp > CMakeFiles/plane_cluster_seg2.dir/plane_cluster_seg2.cpp.i

CMakeFiles/plane_cluster_seg2.dir/plane_cluster_seg2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/plane_cluster_seg2.dir/plane_cluster_seg2.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lwt1104/correspondence_grouping/plane_cluster_seg2.cpp -o CMakeFiles/plane_cluster_seg2.dir/plane_cluster_seg2.cpp.s

CMakeFiles/plane_cluster_seg2.dir/plane_cluster_seg2.cpp.o.requires:
.PHONY : CMakeFiles/plane_cluster_seg2.dir/plane_cluster_seg2.cpp.o.requires

CMakeFiles/plane_cluster_seg2.dir/plane_cluster_seg2.cpp.o.provides: CMakeFiles/plane_cluster_seg2.dir/plane_cluster_seg2.cpp.o.requires
	$(MAKE) -f CMakeFiles/plane_cluster_seg2.dir/build.make CMakeFiles/plane_cluster_seg2.dir/plane_cluster_seg2.cpp.o.provides.build
.PHONY : CMakeFiles/plane_cluster_seg2.dir/plane_cluster_seg2.cpp.o.provides

CMakeFiles/plane_cluster_seg2.dir/plane_cluster_seg2.cpp.o.provides.build: CMakeFiles/plane_cluster_seg2.dir/plane_cluster_seg2.cpp.o

# Object files for target plane_cluster_seg2
plane_cluster_seg2_OBJECTS = \
"CMakeFiles/plane_cluster_seg2.dir/plane_cluster_seg2.cpp.o"

# External object files for target plane_cluster_seg2
plane_cluster_seg2_EXTERNAL_OBJECTS =

plane_cluster_seg2: CMakeFiles/plane_cluster_seg2.dir/plane_cluster_seg2.cpp.o
plane_cluster_seg2: CMakeFiles/plane_cluster_seg2.dir/build.make
plane_cluster_seg2: /usr/lib/x86_64-linux-gnu/libboost_system.so
plane_cluster_seg2: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
plane_cluster_seg2: /usr/lib/x86_64-linux-gnu/libboost_thread.so
plane_cluster_seg2: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
plane_cluster_seg2: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
plane_cluster_seg2: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
plane_cluster_seg2: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
plane_cluster_seg2: /usr/lib/x86_64-linux-gnu/libpthread.so
plane_cluster_seg2: /usr/lib/libpcl_common.so
plane_cluster_seg2: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
plane_cluster_seg2: /usr/lib/libpcl_kdtree.so
plane_cluster_seg2: /usr/lib/libpcl_octree.so
plane_cluster_seg2: /usr/lib/libpcl_search.so
plane_cluster_seg2: /usr/lib/x86_64-linux-gnu/libqhull.so
plane_cluster_seg2: /usr/lib/libpcl_surface.so
plane_cluster_seg2: /usr/lib/libpcl_sample_consensus.so
plane_cluster_seg2: /usr/lib/libOpenNI.so
plane_cluster_seg2: /usr/lib/libOpenNI2.so
plane_cluster_seg2: /usr/lib/libvtkCommon.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkFiltering.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkImaging.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkGraphics.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkGenericFiltering.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkIO.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkRendering.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkVolumeRendering.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkHybrid.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkWidgets.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkParallel.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkInfovis.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkGeovis.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkViews.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkCharts.so.5.8.0
plane_cluster_seg2: /usr/lib/libpcl_io.so
plane_cluster_seg2: /usr/lib/libpcl_filters.so
plane_cluster_seg2: /usr/lib/libpcl_features.so
plane_cluster_seg2: /usr/lib/libpcl_keypoints.so
plane_cluster_seg2: /usr/lib/libpcl_registration.so
plane_cluster_seg2: /usr/lib/libpcl_segmentation.so
plane_cluster_seg2: /usr/lib/libpcl_recognition.so
plane_cluster_seg2: /usr/lib/libpcl_visualization.so
plane_cluster_seg2: /usr/lib/libpcl_people.so
plane_cluster_seg2: /usr/lib/libpcl_outofcore.so
plane_cluster_seg2: /usr/lib/libpcl_tracking.so
plane_cluster_seg2: /usr/lib/libpcl_apps.so
plane_cluster_seg2: /usr/lib/x86_64-linux-gnu/libboost_system.so
plane_cluster_seg2: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
plane_cluster_seg2: /usr/lib/x86_64-linux-gnu/libboost_thread.so
plane_cluster_seg2: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
plane_cluster_seg2: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
plane_cluster_seg2: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
plane_cluster_seg2: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
plane_cluster_seg2: /usr/lib/x86_64-linux-gnu/libpthread.so
plane_cluster_seg2: /usr/lib/x86_64-linux-gnu/libqhull.so
plane_cluster_seg2: /usr/lib/libOpenNI.so
plane_cluster_seg2: /usr/lib/libOpenNI2.so
plane_cluster_seg2: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
plane_cluster_seg2: /usr/lib/libvtkCommon.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkFiltering.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkImaging.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkGraphics.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkGenericFiltering.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkIO.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkRendering.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkVolumeRendering.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkHybrid.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkWidgets.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkParallel.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkInfovis.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkGeovis.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkViews.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkCharts.so.5.8.0
plane_cluster_seg2: /usr/lib/x86_64-linux-gnu/libpng.so
plane_cluster_seg2: /usr/lib/libpcl_common.so
plane_cluster_seg2: /usr/lib/libpcl_kdtree.so
plane_cluster_seg2: /usr/lib/libpcl_octree.so
plane_cluster_seg2: /usr/lib/libpcl_search.so
plane_cluster_seg2: /usr/lib/libpcl_surface.so
plane_cluster_seg2: /usr/lib/libpcl_sample_consensus.so
plane_cluster_seg2: /usr/lib/libpcl_io.so
plane_cluster_seg2: /usr/lib/libpcl_filters.so
plane_cluster_seg2: /usr/lib/libpcl_features.so
plane_cluster_seg2: /usr/lib/libpcl_keypoints.so
plane_cluster_seg2: /usr/lib/libpcl_registration.so
plane_cluster_seg2: /usr/lib/libpcl_segmentation.so
plane_cluster_seg2: /usr/lib/libpcl_recognition.so
plane_cluster_seg2: /usr/lib/libpcl_visualization.so
plane_cluster_seg2: /usr/lib/libpcl_people.so
plane_cluster_seg2: /usr/lib/libpcl_outofcore.so
plane_cluster_seg2: /usr/lib/libpcl_tracking.so
plane_cluster_seg2: /usr/lib/libpcl_apps.so
plane_cluster_seg2: /usr/lib/x86_64-linux-gnu/libpng.so
plane_cluster_seg2: /usr/lib/libvtkViews.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkInfovis.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkWidgets.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkVolumeRendering.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkHybrid.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkParallel.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkRendering.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkImaging.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkGraphics.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkIO.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkFiltering.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtkCommon.so.5.8.0
plane_cluster_seg2: /usr/lib/libvtksys.so.5.8.0
plane_cluster_seg2: CMakeFiles/plane_cluster_seg2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable plane_cluster_seg2"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/plane_cluster_seg2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/plane_cluster_seg2.dir/build: plane_cluster_seg2
.PHONY : CMakeFiles/plane_cluster_seg2.dir/build

CMakeFiles/plane_cluster_seg2.dir/requires: CMakeFiles/plane_cluster_seg2.dir/plane_cluster_seg2.cpp.o.requires
.PHONY : CMakeFiles/plane_cluster_seg2.dir/requires

CMakeFiles/plane_cluster_seg2.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/plane_cluster_seg2.dir/cmake_clean.cmake
.PHONY : CMakeFiles/plane_cluster_seg2.dir/clean

CMakeFiles/plane_cluster_seg2.dir/depend:
	cd /home/lwt1104/correspondence_grouping/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lwt1104/correspondence_grouping /home/lwt1104/correspondence_grouping /home/lwt1104/correspondence_grouping/build /home/lwt1104/correspondence_grouping/build /home/lwt1104/correspondence_grouping/build/CMakeFiles/plane_cluster_seg2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/plane_cluster_seg2.dir/depend

