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
include CMakeFiles/cluster_seg.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cluster_seg.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cluster_seg.dir/flags.make

CMakeFiles/cluster_seg.dir/cluster_seg.cpp.o: CMakeFiles/cluster_seg.dir/flags.make
CMakeFiles/cluster_seg.dir/cluster_seg.cpp.o: ../cluster_seg.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lwt1104/correspondence_grouping/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/cluster_seg.dir/cluster_seg.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/cluster_seg.dir/cluster_seg.cpp.o -c /home/lwt1104/correspondence_grouping/cluster_seg.cpp

CMakeFiles/cluster_seg.dir/cluster_seg.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cluster_seg.dir/cluster_seg.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lwt1104/correspondence_grouping/cluster_seg.cpp > CMakeFiles/cluster_seg.dir/cluster_seg.cpp.i

CMakeFiles/cluster_seg.dir/cluster_seg.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cluster_seg.dir/cluster_seg.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lwt1104/correspondence_grouping/cluster_seg.cpp -o CMakeFiles/cluster_seg.dir/cluster_seg.cpp.s

CMakeFiles/cluster_seg.dir/cluster_seg.cpp.o.requires:
.PHONY : CMakeFiles/cluster_seg.dir/cluster_seg.cpp.o.requires

CMakeFiles/cluster_seg.dir/cluster_seg.cpp.o.provides: CMakeFiles/cluster_seg.dir/cluster_seg.cpp.o.requires
	$(MAKE) -f CMakeFiles/cluster_seg.dir/build.make CMakeFiles/cluster_seg.dir/cluster_seg.cpp.o.provides.build
.PHONY : CMakeFiles/cluster_seg.dir/cluster_seg.cpp.o.provides

CMakeFiles/cluster_seg.dir/cluster_seg.cpp.o.provides.build: CMakeFiles/cluster_seg.dir/cluster_seg.cpp.o

# Object files for target cluster_seg
cluster_seg_OBJECTS = \
"CMakeFiles/cluster_seg.dir/cluster_seg.cpp.o"

# External object files for target cluster_seg
cluster_seg_EXTERNAL_OBJECTS =

cluster_seg: CMakeFiles/cluster_seg.dir/cluster_seg.cpp.o
cluster_seg: CMakeFiles/cluster_seg.dir/build.make
cluster_seg: /usr/lib/x86_64-linux-gnu/libboost_system.so
cluster_seg: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
cluster_seg: /usr/lib/x86_64-linux-gnu/libboost_thread.so
cluster_seg: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
cluster_seg: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
cluster_seg: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
cluster_seg: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
cluster_seg: /usr/lib/x86_64-linux-gnu/libpthread.so
cluster_seg: /usr/lib/libpcl_common.so
cluster_seg: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
cluster_seg: /usr/lib/libpcl_kdtree.so
cluster_seg: /usr/lib/libpcl_octree.so
cluster_seg: /usr/lib/libpcl_search.so
cluster_seg: /usr/lib/x86_64-linux-gnu/libqhull.so
cluster_seg: /usr/lib/libpcl_surface.so
cluster_seg: /usr/lib/libpcl_sample_consensus.so
cluster_seg: /usr/lib/libOpenNI.so
cluster_seg: /usr/lib/libOpenNI2.so
cluster_seg: /usr/lib/libvtkCommon.so.5.8.0
cluster_seg: /usr/lib/libvtkFiltering.so.5.8.0
cluster_seg: /usr/lib/libvtkImaging.so.5.8.0
cluster_seg: /usr/lib/libvtkGraphics.so.5.8.0
cluster_seg: /usr/lib/libvtkGenericFiltering.so.5.8.0
cluster_seg: /usr/lib/libvtkIO.so.5.8.0
cluster_seg: /usr/lib/libvtkRendering.so.5.8.0
cluster_seg: /usr/lib/libvtkVolumeRendering.so.5.8.0
cluster_seg: /usr/lib/libvtkHybrid.so.5.8.0
cluster_seg: /usr/lib/libvtkWidgets.so.5.8.0
cluster_seg: /usr/lib/libvtkParallel.so.5.8.0
cluster_seg: /usr/lib/libvtkInfovis.so.5.8.0
cluster_seg: /usr/lib/libvtkGeovis.so.5.8.0
cluster_seg: /usr/lib/libvtkViews.so.5.8.0
cluster_seg: /usr/lib/libvtkCharts.so.5.8.0
cluster_seg: /usr/lib/libpcl_io.so
cluster_seg: /usr/lib/libpcl_filters.so
cluster_seg: /usr/lib/libpcl_features.so
cluster_seg: /usr/lib/libpcl_keypoints.so
cluster_seg: /usr/lib/libpcl_registration.so
cluster_seg: /usr/lib/libpcl_segmentation.so
cluster_seg: /usr/lib/libpcl_recognition.so
cluster_seg: /usr/lib/libpcl_visualization.so
cluster_seg: /usr/lib/libpcl_people.so
cluster_seg: /usr/lib/libpcl_outofcore.so
cluster_seg: /usr/lib/libpcl_tracking.so
cluster_seg: /usr/lib/libpcl_apps.so
cluster_seg: /usr/lib/x86_64-linux-gnu/libboost_system.so
cluster_seg: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
cluster_seg: /usr/lib/x86_64-linux-gnu/libboost_thread.so
cluster_seg: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
cluster_seg: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
cluster_seg: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
cluster_seg: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
cluster_seg: /usr/lib/x86_64-linux-gnu/libpthread.so
cluster_seg: /usr/lib/x86_64-linux-gnu/libqhull.so
cluster_seg: /usr/lib/libOpenNI.so
cluster_seg: /usr/lib/libOpenNI2.so
cluster_seg: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
cluster_seg: /usr/lib/libvtkCommon.so.5.8.0
cluster_seg: /usr/lib/libvtkFiltering.so.5.8.0
cluster_seg: /usr/lib/libvtkImaging.so.5.8.0
cluster_seg: /usr/lib/libvtkGraphics.so.5.8.0
cluster_seg: /usr/lib/libvtkGenericFiltering.so.5.8.0
cluster_seg: /usr/lib/libvtkIO.so.5.8.0
cluster_seg: /usr/lib/libvtkRendering.so.5.8.0
cluster_seg: /usr/lib/libvtkVolumeRendering.so.5.8.0
cluster_seg: /usr/lib/libvtkHybrid.so.5.8.0
cluster_seg: /usr/lib/libvtkWidgets.so.5.8.0
cluster_seg: /usr/lib/libvtkParallel.so.5.8.0
cluster_seg: /usr/lib/libvtkInfovis.so.5.8.0
cluster_seg: /usr/lib/libvtkGeovis.so.5.8.0
cluster_seg: /usr/lib/libvtkViews.so.5.8.0
cluster_seg: /usr/lib/libvtkCharts.so.5.8.0
cluster_seg: /usr/lib/libpcl_common.so
cluster_seg: /usr/lib/libpcl_kdtree.so
cluster_seg: /usr/lib/libpcl_octree.so
cluster_seg: /usr/lib/libpcl_search.so
cluster_seg: /usr/lib/libpcl_surface.so
cluster_seg: /usr/lib/libpcl_sample_consensus.so
cluster_seg: /usr/lib/libpcl_io.so
cluster_seg: /usr/lib/libpcl_filters.so
cluster_seg: /usr/lib/libpcl_features.so
cluster_seg: /usr/lib/libpcl_keypoints.so
cluster_seg: /usr/lib/libpcl_registration.so
cluster_seg: /usr/lib/libpcl_segmentation.so
cluster_seg: /usr/lib/libpcl_recognition.so
cluster_seg: /usr/lib/libpcl_visualization.so
cluster_seg: /usr/lib/libpcl_people.so
cluster_seg: /usr/lib/libpcl_outofcore.so
cluster_seg: /usr/lib/libpcl_tracking.so
cluster_seg: /usr/lib/libpcl_apps.so
cluster_seg: /usr/lib/libvtkViews.so.5.8.0
cluster_seg: /usr/lib/libvtkInfovis.so.5.8.0
cluster_seg: /usr/lib/libvtkWidgets.so.5.8.0
cluster_seg: /usr/lib/libvtkVolumeRendering.so.5.8.0
cluster_seg: /usr/lib/libvtkHybrid.so.5.8.0
cluster_seg: /usr/lib/libvtkParallel.so.5.8.0
cluster_seg: /usr/lib/libvtkRendering.so.5.8.0
cluster_seg: /usr/lib/libvtkImaging.so.5.8.0
cluster_seg: /usr/lib/libvtkGraphics.so.5.8.0
cluster_seg: /usr/lib/libvtkIO.so.5.8.0
cluster_seg: /usr/lib/libvtkFiltering.so.5.8.0
cluster_seg: /usr/lib/libvtkCommon.so.5.8.0
cluster_seg: /usr/lib/libvtksys.so.5.8.0
cluster_seg: CMakeFiles/cluster_seg.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable cluster_seg"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cluster_seg.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cluster_seg.dir/build: cluster_seg
.PHONY : CMakeFiles/cluster_seg.dir/build

CMakeFiles/cluster_seg.dir/requires: CMakeFiles/cluster_seg.dir/cluster_seg.cpp.o.requires
.PHONY : CMakeFiles/cluster_seg.dir/requires

CMakeFiles/cluster_seg.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cluster_seg.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cluster_seg.dir/clean

CMakeFiles/cluster_seg.dir/depend:
	cd /home/lwt1104/correspondence_grouping/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lwt1104/correspondence_grouping /home/lwt1104/correspondence_grouping /home/lwt1104/correspondence_grouping/build /home/lwt1104/correspondence_grouping/build /home/lwt1104/correspondence_grouping/build/CMakeFiles/cluster_seg.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cluster_seg.dir/depend
