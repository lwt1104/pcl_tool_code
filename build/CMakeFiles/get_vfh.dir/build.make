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
include CMakeFiles/get_vfh.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/get_vfh.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/get_vfh.dir/flags.make

CMakeFiles/get_vfh.dir/get_vfh.cpp.o: CMakeFiles/get_vfh.dir/flags.make
CMakeFiles/get_vfh.dir/get_vfh.cpp.o: ../get_vfh.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lwt1104/correspondence_grouping/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/get_vfh.dir/get_vfh.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/get_vfh.dir/get_vfh.cpp.o -c /home/lwt1104/correspondence_grouping/get_vfh.cpp

CMakeFiles/get_vfh.dir/get_vfh.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/get_vfh.dir/get_vfh.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lwt1104/correspondence_grouping/get_vfh.cpp > CMakeFiles/get_vfh.dir/get_vfh.cpp.i

CMakeFiles/get_vfh.dir/get_vfh.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/get_vfh.dir/get_vfh.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lwt1104/correspondence_grouping/get_vfh.cpp -o CMakeFiles/get_vfh.dir/get_vfh.cpp.s

CMakeFiles/get_vfh.dir/get_vfh.cpp.o.requires:
.PHONY : CMakeFiles/get_vfh.dir/get_vfh.cpp.o.requires

CMakeFiles/get_vfh.dir/get_vfh.cpp.o.provides: CMakeFiles/get_vfh.dir/get_vfh.cpp.o.requires
	$(MAKE) -f CMakeFiles/get_vfh.dir/build.make CMakeFiles/get_vfh.dir/get_vfh.cpp.o.provides.build
.PHONY : CMakeFiles/get_vfh.dir/get_vfh.cpp.o.provides

CMakeFiles/get_vfh.dir/get_vfh.cpp.o.provides.build: CMakeFiles/get_vfh.dir/get_vfh.cpp.o

# Object files for target get_vfh
get_vfh_OBJECTS = \
"CMakeFiles/get_vfh.dir/get_vfh.cpp.o"

# External object files for target get_vfh
get_vfh_EXTERNAL_OBJECTS =

get_vfh: CMakeFiles/get_vfh.dir/get_vfh.cpp.o
get_vfh: CMakeFiles/get_vfh.dir/build.make
get_vfh: /usr/lib/x86_64-linux-gnu/libboost_system.so
get_vfh: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
get_vfh: /usr/lib/x86_64-linux-gnu/libboost_thread.so
get_vfh: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
get_vfh: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
get_vfh: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
get_vfh: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
get_vfh: /usr/lib/x86_64-linux-gnu/libpthread.so
get_vfh: /usr/lib/libpcl_common.so
get_vfh: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
get_vfh: /usr/lib/libpcl_kdtree.so
get_vfh: /usr/lib/libpcl_octree.so
get_vfh: /usr/lib/libpcl_search.so
get_vfh: /usr/lib/x86_64-linux-gnu/libqhull.so
get_vfh: /usr/lib/libpcl_surface.so
get_vfh: /usr/lib/libpcl_sample_consensus.so
get_vfh: /usr/lib/libOpenNI.so
get_vfh: /usr/lib/libOpenNI2.so
get_vfh: /usr/lib/libvtkCommon.so.5.8.0
get_vfh: /usr/lib/libvtkFiltering.so.5.8.0
get_vfh: /usr/lib/libvtkImaging.so.5.8.0
get_vfh: /usr/lib/libvtkGraphics.so.5.8.0
get_vfh: /usr/lib/libvtkGenericFiltering.so.5.8.0
get_vfh: /usr/lib/libvtkIO.so.5.8.0
get_vfh: /usr/lib/libvtkRendering.so.5.8.0
get_vfh: /usr/lib/libvtkVolumeRendering.so.5.8.0
get_vfh: /usr/lib/libvtkHybrid.so.5.8.0
get_vfh: /usr/lib/libvtkWidgets.so.5.8.0
get_vfh: /usr/lib/libvtkParallel.so.5.8.0
get_vfh: /usr/lib/libvtkInfovis.so.5.8.0
get_vfh: /usr/lib/libvtkGeovis.so.5.8.0
get_vfh: /usr/lib/libvtkViews.so.5.8.0
get_vfh: /usr/lib/libvtkCharts.so.5.8.0
get_vfh: /usr/lib/libpcl_io.so
get_vfh: /usr/lib/libpcl_filters.so
get_vfh: /usr/lib/libpcl_features.so
get_vfh: /usr/lib/libpcl_keypoints.so
get_vfh: /usr/lib/libpcl_registration.so
get_vfh: /usr/lib/libpcl_segmentation.so
get_vfh: /usr/lib/libpcl_recognition.so
get_vfh: /usr/lib/libpcl_visualization.so
get_vfh: /usr/lib/libpcl_people.so
get_vfh: /usr/lib/libpcl_outofcore.so
get_vfh: /usr/lib/libpcl_tracking.so
get_vfh: /usr/lib/libpcl_apps.so
get_vfh: /usr/lib/x86_64-linux-gnu/libboost_system.so
get_vfh: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
get_vfh: /usr/lib/x86_64-linux-gnu/libboost_thread.so
get_vfh: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
get_vfh: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
get_vfh: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
get_vfh: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
get_vfh: /usr/lib/x86_64-linux-gnu/libpthread.so
get_vfh: /usr/lib/x86_64-linux-gnu/libqhull.so
get_vfh: /usr/lib/libOpenNI.so
get_vfh: /usr/lib/libOpenNI2.so
get_vfh: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
get_vfh: /usr/lib/libvtkCommon.so.5.8.0
get_vfh: /usr/lib/libvtkFiltering.so.5.8.0
get_vfh: /usr/lib/libvtkImaging.so.5.8.0
get_vfh: /usr/lib/libvtkGraphics.so.5.8.0
get_vfh: /usr/lib/libvtkGenericFiltering.so.5.8.0
get_vfh: /usr/lib/libvtkIO.so.5.8.0
get_vfh: /usr/lib/libvtkRendering.so.5.8.0
get_vfh: /usr/lib/libvtkVolumeRendering.so.5.8.0
get_vfh: /usr/lib/libvtkHybrid.so.5.8.0
get_vfh: /usr/lib/libvtkWidgets.so.5.8.0
get_vfh: /usr/lib/libvtkParallel.so.5.8.0
get_vfh: /usr/lib/libvtkInfovis.so.5.8.0
get_vfh: /usr/lib/libvtkGeovis.so.5.8.0
get_vfh: /usr/lib/libvtkViews.so.5.8.0
get_vfh: /usr/lib/libvtkCharts.so.5.8.0
get_vfh: /usr/lib/libpcl_common.so
get_vfh: /usr/lib/libpcl_kdtree.so
get_vfh: /usr/lib/libpcl_octree.so
get_vfh: /usr/lib/libpcl_search.so
get_vfh: /usr/lib/libpcl_surface.so
get_vfh: /usr/lib/libpcl_sample_consensus.so
get_vfh: /usr/lib/libpcl_io.so
get_vfh: /usr/lib/libpcl_filters.so
get_vfh: /usr/lib/libpcl_features.so
get_vfh: /usr/lib/libpcl_keypoints.so
get_vfh: /usr/lib/libpcl_registration.so
get_vfh: /usr/lib/libpcl_segmentation.so
get_vfh: /usr/lib/libpcl_recognition.so
get_vfh: /usr/lib/libpcl_visualization.so
get_vfh: /usr/lib/libpcl_people.so
get_vfh: /usr/lib/libpcl_outofcore.so
get_vfh: /usr/lib/libpcl_tracking.so
get_vfh: /usr/lib/libpcl_apps.so
get_vfh: /usr/lib/libvtkViews.so.5.8.0
get_vfh: /usr/lib/libvtkInfovis.so.5.8.0
get_vfh: /usr/lib/libvtkWidgets.so.5.8.0
get_vfh: /usr/lib/libvtkVolumeRendering.so.5.8.0
get_vfh: /usr/lib/libvtkHybrid.so.5.8.0
get_vfh: /usr/lib/libvtkParallel.so.5.8.0
get_vfh: /usr/lib/libvtkRendering.so.5.8.0
get_vfh: /usr/lib/libvtkImaging.so.5.8.0
get_vfh: /usr/lib/libvtkGraphics.so.5.8.0
get_vfh: /usr/lib/libvtkIO.so.5.8.0
get_vfh: /usr/lib/libvtkFiltering.so.5.8.0
get_vfh: /usr/lib/libvtkCommon.so.5.8.0
get_vfh: /usr/lib/libvtksys.so.5.8.0
get_vfh: CMakeFiles/get_vfh.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable get_vfh"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/get_vfh.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/get_vfh.dir/build: get_vfh
.PHONY : CMakeFiles/get_vfh.dir/build

CMakeFiles/get_vfh.dir/requires: CMakeFiles/get_vfh.dir/get_vfh.cpp.o.requires
.PHONY : CMakeFiles/get_vfh.dir/requires

CMakeFiles/get_vfh.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/get_vfh.dir/cmake_clean.cmake
.PHONY : CMakeFiles/get_vfh.dir/clean

CMakeFiles/get_vfh.dir/depend:
	cd /home/lwt1104/correspondence_grouping/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lwt1104/correspondence_grouping /home/lwt1104/correspondence_grouping /home/lwt1104/correspondence_grouping/build /home/lwt1104/correspondence_grouping/build /home/lwt1104/correspondence_grouping/build/CMakeFiles/get_vfh.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/get_vfh.dir/depend

