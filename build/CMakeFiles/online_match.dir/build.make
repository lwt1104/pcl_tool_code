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
include CMakeFiles/online_match.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/online_match.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/online_match.dir/flags.make

CMakeFiles/online_match.dir/online_match.cpp.o: CMakeFiles/online_match.dir/flags.make
CMakeFiles/online_match.dir/online_match.cpp.o: ../online_match.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lwt1104/correspondence_grouping/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/online_match.dir/online_match.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/online_match.dir/online_match.cpp.o -c /home/lwt1104/correspondence_grouping/online_match.cpp

CMakeFiles/online_match.dir/online_match.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/online_match.dir/online_match.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lwt1104/correspondence_grouping/online_match.cpp > CMakeFiles/online_match.dir/online_match.cpp.i

CMakeFiles/online_match.dir/online_match.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/online_match.dir/online_match.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lwt1104/correspondence_grouping/online_match.cpp -o CMakeFiles/online_match.dir/online_match.cpp.s

CMakeFiles/online_match.dir/online_match.cpp.o.requires:
.PHONY : CMakeFiles/online_match.dir/online_match.cpp.o.requires

CMakeFiles/online_match.dir/online_match.cpp.o.provides: CMakeFiles/online_match.dir/online_match.cpp.o.requires
	$(MAKE) -f CMakeFiles/online_match.dir/build.make CMakeFiles/online_match.dir/online_match.cpp.o.provides.build
.PHONY : CMakeFiles/online_match.dir/online_match.cpp.o.provides

CMakeFiles/online_match.dir/online_match.cpp.o.provides.build: CMakeFiles/online_match.dir/online_match.cpp.o

# Object files for target online_match
online_match_OBJECTS = \
"CMakeFiles/online_match.dir/online_match.cpp.o"

# External object files for target online_match
online_match_EXTERNAL_OBJECTS =

online_match: CMakeFiles/online_match.dir/online_match.cpp.o
online_match: CMakeFiles/online_match.dir/build.make
online_match: /usr/lib/x86_64-linux-gnu/libboost_system.so
online_match: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
online_match: /usr/lib/x86_64-linux-gnu/libboost_thread.so
online_match: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
online_match: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
online_match: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
online_match: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
online_match: /usr/lib/x86_64-linux-gnu/libpthread.so
online_match: /usr/lib/libpcl_common.so
online_match: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
online_match: /usr/lib/libpcl_kdtree.so
online_match: /usr/lib/libpcl_octree.so
online_match: /usr/lib/libpcl_search.so
online_match: /usr/lib/x86_64-linux-gnu/libqhull.so
online_match: /usr/lib/libpcl_surface.so
online_match: /usr/lib/libpcl_sample_consensus.so
online_match: /usr/lib/libOpenNI.so
online_match: /usr/lib/libOpenNI2.so
online_match: /usr/lib/libvtkCommon.so.5.8.0
online_match: /usr/lib/libvtkFiltering.so.5.8.0
online_match: /usr/lib/libvtkImaging.so.5.8.0
online_match: /usr/lib/libvtkGraphics.so.5.8.0
online_match: /usr/lib/libvtkGenericFiltering.so.5.8.0
online_match: /usr/lib/libvtkIO.so.5.8.0
online_match: /usr/lib/libvtkRendering.so.5.8.0
online_match: /usr/lib/libvtkVolumeRendering.so.5.8.0
online_match: /usr/lib/libvtkHybrid.so.5.8.0
online_match: /usr/lib/libvtkWidgets.so.5.8.0
online_match: /usr/lib/libvtkParallel.so.5.8.0
online_match: /usr/lib/libvtkInfovis.so.5.8.0
online_match: /usr/lib/libvtkGeovis.so.5.8.0
online_match: /usr/lib/libvtkViews.so.5.8.0
online_match: /usr/lib/libvtkCharts.so.5.8.0
online_match: /usr/lib/libpcl_io.so
online_match: /usr/lib/libpcl_filters.so
online_match: /usr/lib/libpcl_features.so
online_match: /usr/lib/libpcl_keypoints.so
online_match: /usr/lib/libpcl_registration.so
online_match: /usr/lib/libpcl_segmentation.so
online_match: /usr/lib/libpcl_recognition.so
online_match: /usr/lib/libpcl_visualization.so
online_match: /usr/lib/libpcl_people.so
online_match: /usr/lib/libpcl_outofcore.so
online_match: /usr/lib/libpcl_tracking.so
online_match: /usr/lib/libpcl_apps.so
online_match: /usr/lib/x86_64-linux-gnu/libboost_system.so
online_match: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
online_match: /usr/lib/x86_64-linux-gnu/libboost_thread.so
online_match: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
online_match: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
online_match: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
online_match: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
online_match: /usr/lib/x86_64-linux-gnu/libpthread.so
online_match: /usr/lib/x86_64-linux-gnu/libqhull.so
online_match: /usr/lib/libOpenNI.so
online_match: /usr/lib/libOpenNI2.so
online_match: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
online_match: /usr/lib/libvtkCommon.so.5.8.0
online_match: /usr/lib/libvtkFiltering.so.5.8.0
online_match: /usr/lib/libvtkImaging.so.5.8.0
online_match: /usr/lib/libvtkGraphics.so.5.8.0
online_match: /usr/lib/libvtkGenericFiltering.so.5.8.0
online_match: /usr/lib/libvtkIO.so.5.8.0
online_match: /usr/lib/libvtkRendering.so.5.8.0
online_match: /usr/lib/libvtkVolumeRendering.so.5.8.0
online_match: /usr/lib/libvtkHybrid.so.5.8.0
online_match: /usr/lib/libvtkWidgets.so.5.8.0
online_match: /usr/lib/libvtkParallel.so.5.8.0
online_match: /usr/lib/libvtkInfovis.so.5.8.0
online_match: /usr/lib/libvtkGeovis.so.5.8.0
online_match: /usr/lib/libvtkViews.so.5.8.0
online_match: /usr/lib/libvtkCharts.so.5.8.0
online_match: /usr/lib/libpcl_common.so
online_match: /usr/lib/libpcl_kdtree.so
online_match: /usr/lib/libpcl_octree.so
online_match: /usr/lib/libpcl_search.so
online_match: /usr/lib/libpcl_surface.so
online_match: /usr/lib/libpcl_sample_consensus.so
online_match: /usr/lib/libpcl_io.so
online_match: /usr/lib/libpcl_filters.so
online_match: /usr/lib/libpcl_features.so
online_match: /usr/lib/libpcl_keypoints.so
online_match: /usr/lib/libpcl_registration.so
online_match: /usr/lib/libpcl_segmentation.so
online_match: /usr/lib/libpcl_recognition.so
online_match: /usr/lib/libpcl_visualization.so
online_match: /usr/lib/libpcl_people.so
online_match: /usr/lib/libpcl_outofcore.so
online_match: /usr/lib/libpcl_tracking.so
online_match: /usr/lib/libpcl_apps.so
online_match: /usr/lib/libvtkViews.so.5.8.0
online_match: /usr/lib/libvtkInfovis.so.5.8.0
online_match: /usr/lib/libvtkWidgets.so.5.8.0
online_match: /usr/lib/libvtkVolumeRendering.so.5.8.0
online_match: /usr/lib/libvtkHybrid.so.5.8.0
online_match: /usr/lib/libvtkParallel.so.5.8.0
online_match: /usr/lib/libvtkRendering.so.5.8.0
online_match: /usr/lib/libvtkImaging.so.5.8.0
online_match: /usr/lib/libvtkGraphics.so.5.8.0
online_match: /usr/lib/libvtkIO.so.5.8.0
online_match: /usr/lib/libvtkFiltering.so.5.8.0
online_match: /usr/lib/libvtkCommon.so.5.8.0
online_match: /usr/lib/libvtksys.so.5.8.0
online_match: CMakeFiles/online_match.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable online_match"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/online_match.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/online_match.dir/build: online_match
.PHONY : CMakeFiles/online_match.dir/build

CMakeFiles/online_match.dir/requires: CMakeFiles/online_match.dir/online_match.cpp.o.requires
.PHONY : CMakeFiles/online_match.dir/requires

CMakeFiles/online_match.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/online_match.dir/cmake_clean.cmake
.PHONY : CMakeFiles/online_match.dir/clean

CMakeFiles/online_match.dir/depend:
	cd /home/lwt1104/correspondence_grouping/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lwt1104/correspondence_grouping /home/lwt1104/correspondence_grouping /home/lwt1104/correspondence_grouping/build /home/lwt1104/correspondence_grouping/build /home/lwt1104/correspondence_grouping/build/CMakeFiles/online_match.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/online_match.dir/depend

