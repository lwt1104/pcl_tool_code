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
include CMakeFiles/pcd_recorder.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pcd_recorder.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pcd_recorder.dir/flags.make

CMakeFiles/pcd_recorder.dir/pcd_recorder.cpp.o: CMakeFiles/pcd_recorder.dir/flags.make
CMakeFiles/pcd_recorder.dir/pcd_recorder.cpp.o: ../pcd_recorder.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lwt1104/correspondence_grouping/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/pcd_recorder.dir/pcd_recorder.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/pcd_recorder.dir/pcd_recorder.cpp.o -c /home/lwt1104/correspondence_grouping/pcd_recorder.cpp

CMakeFiles/pcd_recorder.dir/pcd_recorder.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pcd_recorder.dir/pcd_recorder.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lwt1104/correspondence_grouping/pcd_recorder.cpp > CMakeFiles/pcd_recorder.dir/pcd_recorder.cpp.i

CMakeFiles/pcd_recorder.dir/pcd_recorder.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pcd_recorder.dir/pcd_recorder.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lwt1104/correspondence_grouping/pcd_recorder.cpp -o CMakeFiles/pcd_recorder.dir/pcd_recorder.cpp.s

CMakeFiles/pcd_recorder.dir/pcd_recorder.cpp.o.requires:
.PHONY : CMakeFiles/pcd_recorder.dir/pcd_recorder.cpp.o.requires

CMakeFiles/pcd_recorder.dir/pcd_recorder.cpp.o.provides: CMakeFiles/pcd_recorder.dir/pcd_recorder.cpp.o.requires
	$(MAKE) -f CMakeFiles/pcd_recorder.dir/build.make CMakeFiles/pcd_recorder.dir/pcd_recorder.cpp.o.provides.build
.PHONY : CMakeFiles/pcd_recorder.dir/pcd_recorder.cpp.o.provides

CMakeFiles/pcd_recorder.dir/pcd_recorder.cpp.o.provides.build: CMakeFiles/pcd_recorder.dir/pcd_recorder.cpp.o

# Object files for target pcd_recorder
pcd_recorder_OBJECTS = \
"CMakeFiles/pcd_recorder.dir/pcd_recorder.cpp.o"

# External object files for target pcd_recorder
pcd_recorder_EXTERNAL_OBJECTS =

pcd_recorder: CMakeFiles/pcd_recorder.dir/pcd_recorder.cpp.o
pcd_recorder: CMakeFiles/pcd_recorder.dir/build.make
pcd_recorder: /usr/lib/x86_64-linux-gnu/libboost_system.so
pcd_recorder: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
pcd_recorder: /usr/lib/x86_64-linux-gnu/libboost_thread.so
pcd_recorder: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
pcd_recorder: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
pcd_recorder: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
pcd_recorder: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
pcd_recorder: /usr/lib/x86_64-linux-gnu/libpthread.so
pcd_recorder: /usr/lib/libpcl_common.so
pcd_recorder: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
pcd_recorder: /usr/lib/libpcl_kdtree.so
pcd_recorder: /usr/lib/libpcl_octree.so
pcd_recorder: /usr/lib/libpcl_search.so
pcd_recorder: /usr/lib/x86_64-linux-gnu/libqhull.so
pcd_recorder: /usr/lib/libpcl_surface.so
pcd_recorder: /usr/lib/libpcl_sample_consensus.so
pcd_recorder: /usr/lib/libOpenNI.so
pcd_recorder: /usr/lib/libOpenNI2.so
pcd_recorder: /usr/lib/libvtkCommon.so.5.8.0
pcd_recorder: /usr/lib/libvtkFiltering.so.5.8.0
pcd_recorder: /usr/lib/libvtkImaging.so.5.8.0
pcd_recorder: /usr/lib/libvtkGraphics.so.5.8.0
pcd_recorder: /usr/lib/libvtkGenericFiltering.so.5.8.0
pcd_recorder: /usr/lib/libvtkIO.so.5.8.0
pcd_recorder: /usr/lib/libvtkRendering.so.5.8.0
pcd_recorder: /usr/lib/libvtkVolumeRendering.so.5.8.0
pcd_recorder: /usr/lib/libvtkHybrid.so.5.8.0
pcd_recorder: /usr/lib/libvtkWidgets.so.5.8.0
pcd_recorder: /usr/lib/libvtkParallel.so.5.8.0
pcd_recorder: /usr/lib/libvtkInfovis.so.5.8.0
pcd_recorder: /usr/lib/libvtkGeovis.so.5.8.0
pcd_recorder: /usr/lib/libvtkViews.so.5.8.0
pcd_recorder: /usr/lib/libvtkCharts.so.5.8.0
pcd_recorder: /usr/lib/libpcl_io.so
pcd_recorder: /usr/lib/libpcl_filters.so
pcd_recorder: /usr/lib/libpcl_features.so
pcd_recorder: /usr/lib/libpcl_keypoints.so
pcd_recorder: /usr/lib/libpcl_registration.so
pcd_recorder: /usr/lib/libpcl_segmentation.so
pcd_recorder: /usr/lib/libpcl_recognition.so
pcd_recorder: /usr/lib/libpcl_visualization.so
pcd_recorder: /usr/lib/libpcl_people.so
pcd_recorder: /usr/lib/libpcl_outofcore.so
pcd_recorder: /usr/lib/libpcl_tracking.so
pcd_recorder: /usr/lib/libpcl_apps.so
pcd_recorder: /usr/lib/x86_64-linux-gnu/libboost_system.so
pcd_recorder: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
pcd_recorder: /usr/lib/x86_64-linux-gnu/libboost_thread.so
pcd_recorder: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
pcd_recorder: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
pcd_recorder: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
pcd_recorder: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
pcd_recorder: /usr/lib/x86_64-linux-gnu/libpthread.so
pcd_recorder: /usr/lib/x86_64-linux-gnu/libqhull.so
pcd_recorder: /usr/lib/libOpenNI.so
pcd_recorder: /usr/lib/libOpenNI2.so
pcd_recorder: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
pcd_recorder: /usr/lib/libvtkCommon.so.5.8.0
pcd_recorder: /usr/lib/libvtkFiltering.so.5.8.0
pcd_recorder: /usr/lib/libvtkImaging.so.5.8.0
pcd_recorder: /usr/lib/libvtkGraphics.so.5.8.0
pcd_recorder: /usr/lib/libvtkGenericFiltering.so.5.8.0
pcd_recorder: /usr/lib/libvtkIO.so.5.8.0
pcd_recorder: /usr/lib/libvtkRendering.so.5.8.0
pcd_recorder: /usr/lib/libvtkVolumeRendering.so.5.8.0
pcd_recorder: /usr/lib/libvtkHybrid.so.5.8.0
pcd_recorder: /usr/lib/libvtkWidgets.so.5.8.0
pcd_recorder: /usr/lib/libvtkParallel.so.5.8.0
pcd_recorder: /usr/lib/libvtkInfovis.so.5.8.0
pcd_recorder: /usr/lib/libvtkGeovis.so.5.8.0
pcd_recorder: /usr/lib/libvtkViews.so.5.8.0
pcd_recorder: /usr/lib/libvtkCharts.so.5.8.0
pcd_recorder: /usr/lib/libpcl_common.so
pcd_recorder: /usr/lib/libpcl_kdtree.so
pcd_recorder: /usr/lib/libpcl_octree.so
pcd_recorder: /usr/lib/libpcl_search.so
pcd_recorder: /usr/lib/libpcl_surface.so
pcd_recorder: /usr/lib/libpcl_sample_consensus.so
pcd_recorder: /usr/lib/libpcl_io.so
pcd_recorder: /usr/lib/libpcl_filters.so
pcd_recorder: /usr/lib/libpcl_features.so
pcd_recorder: /usr/lib/libpcl_keypoints.so
pcd_recorder: /usr/lib/libpcl_registration.so
pcd_recorder: /usr/lib/libpcl_segmentation.so
pcd_recorder: /usr/lib/libpcl_recognition.so
pcd_recorder: /usr/lib/libpcl_visualization.so
pcd_recorder: /usr/lib/libpcl_people.so
pcd_recorder: /usr/lib/libpcl_outofcore.so
pcd_recorder: /usr/lib/libpcl_tracking.so
pcd_recorder: /usr/lib/libpcl_apps.so
pcd_recorder: /usr/lib/libvtkViews.so.5.8.0
pcd_recorder: /usr/lib/libvtkInfovis.so.5.8.0
pcd_recorder: /usr/lib/libvtkWidgets.so.5.8.0
pcd_recorder: /usr/lib/libvtkVolumeRendering.so.5.8.0
pcd_recorder: /usr/lib/libvtkHybrid.so.5.8.0
pcd_recorder: /usr/lib/libvtkParallel.so.5.8.0
pcd_recorder: /usr/lib/libvtkRendering.so.5.8.0
pcd_recorder: /usr/lib/libvtkImaging.so.5.8.0
pcd_recorder: /usr/lib/libvtkGraphics.so.5.8.0
pcd_recorder: /usr/lib/libvtkIO.so.5.8.0
pcd_recorder: /usr/lib/libvtkFiltering.so.5.8.0
pcd_recorder: /usr/lib/libvtkCommon.so.5.8.0
pcd_recorder: /usr/lib/libvtksys.so.5.8.0
pcd_recorder: CMakeFiles/pcd_recorder.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable pcd_recorder"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pcd_recorder.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pcd_recorder.dir/build: pcd_recorder
.PHONY : CMakeFiles/pcd_recorder.dir/build

CMakeFiles/pcd_recorder.dir/requires: CMakeFiles/pcd_recorder.dir/pcd_recorder.cpp.o.requires
.PHONY : CMakeFiles/pcd_recorder.dir/requires

CMakeFiles/pcd_recorder.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pcd_recorder.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pcd_recorder.dir/clean

CMakeFiles/pcd_recorder.dir/depend:
	cd /home/lwt1104/correspondence_grouping/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lwt1104/correspondence_grouping /home/lwt1104/correspondence_grouping /home/lwt1104/correspondence_grouping/build /home/lwt1104/correspondence_grouping/build /home/lwt1104/correspondence_grouping/build/CMakeFiles/pcd_recorder.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pcd_recorder.dir/depend

