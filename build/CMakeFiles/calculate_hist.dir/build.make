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
include CMakeFiles/calculate_hist.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/calculate_hist.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/calculate_hist.dir/flags.make

CMakeFiles/calculate_hist.dir/calculate_hist.cpp.o: CMakeFiles/calculate_hist.dir/flags.make
CMakeFiles/calculate_hist.dir/calculate_hist.cpp.o: ../calculate_hist.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lwt1104/correspondence_grouping/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/calculate_hist.dir/calculate_hist.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/calculate_hist.dir/calculate_hist.cpp.o -c /home/lwt1104/correspondence_grouping/calculate_hist.cpp

CMakeFiles/calculate_hist.dir/calculate_hist.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/calculate_hist.dir/calculate_hist.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lwt1104/correspondence_grouping/calculate_hist.cpp > CMakeFiles/calculate_hist.dir/calculate_hist.cpp.i

CMakeFiles/calculate_hist.dir/calculate_hist.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/calculate_hist.dir/calculate_hist.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lwt1104/correspondence_grouping/calculate_hist.cpp -o CMakeFiles/calculate_hist.dir/calculate_hist.cpp.s

CMakeFiles/calculate_hist.dir/calculate_hist.cpp.o.requires:
.PHONY : CMakeFiles/calculate_hist.dir/calculate_hist.cpp.o.requires

CMakeFiles/calculate_hist.dir/calculate_hist.cpp.o.provides: CMakeFiles/calculate_hist.dir/calculate_hist.cpp.o.requires
	$(MAKE) -f CMakeFiles/calculate_hist.dir/build.make CMakeFiles/calculate_hist.dir/calculate_hist.cpp.o.provides.build
.PHONY : CMakeFiles/calculate_hist.dir/calculate_hist.cpp.o.provides

CMakeFiles/calculate_hist.dir/calculate_hist.cpp.o.provides.build: CMakeFiles/calculate_hist.dir/calculate_hist.cpp.o

# Object files for target calculate_hist
calculate_hist_OBJECTS = \
"CMakeFiles/calculate_hist.dir/calculate_hist.cpp.o"

# External object files for target calculate_hist
calculate_hist_EXTERNAL_OBJECTS =

calculate_hist: CMakeFiles/calculate_hist.dir/calculate_hist.cpp.o
calculate_hist: CMakeFiles/calculate_hist.dir/build.make
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
calculate_hist: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
calculate_hist: CMakeFiles/calculate_hist.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable calculate_hist"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/calculate_hist.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/calculate_hist.dir/build: calculate_hist
.PHONY : CMakeFiles/calculate_hist.dir/build

CMakeFiles/calculate_hist.dir/requires: CMakeFiles/calculate_hist.dir/calculate_hist.cpp.o.requires
.PHONY : CMakeFiles/calculate_hist.dir/requires

CMakeFiles/calculate_hist.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/calculate_hist.dir/cmake_clean.cmake
.PHONY : CMakeFiles/calculate_hist.dir/clean

CMakeFiles/calculate_hist.dir/depend:
	cd /home/lwt1104/correspondence_grouping/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lwt1104/correspondence_grouping /home/lwt1104/correspondence_grouping /home/lwt1104/correspondence_grouping/build /home/lwt1104/correspondence_grouping/build /home/lwt1104/correspondence_grouping/build/CMakeFiles/calculate_hist.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/calculate_hist.dir/depend

