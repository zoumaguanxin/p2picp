# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

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
CMAKE_SOURCE_DIR = /home/shaoan/projects/ICRP

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/shaoan/projects/ICRP/build

# Include any dependencies generated for this target.
include CMakeFiles/ICRP.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ICRP.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ICRP.dir/flags.make

CMakeFiles/ICRP.dir/main.cpp.o: CMakeFiles/ICRP.dir/flags.make
CMakeFiles/ICRP.dir/main.cpp.o: ../main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/shaoan/projects/ICRP/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/ICRP.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/ICRP.dir/main.cpp.o -c /home/shaoan/projects/ICRP/main.cpp

CMakeFiles/ICRP.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ICRP.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/shaoan/projects/ICRP/main.cpp > CMakeFiles/ICRP.dir/main.cpp.i

CMakeFiles/ICRP.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ICRP.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/shaoan/projects/ICRP/main.cpp -o CMakeFiles/ICRP.dir/main.cpp.s

CMakeFiles/ICRP.dir/main.cpp.o.requires:
.PHONY : CMakeFiles/ICRP.dir/main.cpp.o.requires

CMakeFiles/ICRP.dir/main.cpp.o.provides: CMakeFiles/ICRP.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/ICRP.dir/build.make CMakeFiles/ICRP.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/ICRP.dir/main.cpp.o.provides

CMakeFiles/ICRP.dir/main.cpp.o.provides.build: CMakeFiles/ICRP.dir/main.cpp.o

# Object files for target ICRP
ICRP_OBJECTS = \
"CMakeFiles/ICRP.dir/main.cpp.o"

# External object files for target ICRP
ICRP_EXTERNAL_OBJECTS =

ICRP: CMakeFiles/ICRP.dir/main.cpp.o
ICRP: CMakeFiles/ICRP.dir/build.make
ICRP: /usr/lib/x86_64-linux-gnu/libboost_system.so
ICRP: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
ICRP: /usr/lib/x86_64-linux-gnu/libboost_thread.so
ICRP: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
ICRP: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
ICRP: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
ICRP: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
ICRP: /usr/lib/x86_64-linux-gnu/libpthread.so
ICRP: /usr/lib/libpcl_common.so
ICRP: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
ICRP: /usr/lib/libpcl_kdtree.so
ICRP: /usr/lib/libpcl_octree.so
ICRP: /usr/lib/libpcl_search.so
ICRP: /usr/lib/x86_64-linux-gnu/libqhull.so
ICRP: /usr/lib/libpcl_surface.so
ICRP: /usr/lib/libpcl_sample_consensus.so
ICRP: /usr/lib/libOpenNI.so
ICRP: /usr/lib/libOpenNI2.so
ICRP: /usr/lib/libpcl_io.so
ICRP: /usr/lib/libpcl_filters.so
ICRP: /usr/lib/libpcl_features.so
ICRP: /usr/lib/libpcl_keypoints.so
ICRP: /usr/lib/libpcl_registration.so
ICRP: /usr/lib/libpcl_segmentation.so
ICRP: /usr/lib/libpcl_recognition.so
ICRP: /usr/lib/libpcl_visualization.so
ICRP: /usr/lib/libpcl_people.so
ICRP: /usr/lib/libpcl_outofcore.so
ICRP: /usr/lib/libpcl_tracking.so
ICRP: /usr/lib/libpcl_apps.so
ICRP: /usr/lib/x86_64-linux-gnu/libboost_system.so
ICRP: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
ICRP: /usr/lib/x86_64-linux-gnu/libboost_thread.so
ICRP: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
ICRP: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
ICRP: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
ICRP: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
ICRP: /usr/lib/x86_64-linux-gnu/libpthread.so
ICRP: /usr/lib/x86_64-linux-gnu/libqhull.so
ICRP: /usr/lib/libOpenNI.so
ICRP: /usr/lib/libOpenNI2.so
ICRP: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
ICRP: /usr/lib/libvtkGenericFiltering.so.5.8.0
ICRP: /usr/lib/libvtkGeovis.so.5.8.0
ICRP: /usr/lib/libvtkCharts.so.5.8.0
ICRP: /usr/lib/libpcl_common.so
ICRP: /usr/lib/libpcl_kdtree.so
ICRP: /usr/lib/libpcl_octree.so
ICRP: /usr/lib/libpcl_search.so
ICRP: /usr/lib/libpcl_surface.so
ICRP: /usr/lib/libpcl_sample_consensus.so
ICRP: /usr/lib/libpcl_io.so
ICRP: /usr/lib/libpcl_filters.so
ICRP: /usr/lib/libpcl_features.so
ICRP: /usr/lib/libpcl_keypoints.so
ICRP: /usr/lib/libpcl_registration.so
ICRP: /usr/lib/libpcl_segmentation.so
ICRP: /usr/lib/libpcl_recognition.so
ICRP: /usr/lib/libpcl_visualization.so
ICRP: /usr/lib/libpcl_people.so
ICRP: /usr/lib/libpcl_outofcore.so
ICRP: /usr/lib/libpcl_tracking.so
ICRP: /usr/lib/libpcl_apps.so
ICRP: /usr/lib/libvtkViews.so.5.8.0
ICRP: /usr/lib/libvtkInfovis.so.5.8.0
ICRP: /usr/lib/libvtkWidgets.so.5.8.0
ICRP: /usr/lib/libvtkVolumeRendering.so.5.8.0
ICRP: /usr/lib/libvtkHybrid.so.5.8.0
ICRP: /usr/lib/libvtkParallel.so.5.8.0
ICRP: /usr/lib/libvtkRendering.so.5.8.0
ICRP: /usr/lib/libvtkImaging.so.5.8.0
ICRP: /usr/lib/libvtkGraphics.so.5.8.0
ICRP: /usr/lib/libvtkIO.so.5.8.0
ICRP: /usr/lib/libvtkFiltering.so.5.8.0
ICRP: /usr/lib/libvtkCommon.so.5.8.0
ICRP: /usr/lib/libvtksys.so.5.8.0
ICRP: CMakeFiles/ICRP.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ICRP"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ICRP.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ICRP.dir/build: ICRP
.PHONY : CMakeFiles/ICRP.dir/build

CMakeFiles/ICRP.dir/requires: CMakeFiles/ICRP.dir/main.cpp.o.requires
.PHONY : CMakeFiles/ICRP.dir/requires

CMakeFiles/ICRP.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ICRP.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ICRP.dir/clean

CMakeFiles/ICRP.dir/depend:
	cd /home/shaoan/projects/ICRP/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/shaoan/projects/ICRP /home/shaoan/projects/ICRP /home/shaoan/projects/ICRP/build /home/shaoan/projects/ICRP/build /home/shaoan/projects/ICRP/build/CMakeFiles/ICRP.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ICRP.dir/depend

