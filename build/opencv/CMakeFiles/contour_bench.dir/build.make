# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_SOURCE_DIR = /root/ali_ws/apriltag_new/apriltag

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /root/ali_ws/apriltag_new/apriltag/build

# Include any dependencies generated for this target.
include opencv/CMakeFiles/contour_bench.dir/depend.make

# Include the progress variables for this target.
include opencv/CMakeFiles/contour_bench.dir/progress.make

# Include the compile flags for this target's objects.
include opencv/CMakeFiles/contour_bench.dir/flags.make

opencv/CMakeFiles/contour_bench.dir/contour_bench.cpp.o: opencv/CMakeFiles/contour_bench.dir/flags.make
opencv/CMakeFiles/contour_bench.dir/contour_bench.cpp.o: ../opencv/contour_bench.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/ali_ws/apriltag_new/apriltag/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object opencv/CMakeFiles/contour_bench.dir/contour_bench.cpp.o"
	cd /root/ali_ws/apriltag_new/apriltag/build/opencv && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/contour_bench.dir/contour_bench.cpp.o -c /root/ali_ws/apriltag_new/apriltag/opencv/contour_bench.cpp

opencv/CMakeFiles/contour_bench.dir/contour_bench.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/contour_bench.dir/contour_bench.cpp.i"
	cd /root/ali_ws/apriltag_new/apriltag/build/opencv && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/ali_ws/apriltag_new/apriltag/opencv/contour_bench.cpp > CMakeFiles/contour_bench.dir/contour_bench.cpp.i

opencv/CMakeFiles/contour_bench.dir/contour_bench.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/contour_bench.dir/contour_bench.cpp.s"
	cd /root/ali_ws/apriltag_new/apriltag/build/opencv && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/ali_ws/apriltag_new/apriltag/opencv/contour_bench.cpp -o CMakeFiles/contour_bench.dir/contour_bench.cpp.s

# Object files for target contour_bench
contour_bench_OBJECTS = \
"CMakeFiles/contour_bench.dir/contour_bench.cpp.o"

# External object files for target contour_bench
contour_bench_EXTERNAL_OBJECTS =

contour_bench: opencv/CMakeFiles/contour_bench.dir/contour_bench.cpp.o
contour_bench: opencv/CMakeFiles/contour_bench.dir/build.make
contour_bench: lib/libapriltag_opencv.so
contour_bench: lib/libapriltag.so
contour_bench: opencv/CMakeFiles/contour_bench.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/root/ali_ws/apriltag_new/apriltag/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../contour_bench"
	cd /root/ali_ws/apriltag_new/apriltag/build/opencv && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/contour_bench.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
opencv/CMakeFiles/contour_bench.dir/build: contour_bench

.PHONY : opencv/CMakeFiles/contour_bench.dir/build

opencv/CMakeFiles/contour_bench.dir/clean:
	cd /root/ali_ws/apriltag_new/apriltag/build/opencv && $(CMAKE_COMMAND) -P CMakeFiles/contour_bench.dir/cmake_clean.cmake
.PHONY : opencv/CMakeFiles/contour_bench.dir/clean

opencv/CMakeFiles/contour_bench.dir/depend:
	cd /root/ali_ws/apriltag_new/apriltag/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/ali_ws/apriltag_new/apriltag /root/ali_ws/apriltag_new/apriltag/opencv /root/ali_ws/apriltag_new/apriltag/build /root/ali_ws/apriltag_new/apriltag/build/opencv /root/ali_ws/apriltag_new/apriltag/build/opencv/CMakeFiles/contour_bench.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : opencv/CMakeFiles/contour_bench.dir/depend

