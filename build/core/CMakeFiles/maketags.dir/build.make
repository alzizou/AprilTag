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
include core/CMakeFiles/maketags.dir/depend.make

# Include the progress variables for this target.
include core/CMakeFiles/maketags.dir/progress.make

# Include the compile flags for this target's objects.
include core/CMakeFiles/maketags.dir/flags.make

core/CMakeFiles/maketags.dir/contrib/maketags.c.o: core/CMakeFiles/maketags.dir/flags.make
core/CMakeFiles/maketags.dir/contrib/maketags.c.o: ../core/contrib/maketags.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/ali_ws/apriltag_new/apriltag/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object core/CMakeFiles/maketags.dir/contrib/maketags.c.o"
	cd /root/ali_ws/apriltag_new/apriltag/build/core && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/maketags.dir/contrib/maketags.c.o   -c /root/ali_ws/apriltag_new/apriltag/core/contrib/maketags.c

core/CMakeFiles/maketags.dir/contrib/maketags.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/maketags.dir/contrib/maketags.c.i"
	cd /root/ali_ws/apriltag_new/apriltag/build/core && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /root/ali_ws/apriltag_new/apriltag/core/contrib/maketags.c > CMakeFiles/maketags.dir/contrib/maketags.c.i

core/CMakeFiles/maketags.dir/contrib/maketags.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/maketags.dir/contrib/maketags.c.s"
	cd /root/ali_ws/apriltag_new/apriltag/build/core && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /root/ali_ws/apriltag_new/apriltag/core/contrib/maketags.c -o CMakeFiles/maketags.dir/contrib/maketags.c.s

# Object files for target maketags
maketags_OBJECTS = \
"CMakeFiles/maketags.dir/contrib/maketags.c.o"

# External object files for target maketags
maketags_EXTERNAL_OBJECTS =

maketags: core/CMakeFiles/maketags.dir/contrib/maketags.c.o
maketags: core/CMakeFiles/maketags.dir/build.make
maketags: lib/libapriltag.so
maketags: core/CMakeFiles/maketags.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/root/ali_ws/apriltag_new/apriltag/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable ../maketags"
	cd /root/ali_ws/apriltag_new/apriltag/build/core && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/maketags.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
core/CMakeFiles/maketags.dir/build: maketags

.PHONY : core/CMakeFiles/maketags.dir/build

core/CMakeFiles/maketags.dir/clean:
	cd /root/ali_ws/apriltag_new/apriltag/build/core && $(CMAKE_COMMAND) -P CMakeFiles/maketags.dir/cmake_clean.cmake
.PHONY : core/CMakeFiles/maketags.dir/clean

core/CMakeFiles/maketags.dir/depend:
	cd /root/ali_ws/apriltag_new/apriltag/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/ali_ws/apriltag_new/apriltag /root/ali_ws/apriltag_new/apriltag/core /root/ali_ws/apriltag_new/apriltag/build /root/ali_ws/apriltag_new/apriltag/build/core /root/ali_ws/apriltag_new/apriltag/build/core/CMakeFiles/maketags.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : core/CMakeFiles/maketags.dir/depend
