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
CMAKE_COMMAND = /opt/clion-2018.3.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /opt/clion-2018.3.4/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/roboy/workspace/tracking_ws/src/roboy_darkroom/darkroom

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/roboy/workspace/tracking_ws/src/roboy_darkroom/darkroom/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/Transform.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Transform.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Transform.dir/flags.make

CMakeFiles/Transform.dir/src/Transform.cpp.o: CMakeFiles/Transform.dir/flags.make
CMakeFiles/Transform.dir/src/Transform.cpp.o: ../src/Transform.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/roboy/workspace/tracking_ws/src/roboy_darkroom/darkroom/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Transform.dir/src/Transform.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Transform.dir/src/Transform.cpp.o -c /home/roboy/workspace/tracking_ws/src/roboy_darkroom/darkroom/src/Transform.cpp

CMakeFiles/Transform.dir/src/Transform.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Transform.dir/src/Transform.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/roboy/workspace/tracking_ws/src/roboy_darkroom/darkroom/src/Transform.cpp > CMakeFiles/Transform.dir/src/Transform.cpp.i

CMakeFiles/Transform.dir/src/Transform.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Transform.dir/src/Transform.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/roboy/workspace/tracking_ws/src/roboy_darkroom/darkroom/src/Transform.cpp -o CMakeFiles/Transform.dir/src/Transform.cpp.s

# Object files for target Transform
Transform_OBJECTS = \
"CMakeFiles/Transform.dir/src/Transform.cpp.o"

# External object files for target Transform
Transform_EXTERNAL_OBJECTS =

libTransform.a: CMakeFiles/Transform.dir/src/Transform.cpp.o
libTransform.a: CMakeFiles/Transform.dir/build.make
libTransform.a: CMakeFiles/Transform.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/roboy/workspace/tracking_ws/src/roboy_darkroom/darkroom/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libTransform.a"
	$(CMAKE_COMMAND) -P CMakeFiles/Transform.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Transform.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Transform.dir/build: libTransform.a

.PHONY : CMakeFiles/Transform.dir/build

CMakeFiles/Transform.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Transform.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Transform.dir/clean

CMakeFiles/Transform.dir/depend:
	cd /home/roboy/workspace/tracking_ws/src/roboy_darkroom/darkroom/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/roboy/workspace/tracking_ws/src/roboy_darkroom/darkroom /home/roboy/workspace/tracking_ws/src/roboy_darkroom/darkroom /home/roboy/workspace/tracking_ws/src/roboy_darkroom/darkroom/cmake-build-debug /home/roboy/workspace/tracking_ws/src/roboy_darkroom/darkroom/cmake-build-debug /home/roboy/workspace/tracking_ws/src/roboy_darkroom/darkroom/cmake-build-debug/CMakeFiles/Transform.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Transform.dir/depend

