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
include CMakeFiles/InYourGibbousPhase4.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/InYourGibbousPhase4.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/InYourGibbousPhase4.dir/flags.make

CMakeFiles/InYourGibbousPhase4.dir/src/InYourGibbousPhase4.cpp.o: CMakeFiles/InYourGibbousPhase4.dir/flags.make
CMakeFiles/InYourGibbousPhase4.dir/src/InYourGibbousPhase4.cpp.o: ../src/InYourGibbousPhase4.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/roboy/workspace/tracking_ws/src/roboy_darkroom/darkroom/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/InYourGibbousPhase4.dir/src/InYourGibbousPhase4.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/InYourGibbousPhase4.dir/src/InYourGibbousPhase4.cpp.o -c /home/roboy/workspace/tracking_ws/src/roboy_darkroom/darkroom/src/InYourGibbousPhase4.cpp

CMakeFiles/InYourGibbousPhase4.dir/src/InYourGibbousPhase4.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/InYourGibbousPhase4.dir/src/InYourGibbousPhase4.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/roboy/workspace/tracking_ws/src/roboy_darkroom/darkroom/src/InYourGibbousPhase4.cpp > CMakeFiles/InYourGibbousPhase4.dir/src/InYourGibbousPhase4.cpp.i

CMakeFiles/InYourGibbousPhase4.dir/src/InYourGibbousPhase4.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/InYourGibbousPhase4.dir/src/InYourGibbousPhase4.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/roboy/workspace/tracking_ws/src/roboy_darkroom/darkroom/src/InYourGibbousPhase4.cpp -o CMakeFiles/InYourGibbousPhase4.dir/src/InYourGibbousPhase4.cpp.s

# Object files for target InYourGibbousPhase4
InYourGibbousPhase4_OBJECTS = \
"CMakeFiles/InYourGibbousPhase4.dir/src/InYourGibbousPhase4.cpp.o"

# External object files for target InYourGibbousPhase4
InYourGibbousPhase4_EXTERNAL_OBJECTS =

libInYourGibbousPhase4.a: CMakeFiles/InYourGibbousPhase4.dir/src/InYourGibbousPhase4.cpp.o
libInYourGibbousPhase4.a: CMakeFiles/InYourGibbousPhase4.dir/build.make
libInYourGibbousPhase4.a: CMakeFiles/InYourGibbousPhase4.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/roboy/workspace/tracking_ws/src/roboy_darkroom/darkroom/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libInYourGibbousPhase4.a"
	$(CMAKE_COMMAND) -P CMakeFiles/InYourGibbousPhase4.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/InYourGibbousPhase4.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/InYourGibbousPhase4.dir/build: libInYourGibbousPhase4.a

.PHONY : CMakeFiles/InYourGibbousPhase4.dir/build

CMakeFiles/InYourGibbousPhase4.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/InYourGibbousPhase4.dir/cmake_clean.cmake
.PHONY : CMakeFiles/InYourGibbousPhase4.dir/clean

CMakeFiles/InYourGibbousPhase4.dir/depend:
	cd /home/roboy/workspace/tracking_ws/src/roboy_darkroom/darkroom/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/roboy/workspace/tracking_ws/src/roboy_darkroom/darkroom /home/roboy/workspace/tracking_ws/src/roboy_darkroom/darkroom /home/roboy/workspace/tracking_ws/src/roboy_darkroom/darkroom/cmake-build-debug /home/roboy/workspace/tracking_ws/src/roboy_darkroom/darkroom/cmake-build-debug /home/roboy/workspace/tracking_ws/src/roboy_darkroom/darkroom/cmake-build-debug/CMakeFiles/InYourGibbousPhase4.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/InYourGibbousPhase4.dir/depend

