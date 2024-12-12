# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build

# Include any dependencies generated for this target.
include DeviceStartStopStreaming/CMakeFiles/DeviceStartStopStreaming.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include DeviceStartStopStreaming/CMakeFiles/DeviceStartStopStreaming.dir/compiler_depend.make

# Include the progress variables for this target.
include DeviceStartStopStreaming/CMakeFiles/DeviceStartStopStreaming.dir/progress.make

# Include the compile flags for this target's objects.
include DeviceStartStopStreaming/CMakeFiles/DeviceStartStopStreaming.dir/flags.make

DeviceStartStopStreaming/CMakeFiles/DeviceStartStopStreaming.dir/DeviceStartStopStreaming.cpp.o: DeviceStartStopStreaming/CMakeFiles/DeviceStartStopStreaming.dir/flags.make
DeviceStartStopStreaming/CMakeFiles/DeviceStartStopStreaming.dir/DeviceStartStopStreaming.cpp.o: ../DeviceStartStopStreaming/DeviceStartStopStreaming.cpp
DeviceStartStopStreaming/CMakeFiles/DeviceStartStopStreaming.dir/DeviceStartStopStreaming.cpp.o: DeviceStartStopStreaming/CMakeFiles/DeviceStartStopStreaming.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object DeviceStartStopStreaming/CMakeFiles/DeviceStartStopStreaming.dir/DeviceStartStopStreaming.cpp.o"
	cd /home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/DeviceStartStopStreaming && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT DeviceStartStopStreaming/CMakeFiles/DeviceStartStopStreaming.dir/DeviceStartStopStreaming.cpp.o -MF CMakeFiles/DeviceStartStopStreaming.dir/DeviceStartStopStreaming.cpp.o.d -o CMakeFiles/DeviceStartStopStreaming.dir/DeviceStartStopStreaming.cpp.o -c /home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/DeviceStartStopStreaming/DeviceStartStopStreaming.cpp

DeviceStartStopStreaming/CMakeFiles/DeviceStartStopStreaming.dir/DeviceStartStopStreaming.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DeviceStartStopStreaming.dir/DeviceStartStopStreaming.cpp.i"
	cd /home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/DeviceStartStopStreaming && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/DeviceStartStopStreaming/DeviceStartStopStreaming.cpp > CMakeFiles/DeviceStartStopStreaming.dir/DeviceStartStopStreaming.cpp.i

DeviceStartStopStreaming/CMakeFiles/DeviceStartStopStreaming.dir/DeviceStartStopStreaming.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DeviceStartStopStreaming.dir/DeviceStartStopStreaming.cpp.s"
	cd /home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/DeviceStartStopStreaming && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/DeviceStartStopStreaming/DeviceStartStopStreaming.cpp -o CMakeFiles/DeviceStartStopStreaming.dir/DeviceStartStopStreaming.cpp.s

# Object files for target DeviceStartStopStreaming
DeviceStartStopStreaming_OBJECTS = \
"CMakeFiles/DeviceStartStopStreaming.dir/DeviceStartStopStreaming.cpp.o"

# External object files for target DeviceStartStopStreaming
DeviceStartStopStreaming_EXTERNAL_OBJECTS =

/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/PrecompiledSamples/NYX650_Samples/DeviceStartStopStreaming: DeviceStartStopStreaming/CMakeFiles/DeviceStartStopStreaming.dir/DeviceStartStopStreaming.cpp.o
/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/PrecompiledSamples/NYX650_Samples/DeviceStartStopStreaming: DeviceStartStopStreaming/CMakeFiles/DeviceStartStopStreaming.dir/build.make
/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/PrecompiledSamples/NYX650_Samples/DeviceStartStopStreaming: DeviceStartStopStreaming/CMakeFiles/DeviceStartStopStreaming.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/PrecompiledSamples/NYX650_Samples/DeviceStartStopStreaming"
	cd /home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/DeviceStartStopStreaming && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/DeviceStartStopStreaming.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
DeviceStartStopStreaming/CMakeFiles/DeviceStartStopStreaming.dir/build: /home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/PrecompiledSamples/NYX650_Samples/DeviceStartStopStreaming
.PHONY : DeviceStartStopStreaming/CMakeFiles/DeviceStartStopStreaming.dir/build

DeviceStartStopStreaming/CMakeFiles/DeviceStartStopStreaming.dir/clean:
	cd /home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/DeviceStartStopStreaming && $(CMAKE_COMMAND) -P CMakeFiles/DeviceStartStopStreaming.dir/cmake_clean.cmake
.PHONY : DeviceStartStopStreaming/CMakeFiles/DeviceStartStopStreaming.dir/clean

DeviceStartStopStreaming/CMakeFiles/DeviceStartStopStreaming.dir/depend:
	cd /home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650 /home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/DeviceStartStopStreaming /home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build /home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/DeviceStartStopStreaming /home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/DeviceStartStopStreaming/CMakeFiles/DeviceStartStopStreaming.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : DeviceStartStopStreaming/CMakeFiles/DeviceStartStopStreaming.dir/depend
